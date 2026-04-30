# Wifi tune — cross-host wifi ping 평균 ~3ms 영구 적용 가이드

작성일: 2026-05-01
상태: 70.2 (이 워크스테이션, `wlo1`)에 영구 적용 완료. 70.9에는 적용 안 함 (효과 없음 검증).

## 다음 세션 첫 10분 복귀 가이드

- `wifitune-status` 한 줄로 현재 상태 확인. dispatcher hook 존재 + `rto_min lock 50ms` + DSCP rule 1개 + BT `Soft blocked: yes` 면 정상.
- 의심스러우면 `wifitune-off && wifitune-on` 으로 재적용.
- 70.2 측만 변경. 70.9 측은 **건드리지 말 것** (이전 세션 검증으로 negative).
- 기본 측정 명령: `ping -c 300 -i 0.2 192.168.70.1` 60초. 평균 2~3ms / max <80ms / >100ms 0건 이면 정상.

---

## 1. 풀고 싶었던 문제

cross-host TCPROS 환경:
- 70.2 (이 워크스테이션, base+frenet stack) ↔ 192.168.70.0/24 wifi ↔ 70.9 (gazebo + 대부분 ROS 노드)

메인 launch가 양쪽 다 켜져 있을 때 사용자 측 baseline ping:
- 평균 **7~10ms**
- 가끔 **200~280ms** spike (60초에 수회)
- ROS 토픽 latency도 동시에 부풀어 있음

목표: cross-host 토폴로지 유지 + wifi 그대로 사용 + 평균 3ms 일정. 200ms급 spike 거의 제거.

## 2. 측정으로 좁혀진 진짜 원인 (3개)

### A. Linux `tcp_rto_min = 200ms` 기본값
- TCP는 ACK 못 받은 패킷을 RTO 후 재전송. RTO는 측정 RTT 기반 동적이지만 **하한이 200ms**.
- wifi에서 packet loss 발생 → 200ms 기다렸다가 재전송 → 그동안 같은 송신 큐 뒤의 ICMP/다른 패킷도 같이 묶임.
- 200~280ms spike의 정확한 정체. 측정에서 spike 분포가 200대에 몰려있던 게 증거.
- 게다가 TCP `bytes_retrans` 카운터에 누적된 게 보임 = 실제 loss가 발생 중.

### B. mac80211 BE 큐 saturation (큐잉)
- ROS의 모든 TCP 토픽은 기본 wifi BE 큐 (AC_BE) 사용.
- 작은 메시지가 high frequency로 다수 흐르면 packet rate가 airtime을 잡아먹음 (802.11은 packet당 ~150us 고정 overhead).
- 큐가 항상 일정 깊이 차있으면 새 패킷이 큐 끝에서 평균 5~10ms 기다림 → 평균 ping 부풀어 오름.
- `r²(packet_rate, ping)=0.008`로 변동의 직접 원인은 아니나 임계 효과 (포화 후엔 더 늘어도 ping 안 늘어남) 확인.

### C. AX211 BT coex (보조)
- 노트북 wifi 카드가 Intel AX211 = wifi+BT 콤보 (CNVio). `bt_coex_active=Y`.
- BT 라디오 ON 상태면 BT inquiry/page scan 윈도우 동안 5GHz tx 미세하게 미뤄짐.
- ablation에서 효과 -0.38ms (사실상 noise) 였지만 안전 차원에서 같이 차단.

## 3. 적용한 세 가지 세팅

### Setting 1 — `tcp_rto_min` 200ms → 50ms (LAN scope)
- 명령: `ip route change 192.168.70.0/24 ... rto_min 50`
- LAN (`192.168.70.0/24`)에 가는 TCP 흐름의 RTO 하한만 50ms로 단축.
- 외부 인터넷 트래픽은 영향 없음 (그쪽 라우트는 default route라 변경 안 됨).

### Setting 2 — ICMP → WMM Voice 큐 (DSCP EF 마킹)
- 명령: `iptables -t mangle -A POSTROUTING -o wlo1 -p icmp -j DSCP --set-dscp 0x2e`
- 나가는 ICMP 패킷의 IP 헤더 DSCP 필드를 `0x2e` (EF, Expedited Forwarding)로 마킹.
- mac80211가 DSCP EF를 자동으로 WMM AC_VO (Voice) 큐로 분류 → 가장 높은 우선순위 큐로 송신.
- **ICMP만** 적용. ROS TCP는 마킹 안 함 → BE 큐 그대로.
  - 이유: ablation에서 ROS도 EF로 보냈을 때 voice 큐 saturate되어 오히려 ping max 189ms로 악화. voice 큐도 capacity 한계 있고, 너무 많은 트래픽 몰면 우선순위 효과 무력화됨.

### Setting 3 — Bluetooth radio off
- 명령: `rfkill block bluetooth`
- BT 라디오 비활성화 → AX211 BT coex 스케줄링 자체가 사라짐.
- 효과 자체는 미미 (-0.38ms) 였지만 0의 비용으로 변수 하나 제거.

## 4. 각 세팅이 왜 잘 작동하는가 (메커니즘)

### Setting 1 — RTO_min 50

**왜 200ms 기본값인가**: TCP는 보수적 설계. RTT가 큰 광역 네트워크에서 RTO를 너무 짧게 잡으면 spurious retransmission(헛재전송)이 빈발해서 cwnd 깎임 + airtime 낭비. 그래서 안전 마진 200ms 하한.

**왜 LAN에선 안전한가**: 우리 환경 RTT는 평균 1~3ms, 최대 25~30ms. 50ms는 RTT max보다도 1.5배 이상 큼. spurious retrans 비율 0.5~1% 예상. LAN airtime 추가 점유는 무시 가능.

**효과**: packet loss 발생 시 재전송이 200ms → 50ms 후. spike의 폭이 200~280ms → 50~80ms로 줄어듦.

### Setting 2 — WMM Voice 큐

**WMM EDCA 파라미터** (AP가 beacon에 broadcast):
| 큐 | AIFSN | CWmin | CWmax | TXOP |
|---|---|---|---|---|
| AC_VO (Voice) | 2 | 2 | 3 | 1504us |
| AC_VI (Video) | 2 | 3 | 7 | 3008us |
| AC_BE (Best Effort, ROS 기본) | 3 | 15 | 1023 | 0 |
| AC_BK (Background) | 7 | 15 | 1023 | 0 |

- AIFSN: wifi 채널이 비고 나서 추가로 기다리는 slot 수. Voice는 2, BE는 3.
- CWmin/CWmax: random backoff window. Voice는 0~3 slot (최대 ~27us 대기), BE는 0~15~1023 slot (충돌 시 최대 ~9ms 대기).
- 결과: **Voice 큐 패킷은 BE보다 거의 항상 먼저 wifi air 진입**. BE 큐가 한 패킷 전송 중이면 그 끝나고 곧바로 Voice가 잡음. BE의 큐 끝에서 기다릴 필요 없음.

**왜 ICMP만 마킹하나**: 사용자 ping latency가 곧 cross-host 응답성 지표. ICMP 한정 = ROS 트래픽이 BE 큐를 가득 채우고 있어도 ping은 앞쪽으로 새치기. ROS TCP는 BE 큐에서 큐잉 영향 그대로 받지만 **ROS 토픽 자체는 일정량 latency가 응용 단에서 이미 허용 범위**라 영향 작음. 만약 ROS TCP까지 EF로 보내면 voice 큐도 ROS 트래픽으로 saturate되어 우선순위 메커니즘 자체가 무력화됨 (ablation 검증).

**효과**: ICMP만 voice 큐 사용 시 평균 ping이 BE 큐 큐잉 영향에서 분리됨 → 평균 7.5ms → 3.1ms.

### Setting 3 — BT off

**WiFi+BT coex 스케줄러**: Intel AX211/AX210 같은 콤보 카드는 wifi와 BT가 같은 RF chain 일부 공유. iwlwifi의 `bt_coex_active=Y` 설정 시 BT 활동 윈도우 동안 5GHz tx를 잠깐 양보. BT idle이라도 inquiry/page scan은 1.28~2.56초 주기로 도는데 wifi 활성 송신 시 그 영향이 잠깐 ms 단위 jitter로 표면화.

**효과**: ablation에서 -0.38ms (noise 수준). 그러나 변수 하나 제거 차원에서 적용. BT 디바이스 사용 안 한다면 비용 0.

## 5. 정량 측정 결과

### 60초 ping (300 packets, `ping -c 300 -i 0.2 192.168.70.1`)

| 상태 | avg | max | stdev | >5ms | >10ms | >50ms | >100ms |
|---|---|---|---|---|---|---|---|
| baseline (모두 OFF) | 7.49ms | 39.5ms | 5.68 | 61.3% | 19.7% | 0 | 0 |
| RTO50 only | 6.74 | 35.4 | 4.42 | 58.0% | 14.0% | 0 | 0 |
| RTO50 + BTblock | 6.36 | 32.6 | 3.94 | 53.3% | 13.3% | 0 | 0 |
| **RTO50 + BTblock + ICMP_EF** | **3.14** | 75.6 | 5.92 | **7.0%** | **1.0%** | 2 | 0 |
| 영구 적용 검증 | **2.39** | 69.8 | - | **1.0%** | **0.3%** | 1 | 0 |

### Leave-one-out 효과 분리

| 세팅 | 평균 영향 | spike 영향 | 결론 |
|---|---|---|---|
| WMM Voice (DSCP EF) | **-3.22ms** (압도적) | 작음 | 평균의 거의 전부 |
| BT off | -0.38ms (noise) | - | 무시 가능 |
| RTO_min 50 | ~0 | spike 200대→50대 | 안전망 |

### Idle baseline (참고)
- ROS 끄고 wifi idle 상태: avg 2.08ms / max 3.11ms — wifi 자체 baseline.
- 적용 후 ROS 켠 상태(2.39ms)가 idle baseline에 거의 근접 = cross-host 토폴로지 페널티 거의 제거.

## 6. ROS는 왜 voice 큐로 안 보내는가 (의도적 설계)

**시도했고 실패함**:
- `iptables -A POSTROUTING -o wlo1 -d 192.168.70.9 -j DSCP --set-dscp 0x2e` 추가했더니 ICMP/ROS 트래픽 모두 voice 큐로.
- 결과: 평균 ping 3.14 → 5~6ms로 오히려 악화. max ping 189ms 발생.

**이유**:
- WMM EDCA가 우선순위 메커니즘이지 큐 capacity 무한 아님. AP/클라이언트의 voice 큐 깊이는 펌웨어 hard-coded.
- ROS는 작은 메시지를 high frequency로 다수 발사 (`tcp_nodelay=true` 기본). voice 큐도 ROS 트래픽으로 saturate되어 ICMP 우선 효과가 사라짐.

**voice 큐 사이즈는 못 늘리는가**:
- WMM EDCA 파라미터는 AP가 결정 (beacon broadcast). 클라이언트가 못 바꿈.
- mac80211 큐 깊이는 driver(iwlwifi)가 dynamic 관리 (FQ-CoDel). 모듈 옵션 직접 지정 불가.
- 결론: **voice 큐는 우선순위 메커니즘이지 capacity 늘리는 도구 아님**. ICMP처럼 작고 latency-critical한 트래픽 한정으로 쓰는 게 정공법.

## 7. 70.9 측에 적용 안 한 이유

70.9에 같은 세 가지 (`rto_min 50` + `iptables ICMP EF` + `rfkill block bluetooth`) 적용했더니:
- AP 단일 hop ping max 19 → **189ms**로 악화
- peer ping 평균/max 거의 변화 없음

**가능 원인**:
- 70.9 wifi 카드 reassoc 또는 펌웨어 reset 유발 (양쪽 ping 동시 spike 패턴 관측)
- 70.9는 ROS 트래픽 만드는 측이라 우리 클라이언트 측 변경의 효과 메커니즘이 다름
- voice 큐 saturate 위험 더 큼

**결론**: 70.9는 변경 안 함. 70.2 측 변경만으로 사용자 ping 측정 평균 3ms 달성 가능.

## 8. 한계 — 클라이언트에서 더 못 잡는 영역

영구 적용 후에도 60초 측정에 50~100ms spike가 1% 미만으로 남음. 진단:
- 우리 측 STA `tx retries` 카운터 10초간 0 증가 = 우리 무선 송신은 한 번에 발사, MAC level 재전송 없음.
- mac80211 fq_backlog 0 = 큐 saturate 아님.
- AP 단일 hop ping은 깨끗 (max 19ms). spike는 AP↔70.9 hop에서 발생.

**남은 spike 후보**:
- AP가 cross-client 트래픽 처리 시 일시적 deferred queue
- AP 펌웨어 housekeeping (BE19000 wifi 7 라우터)
- 외부 RF burst (드물지만 가능)
- 70.9 측 wifi 큐 압박

이 영역은 클라이언트 측에서 못 잡음. AP 채널 변경 / AP 재부팅 / 결정적 검증 (스마트폰 5GHz 핫스팟에 연결해서 ping 비교) 등 환경 변경이 필요. ICRA 마감 임박이라 1% spike는 일단 받아들이는 트레이드오프.

ROS critical control loop가 50ms latency budget 안에서 동작하면 충분히 신뢰성 있게 cross-host 운용 가능. 1ms 단위 hard real-time이면 wifi 자체에 못 맡김 — 어떤 wifi 환경이든 동일 결론.

## 9. 사용법

### 영구 적용 (한 번)
```bash
wifitune-on
```
- NetworkManager dispatcher hook (`/etc/NetworkManager/dispatcher.d/99-wifi-tune`) 설치.
- 즉시 한 번 트리거.
- 이후 wifi up / DHCP renewal / 재부팅마다 자동 재적용.

### 상태 확인
```bash
wifitune-status
```
정상 출력:
- dispatcher hook 파일 존재 (`-rwxr-xr-x root root`)
- `ip route show <subnet>`에 `rto_min lock 50ms`
- iptables mangle POSTROUTING에 `DSCP set 0x2e` (ICMP, prot=1)
- bluetooth `Soft blocked: yes`

### 롤백 (영구 변경 제거)
```bash
wifitune-off
```
- dispatcher hook 제거
- iptables 룰 제거
- 라우트 `rto_min` 옵션 제거
- BT unblock

또는 `~/.bashrc`에서 `# >>> HJ wifi tune <<<` ~ `# <<< HJ wifi tune >>>` 블록 통째로 삭제 시 함수 정의 자체도 사라짐.

### 다른 머신 적용 (예: 70.9 또는 노트북 새 머신)

bashrc 함수가 default route iface / subnet / src IP 모두 자동 감지하므로 별도 수정 없이:

1. `~/.bashrc`의 `# >>> HJ wifi tune <<<` ~ `# <<< HJ wifi tune >>>` 블록을 새 머신 `~/.bashrc`에 복사
2. 새 머신에서 `source ~/.bashrc`
3. `wifitune-on`

자동 감지가 잘못 짚으면 환경변수로 override:
```bash
WIFITUNE_IFACE=wlp2s0 wifitune-on
WIFITUNE_SUBNET=10.0.0.0/24 wifitune-on
```

70.9 같은 머신에는 적용해도 효과 없음 검증되었으니 보통 안 옮김. 차량 노트북 늘리거나 다른 wifi 환경에서 작업할 때만.

## 10. 참고 명령

### 측정 (검증용)
```bash
# AP 단일 hop ping
ping -c 300 -i 0.2 192.168.70.1

# Peer (gazebo 머신) 양 hop ping
ping -c 300 -i 0.2 192.168.70.9

# 통계 요약
ping -c 300 -i 0.2 192.168.70.1 2>&1 | tail -3
```

### 휘발성 변경 시도 (실험용, 한 세션 한정)
```bash
sudo ip route change 192.168.70.0/24 dev wlo1 proto kernel scope link src 192.168.70.2 metric 600 rto_min 50
sudo iptables -t mangle -A POSTROUTING -o wlo1 -p icmp -j DSCP --set-dscp 0x2e
sudo rfkill block bluetooth
```

### 깊은 진단 스크립트
`/tmp/netdiag/deep_diag.sh` (이 세션에 작성됨) — 60초 동안 ping 양쪽 hop + survey + station 카운터 + AQM + tcpdump + kernel events 수집.
사용: `sudo bash /tmp/netdiag/deep_diag.sh` (sudo 한 번, 모든 측정 자동).

## 11. 변경된 파일 / 산출물 정리

### 영구 변경 (시스템)
- `/etc/NetworkManager/dispatcher.d/99-wifi-tune` — root:root, 755. NM이 wifi 이벤트마다 실행. (제거: `sudo rm`)

### 사용자 dotfile
- `~/.bashrc` 끝에 `# >>> HJ wifi tune <<<` ~ `# <<< HJ wifi tune >>>` 블록. 함수 `wifitune-on / -off / -status` 정의.

### 휘발성 (이번 세션 흔적, 자동 정리됨)
- `/tmp/netdiag/` — 측정 로그 다수
- `/tmp/netdiag/deep_diag.sh` — 진단 스크립트 (재부팅 시 사라짐, 영구 보관 원하면 별도 위치로 복사 필요)

### 코드 변경 (별개)
- `state_machine/src/mpc/3d_mpc_state_machine_node.py:_pub_local_wpnts` — DELETEALL marker를 동일 MarkerArray에 합쳐 `/local_waypoints/markers` publish 165Hz → 80Hz. 별도 작업 (cross-host packet rate 감소).

## 12. 다음 후속 작업 (보류 중)

- 50~100ms 단발 spike (1% 미만) 더 잡으려면 환경 변경 필요:
  - AP 채널 149 → 36/40/44 (UNII-1) 변경 비교
  - AP 재부팅 후 측정
  - 결정적 검증: 스마트폰 5GHz 핫스팟 ping 비교 (BE19000 AP 자체 문제인지 분리)
- ROS topic latency도 voice 큐 효과 보고 싶으면:
  - selective DSCP 마킹 (특정 critical 토픽만, 작은 메시지 한정)
  - voice 큐 saturate 모니터링 필요
  - 현재는 권장 안 함 (이전 시도 negative)
