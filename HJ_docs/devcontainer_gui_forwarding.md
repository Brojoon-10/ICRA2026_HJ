# Devcontainer GUI Forwarding 문제 해결

## 1. 문제 상황

여러 SSH 터미널에서 `icra2026` 컨테이너에 들어가서 GUI 앱(rviz, xclock, xeyes 등)을 띄우려 하면, 일부 터미널에서만 뜨고 다른 터미널에서는 실패:

```
X11 connection rejected because of wrong authentication.
Error: Can't open display: localhost:14.0
```

**원하던 동작**: 컨테이너 재시작 없이, 현재 접속한 모든 SSH 세션에 대해 컨테이너 GUI 권한을 일괄 부여.

---

## 2. X11 GUI 동작 원리

GUI가 뜨려면 세 가지가 동시에 맞아야 한다.

### (a) X 서버 ACL
- 클라이언트 X 서버가 "누구의 연결을 받을지" 정하는 정책.
- `xhost +local:$USER` → 로컬 유저 허용. 한 번 설정되면 X 서버 세션 동안 유효.
- devcontainer는 시작 시 [.devcontainer/.install_utils/xauth_setup.sh](../.devcontainer/.install_utils/xauth_setup.sh)에서 자동 실행.

### (b) Magic Cookie 인증
- `~/.Xauthority` 파일에 저장된 비밀 키.
- X 클라이언트가 이 cookie를 제시해야 X 서버가 연결 수락.
- **cookie는 DISPLAY 번호별로 분리됨** (`:10`, `:11`, `:14` 각각 다른 cookie).

### (c) DISPLAY 연결 채널
- `DISPLAY=localhost:14.0` = "서버 loopback TCP 6014 포트로 X 연결을 보내라".
- SSH `-X` 옵션이 이 TCP 포트를 클라이언트 X 서버까지 터널링.

### SSH `-X` 접속 시 자동으로 일어나는 일

1. sshd가 빈 DISPLAY 번호 할당 (예: `:14`).
2. 서버 `localhost:6014`에 TCP 리슨 시작 (터널 입구).
3. 세션용 fake cookie 생성 → 서버 `~/.Xauthority`에 `unicorn/unix:14` 형식으로 추가.
4. 세션 shell에 `DISPLAY=localhost:14.0` env 세팅.

세션이 끊기면 (2), (4)는 사라지지만 (3)의 cookie는 `~/.Xauthority`에 누적됨.

---

## 3. 기존 설정의 구조

### [devcontainer.json](../.devcontainer/devcontainer.json) 관련 부분

```json
"containerEnv": {
    "DISPLAY": "${localEnv:DISPLAY}",        // 컨테이너 기본 DISPLAY
    "XAUTHORITY": "/tmp/.Xauthority"          // 컨테이너 xauth DB 경로
},
"mounts": [
    "source=/home/${localEnv:USER}/.Xauthority,target=/tmp/.Xauthority,type=bind,...",
    "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,..."
]
```

### 기존 `~/.bashrc` alias

```bash
alias icra_unicorn='docker exec -it -e DISPLAY="$DISPLAY" icra2026 bash'
```

이 alias는 **현재 SSH 세션의 `$DISPLAY`를 컨테이너에 덮어씌워 주입**하므로, 다중 SSH 터미널별 DISPLAY 분리는 정상 작동.

---

## 4. 실제 병목 — Xauthority bind mount stale

### 증상

- 호스트 `~/.Xauthority`에 `:14` cookie 있음 (`xauth list`로 확인).
- 그런데 컨테이너 안 `/tmp/.Xauthority`에는 `:14` cookie 없음.
- `stat`으로 inode 비교하면 호스트와 컨테이너의 `~/.Xauthority`가 서로 **다른 inode**.

### 원인 — rename 기반 파일 갱신

`~/.Xauthority`는 `xauth add`/`ssh -X` 등이 쓸 때 **rename으로 원자적 교체**를 한다. 이게 bind mount와 치명적 상성 문제:

```
시간 T0:
  호스트 ~/.Xauthority  (inode 100)
  컨테이너 /tmp/.Xauthority  (inode 100)   ← bind mount로 공유

시간 T1 (새 SSH 세션 접속, xauth가 rename):
  호스트 ~/.Xauthority  (inode 200) ← 새 파일로 교체
  컨테이너 /tmp/.Xauthority  (inode 100) ← orphan inode, 이름 없이 mount만 참조
```

- Bind mount는 **경로가 아니라 inode를 붙잡는다.**
- rename은 파일 이름만 옮기는 게 아니라 inode 교체에 가까운 동작.
- 그래서 컨테이너는 과거 snapshot을 영구적으로 바라보게 됨.

### 결과

- 컨테이너 시작 시점에 이미 있던 cookie(`:10`~`:15` 등)의 터미널은 GUI 뜸.
- 시작 이후 새로 생긴 SSH 세션(`:32` 등)은 cookie가 컨테이너에 없어서 "wrong authentication" 실패.

### 왜 단순 "다시 동기화"가 어려운가

| 접근 | 결과 |
|---|---|
| `docker cp ~/.Xauthority icra2026:/tmp/.Xauthority` | `unlinkat ... device or resource busy` (bind mount 파일은 unlink 못 함) |
| `docker exec -i icra2026 xauth merge - < ~/.Xauthority` | `unable to rename authority file /tmp/.Xauthority` (xauth merge는 rename 사용) |
| `docker exec -i icra2026 bash -c 'cat > /tmp/.Xauthority' < ~/.Xauthority` | 동작은 하지만 bind mount 원본 파일을 쓰는 거라 **호스트 다른 세션에 영향 가능** (위험) |

---

## 5. 해결 전략 — Bind Mount 의존 제거

`XAUTHORITY`가 참조하는 cookie DB 위치를 **bind mount 파일에서 컨테이너 내부 전용 파일로 이전**한다.

### 구조 변화

```
[Before]
호스트 ~/.Xauthority  ←bind mount→  컨테이너 /tmp/.Xauthority
                                    ↑
                                    XAUTHORITY 가 가리킴
                                    (stale 문제)

[After]
호스트 ~/.Xauthority                 컨테이너 /home/unicorn/.Xauthority_local
        │                            ↑
        └────cat >──────────────────>│  (매번 한 방향 복사)
                                     │
                                     XAUTHORITY 가 가리킴
```

### 왜 복사가 필요한가

- `XAUTHORITY` 환경변수는 **컨테이너 안의 파일 경로**를 요구. stdin 파이프로 넘길 수 없음.
- 호스트 파일을 컨테이너 안에서 보려면 bind mount 또는 복사 둘 중 하나 필요.
- Bind mount는 위의 stale 문제로 사용 불가 → **복사 방식**으로 가야 함.
- 새 SSH 세션이 추가될 때마다 cookie가 늘어나므로 **매번 호출 시점에 전체 복사**.

### 왜 이 방식이 안전한가

- 복사는 호스트 파일을 **읽기만** 함 → 호스트 `~/.Xauthority`에 절대 영향 없음.
- 컨테이너 전용 파일은 bind mount가 아니므로 unlink/rename 자유로움.
- `cat >`는 파일 inode를 유지한 채 내용만 덮어쓰는 단순 write라 rename 문제도 없음.
- idempotent(반복 호출해도 결과 동일) — 매번 함수 진입 시 재실행해도 OK.

---

## 6. 함수화

### 원타임 세팅 (컨테이너 생성 시 한 번)

함수 자체가 자동으로 세팅하도록 설계했으므로 별도 사전 작업 불필요.
(수동으로 하려면: `docker exec icra2026 bash -c 'touch /home/unicorn/.Xauthority_local && chown unicorn:unicorn /home/unicorn/.Xauthority_local'`)

### `~/.bashrc` 최종 함수 정의

```bash
# 기존 alias는 제거 또는 주석 처리
# alias icra_unicorn='docker exec -it -e DISPLAY="$DISPLAY" icra2026 bash'

# alias와 이름 충돌 방지 (이전 세션 alias 잔재 제거)
unalias icra_unicorn 2>/dev/null

icra_unicorn() {
  # (1) 컨테이너 전용 cookie 파일이 없으면 생성 (재생성 대비, idempotent)
  docker exec icra2026 bash -c '[ -f /home/unicorn/.Xauthority_local ] || \
    (touch /home/unicorn/.Xauthority_local && chown unicorn:unicorn /home/unicorn/.Xauthority_local)'

  # (2) 호스트 ~/.Xauthority 전체를 컨테이너 전용 파일로 복사
  #     rename 없이 파일 내용만 덮어씀 → bind mount 영향 없음
  docker exec -i icra2026 sh -c 'cat > /home/unicorn/.Xauthority_local' < ~/.Xauthority

  # (3) 현재 세션 DISPLAY + 컨테이너 전용 XAUTHORITY 로 컨테이너 진입
  docker exec -it -e DISPLAY="$DISPLAY" -e XAUTHORITY=/home/unicorn/.Xauthority_local icra2026 bash
}
```

### 사용법

```bash
# 1. SSH 접속 (X forwarding 필수)
ssh -X unicorn@192.168.70.3

# 2. 컨테이너 진입 (sync + exec 자동)
icra_unicorn

# 3. GUI 앱 실행
xeyes    # 테스트
rviz     # 실제 사용
```

---

## 7. 동작 보장 범위

함수가 매 호출 시 내부 파일을 최신 `~/.Xauthority`로 갱신하므로:

| 시나리오 | 대응 |
|---|---|
| 새 SSH 세션 접속 | `~/.Xauthority`에 새 cookie 자동 추가됨 → 다음 `icra_unicorn` 호출 시 자동 반영 |
| 여러 SSH 터미널 동시 사용 | 각자 `icra_unicorn` → 각 세션 DISPLAY가 독립 주입, cookie는 공통 |
| 컨테이너 `stop/start`, `restart` | 내부 파일 유지 → 그대로 작동 |
| 컨테이너 `rm` 후 재생성 | (1)의 존재 확인 로직이 파일 자동 생성 → 복구됨 |
| 호스트 X 서버 재시작 | `~/.Xauthority` 비워질 수 있음 → 새 `ssh -X` 접속으로 다시 채워짐 |
| 다른 클라이언트 PC에서 접속 | 서버 설정 그대로 유지, 클라이언트는 `ssh -X`만 있으면 됨 |

전제:
- `ssh -X`로 접속할 것 (그래야 호스트 DISPLAY/cookie가 세팅됨).
- 컨테이너 `icra2026`이 실행 중일 것.

---

## 8. 트러블슈팅

### `sb` (source ~/.bashrc) 했더니 `syntax error near unexpected token '('`

현재 shell 세션에 이전 `icra_unicorn` alias가 남아있어서 함수 이름이 alias로 expand된 것. 해결:

```bash
unalias icra_unicorn 2>/dev/null
source ~/.bashrc
```

또는 새 터미널을 열면 초기화됨. `~/.bashrc`에 함수 정의 앞에 `unalias icra_unicorn 2>/dev/null`을 두면 영구 방어.

### 특정 SSH 터미널만 GUI 안 뜸

1. **그 터미널의 호스트 `xclock` 먼저 확인**
   - 호스트에서도 안 뜨면 SSH 터널이 죽은 세션 → `exit` 후 `ssh -X`로 재접속
   - 호스트에서 뜨면 컨테이너 쪽 문제 → `icra_unicorn` 재실행

2. **DISPLAY env 확인**
   ```bash
   echo $DISPLAY    # 비어있으면 ssh -X 없이 접속한 것
   ```

3. **cookie 존재 확인**
   ```bash
   xauth list | grep "$(echo $DISPLAY | sed 's/.*://;s/\..*//')"
   ```

### 호스트 `~/.Xauthority` cookie가 너무 많이 누적됨

SSH 세션 종료 후에도 cookie는 남아서 수십~수백 개 쌓임. 실질적 문제는 없지만 정리하려면:

```bash
rm ~/.Xauthority && touch ~/.Xauthority
# 모든 SSH 세션 종료 후 ssh -X로 재접속
```

---

## 9. 핵심 요약

- 문제 본질: bind mount가 rename-교체되는 `~/.Xauthority`와 궁합이 안 맞아 새 cookie가 컨테이너에서 stale됨.
- 해결 본질: bind mount 의존 버리고, 컨테이너 내부 전용 cookie DB 파일을 매 진입 시 호스트 내용으로 덮어씌우기.
- 실행 단순화: `icra_unicorn` 함수 하나로 sync + 진입 자동화. 여러 SSH 세션에서 반복 호출해도 안전.
