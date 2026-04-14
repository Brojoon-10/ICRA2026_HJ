# HSL (Coin-HSL) Setup for IPOPT ma27

## 왜 필요한가

`fast_gen_gg_diagrams.py`, `calc_max_slip_map.py`, `3d_optimized_vel_planner.py` 내부의 IPOPT NLP는 기본적으로 **MUMPS** 선형 solver를 쓰는데, 소규모 NLP에서는 **HSL ma27이 2~5배 빠름**. 모든 스크립트에 **자동 fallback** 로직이 들어있어서:

- HSL 설치되어 있으면 → ma27 사용 (빠름)
- 설치 안 되어 있으면 → MUMPS fallback (동작은 정상)

성능 비교 (실측):

| | MUMPS | ma27 |
|---|---|---|
| fast_ggv_gen fast (300 NLPs) | 2.8초 | **0.5초** |
| fast_ggv_gen full (16,875 NLPs) | 22초 | **8.6초** |
| 3d_optimized_vel_planner | 2.5초 | (개선 예상) |

## 라이센스

**Coin-HSL Archive**는 무료(학술/상업용 모두)지만 **라이센스 신청은 필요**하고, **재배포 금지**. 각 사용자가 직접 받아야 한다.

- 신청: https://licences.stfc.ac.uk/product/coin-hsl-archive
- 가격: Free of charge (HSL Archive Licence)
- 승인: 이메일로 신청 즉시 또는 수 시간 내

---

## 설치 방법

### 1. 소스 받기

위 링크에서 계정 생성 → 신청 → 승인되면 다운로드 섹션에서 **`coinhsl-<날짜>.tar.gz`** (source tarball) 받기.  
**mac/windows binaries는 사용 불가** (우리는 Linux Docker). **source tarball** 받아야 함.

### 2. 이 폴더에 tarball 배치

받은 파일을:
```
planner/3d_gb_optimizer/fast_ggv_gen/solver/coinhsl-2024.05.15.tar.gz
```
위치에 복사. (파일명 날짜는 달라도 OK — 스크립트가 자동 감지)

> `*.tar.gz`는 `.gitignore`로 제외되어 있어서 실수로도 git에 안 올라감.

### 3. 컨테이너 내부에서 setup 스크립트 실행

```bash
docker exec -it icra2026 bash
```

컨테이너 안에서:

```bash
cd /home/unicorn/catkin_ws/src/race_stack/planner/3d_gb_optimizer/fast_ggv_gen/solver
bash setup_hsl.sh
```

스크립트가 하는 일:
1. `gfortran`, `libblas-dev`, `liblapack-dev`, `libmetis-dev`, `ninja-build` 설치 (apt)
2. `meson` 버전 확인, 필요시 최신으로 업그레이드 (pip)
3. tarball 압축 해제 → `coinhsl-<version>/` 폴더 생성
4. Meson + Ninja로 빌드 → `builddir/libcoinhsl.so` 생성
5. `install/` 로 install
6. CasADi가 찾는 이름으로 **심볼릭 링크**:  
   `/usr/local/lib/python3.8/dist-packages/casadi/libhsl.so → .../install/lib/x86_64-linux-gnu/libcoinhsl.so`
7. 간단한 NLP로 ma27 동작 확인

성공하면 마지막에:
```
==============================================
 HSL ma27 installed and verified successfully.
==============================================
```

### 4. 실행 확인

```bash
cd /home/unicorn/catkin_ws/src/race_stack/planner/3d_gb_optimizer/fast_ggv_gen
python3 fast_gen_gg_diagrams.py --vehicle_name rc_car_10th --fast
```

로그 첫 줄에 **`[fast_gg] linear_solver: ma27 (HSL)`** 이 나오면 성공. MUMPS로 나오면 심볼릭 링크 확인 필요.

---

## 권한 문제

Docker 컨테이너 기본 사용자가 `root`가 아니고 `sudo`도 없으면:

```bash
docker exec -u 0 -it icra2026 bash
cd /home/unicorn/catkin_ws/src/race_stack/planner/3d_gb_optimizer/fast_ggv_gen/solver
bash setup_hsl.sh
```

로 root로 실행.

---

## 컨테이너 재생성 시

심볼릭 링크는 컨테이너 내부의 `/usr/local/lib/python3.8/dist-packages/casadi/`에 만들어지므로 **컨테이너를 새로 띄우면 사라짐**. 다시 돌리려면:

- 이미 빌드된 `install/libcoinhsl.so`는 호스트 볼륨이므로 그대로 있음
- 심볼릭 링크만 새로 만들면 됨:
  ```bash
  ln -sf /home/unicorn/catkin_ws/src/race_stack/planner/3d_gb_optimizer/fast_ggv_gen/solver/install/lib/x86_64-linux-gnu/libcoinhsl.so \
         /usr/local/lib/python3.8/dist-packages/casadi/libhsl.so
  ```
- 또는 `bash setup_hsl.sh`를 다시 실행 (빌드는 스킵됨, 링크만 재생성)

---

## 폴더 구조

```
solver/
├── README.md                          ← 이 문서 (git 커밋됨)
├── setup_hsl.sh                       ← 설치 스크립트 (git 커밋됨)
├── .gitignore 규칙 (fast_ggv_gen/.gitignore)
│   └── solver/coinhsl-*.tar.gz, coinhsl-*/, install/ 는 무시
│
├── coinhsl-2024.05.15.tar.gz          ← 라이센스 소스 (.gitignore)
├── coinhsl-2024.05.15/                ← 압축 해제 (.gitignore)
│   └── builddir/libcoinhsl.so         ← 빌드 결과물
└── install/                           ← meson install prefix (.gitignore)
    └── lib/x86_64-linux-gnu/
        └── libcoinhsl.so              ← 최종 배포본 → CasADi libhsl.so 로 링크
```

---

## Fallback 동작 확인

세 파일 모두 **import 시점에 probe**로 ma27 가능 여부를 확인하고 자동으로 선택:

```python
def _select_linear_solver():
    try:
        x = MX.sym('x')
        probe = nlpsol('hsl_probe', 'ipopt',
                       {'x': x, 'f': (x - 1.0)**2},
                       {'ipopt.linear_solver': 'ma27',
                        'ipopt.print_level': 0, 'print_time': 0})
        probe(x0=0.0)
        if probe.stats().get('success', False):
            return 'ma27'
    except Exception:
        pass
    return 'mumps'
```

HSL 없는 환경(다른 개발자의 컨테이너, CI 등)에서도 코드 수정 없이 돌아간다. 단지 느릴 뿐.

---

## 문제해결

### `meson setup`에서 `add_install_script args must be strings` 에러
→ Meson 0.53 이하. 스크립트가 자동으로 pip으로 업그레이드하지만, 안 되면 수동으로:
```bash
pip3 install --upgrade meson
```

### `metis.h: No such file or directory`
→ `libmetis-dev` 미설치. `apt-get install libmetis-dev`.

### `probe success=False` / `DYNAMIC_LIBRARY_FAILURE`
→ 심볼릭 링크가 없거나 대상이 잘못됨. 6단계를 수동으로:
```bash
ls -la /usr/local/lib/python3.8/dist-packages/casadi/libhsl.so
# → 반드시 .../install/lib/x86_64-linux-gnu/libcoinhsl.so 로 링크되어 있어야 함
```
