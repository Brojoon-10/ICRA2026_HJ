# 디버그 노트: `glim`이 `devel/_setup_util.py`를 망가뜨려 catkin 워크스페이스 전체 빌드 실패

## 증상

새 셸에서 `source devel/setup.bash` 후 `catkin build`를 돌리면 다음과 같은 에러가 다수 패키지에서 발생:

```
ModuleNotFoundError: No module named 'genmsg'
ModuleNotFoundError: No module named 'catkin'
```

영향 받은 패키지: `blink1`, `f110_msgs`, `cartographer_ros_msgs`, `global_line_3d` 등. 사실 이 패키지들 자체에는 문제가 없으며, 의존 패키지들이 줄줄이 abandoned 처리되며 연쇄 실패가 발생함.

## 빠른 재현 확인

```bash
bash -c 'source /opt/ros/noetic/setup.bash && echo BEFORE=$PYTHONPATH \
  && source ~/catkin_ws/devel/setup.bash && echo AFTER=$PYTHONPATH'
```

정상 상태:
```
BEFORE=/opt/ros/noetic/lib/python3/dist-packages
AFTER=/home/unicorn/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
```

깨진 상태 (이 버그):
```
BEFORE=/opt/ros/noetic/lib/python3/dist-packages
AFTER=/home/unicorn/catkin_ws/devel/lib/python3/dist-packages
```
ROS 경로가 **체이닝되지 않고 통째로 사라짐**.

## 근본 원인

`devel/_setup_util.py`의 271번 줄이 ROS 경로 없이 devel만 포함한 채로 박제되어 있음:

```python
CMAKE_PREFIX_PATH = r'/home/unicorn/catkin_ws/devel'.split(';')
```

정상이라면:

```python
CMAKE_PREFIX_PATH = r'/home/unicorn/catkin_ws/devel;/opt/ros/noetic'.split(';')
```

`_setup_util.py`가 source될 때, 이 스크립트는 이전 워크스페이스(=ROS)의 PYTHONPATH 항목을 **롤백(제거)**하고, 위 리스트에 있는 경로만 다시 **prepend**한다. 그런데 ROS 경로가 리스트에 없으므로 롤백만 일어나고 재추가는 없음 → `genmsg`, `catkin` 등 ROS Python 모듈이 `sys.path`에서 사라지게 됨.

## `_setup_util.py`가 박제되는 메커니즘

`devel/_setup_util.py`는 catkin 패키지의 cmake configure가 실행될 때마다 `/opt/ros/noetic/share/catkin/cmake/catkin_generate_environment.cmake:40`의 `configure_file()`로 재생성됨. 치환되는 값 `@CMAKE_PREFIX_PATH_AS_IS@`는 `all.cmake:55`에서 캡처됨:

```cmake
set(CMAKE_PREFIX_PATH_AS_IS ${CMAKE_PREFIX_PATH})
```

즉, **마지막으로 cmake configure가 성공한 패키지의 `CMAKE_PREFIX_PATH` 값**이 `_setup_util.py`에 박제되는 구조.

## 범인 식별 (mtime 포렌식)

```
devel/_setup_util.py                          → 2026-04-07 10:45:02   ← 망가진 파일
logs/global_line_3d/build.cmake.002.log       → 2026-04-07 10:45:02   ← 같은 초!
```

각 패키지의 cmake 로그를 교차 확인:

```
grep -r "Using CMAKE_PREFIX_PATH" ~/catkin_ws/logs/
```

거의 모든 패키지가 `/opt/ros/noetic`을 보여줌 — 단 두 패키지만 예외:

```
logs/glim/build.cmake.log
  -- Using CMAKE_PREFIX_PATH: .../GLIL_unicorn_racing/glim/../../kiss_matcher_install
logs/global_line_3d/build.cmake.001.log
  -- Using CMAKE_PREFIX_PATH: /home/unicorn/catkin_ws/devel
logs/global_line_3d/build.cmake.002.log
  -- Using CMAKE_PREFIX_PATH: /home/unicorn/catkin_ws/devel
```

### 부패가 일어난 시간 순서

```
10:41:58  global_line_3d build.cmake.000  → /opt/ros/noetic        (정상)
10:42:03  glim          build.cmake.000   → kiss_matcher_install   (이상 발생)
10:44:26  global_line_3d build.cmake.001  → /home/.../devel        (이미 깨짐)
10:45:02  global_line_3d build.cmake.002  → /home/.../devel        ← _setup_util.py 덮어씀
```

## `glim`이 근본 원인인 이유

`glim`의 [package.xml](../../state_estimation/GLIL_unicorn_racing/glim/package.xml)은 다음과 같이 선언함:

```xml
<buildtool_depend condition="$ROS_VERSION == 1">catkin</buildtool_depend>
<export>
  <build_type condition="$ROS_VERSION == 1">catkin</build_type>
</export>
```

따라서 catkin tools는 glim을 **catkin 패키지**로 인식한다. 그런데 정작 [CMakeLists.txt](../../state_estimation/GLIL_unicorn_racing/glim/CMakeLists.txt)에서는 **`find_package(catkin)`도 `catkin_package()`도 호출하지 않음**. 대신 다음을 수행:

```cmake
list(PREPEND CMAKE_PREFIX_PATH "${KISS_MATCHER_PREFIX}")
```

`package.xml`(catkin 선언)과 `CMakeLists.txt`(순수 cmake 동작)의 **불일치**가 catkin tools의 환경 캐시를 흐트러뜨림. glim 빌드 후, catkin tools가 다음 패키지(`global_line_3d`)에 전달하는 per-package env 캐시에서 `/opt/ros/noetic`이 빠지게 됨. 그 결과 `global_line_3d`가 `CMAKE_PREFIX_PATH`가 깨진 상태로 cmake configure를 실행하고, 성공적으로 끝나면서 `devel/_setup_util.py`를 잘못된 값으로 덮어씀.

이 시점부터 매번 `source devel/setup.bash`가 ROS의 `PYTHONPATH`를 wipe하고, 다음 `catkin build`는 `genmsg`/`catkin` Python 모듈이 필요한 모든 패키지에서 실패한다.

## 해결 방법 (권장 — glim 최소 패치)

[glim/CMakeLists.txt](../../state_estimation/GLIL_unicorn_racing/glim/CMakeLists.txt)를 열고 `project(...)` 바로 뒤에 **두 줄만** 추가:

```cmake
project(glim VERSION 1.1.0 LANGUAGES C CXX)

find_package(catkin REQUIRED)
catkin_package()

add_compile_options(-std=c++17)
...
```

이렇게 하면:
- `find_package(catkin)`이 `all.cmake`를 include하면서 `CMAKE_PREFIX_PATH_AS_IS`에 ROS 경로가 정상 캡처됨
- `catkin_package()`가 `devel/_setup_util.py`를 올바른 체인 값으로 재생성함
- `package.xml`의 `build_type=catkin` 선언과 `CMakeLists.txt`가 일치하게 됨
- glim 자체의 빌드 로직(GTSAM, kiss_matcher, TBB 등)은 **전혀 건드리지 않음**

## 대안 — glim을 plain cmake 패키지로 선언

[glim/package.xml](../../state_estimation/GLIL_unicorn_racing/glim/package.xml)을 다음과 같이 수정:

```xml
<buildtool_depend>cmake</buildtool_depend>
<export>
  <build_type>cmake</build_type>
</export>
```

이렇게 하면 catkin tools가 glim을 "plain cmake package"로 다루고, 빌드 도중 `devel/_setup_util.py`를 일절 건드리지 않음.

## 패치 적용 후 복구 절차

```bash
cd ~/catkin_ws
catkin clean -y
unset PYTHONPATH
source /opt/ros/noetic/setup.bash
echo $CMAKE_PREFIX_PATH      # /opt/ros/noetic 포함 확인
catkin build
```

빌드 후 검증:

```bash
grep "^            CMAKE_PREFIX_PATH = r" devel/_setup_util.py
# 기대 결과:
#     CMAKE_PREFIX_PATH = r'/home/unicorn/catkin_ws/devel;/opt/ros/noetic'.split(';')
```

그리고:

```bash
bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 -c "import genmsg, catkin; print(\"OK\")"'
```

## 핵심 파일

- [devel/_setup_util.py:271](../../../devel/_setup_util.py#L271) — 박제되는 줄
- `/opt/ros/noetic/share/catkin/cmake/all.cmake:55` — `CMAKE_PREFIX_PATH_AS_IS`가 캡처되는 위치
- `/opt/ros/noetic/share/catkin/cmake/catkin_generate_environment.cmake:40` — 템플릿이 configure되는 위치
- [glim/CMakeLists.txt](../../state_estimation/GLIL_unicorn_racing/glim/CMakeLists.txt) — 패치 대상 파일
- [glim/package.xml](../../state_estimation/GLIL_unicorn_racing/glim/package.xml) — 불일치가 있는 선언 파일

---

## 2026-05-03 재발 사례 + 적용 결과

### 재발 증상
컨테이너 빌드 시 `livox_ros_driver2`, `f110_msgs` 등 메시지 패키지에서 동일한 `ModuleNotFoundError: No module named 'genmsg'` 발생.

### 검증 (가설 = 결과 일치)
- `devel/_setup_util.py:271`이 `'.../kiss_matcher_install'`로만 박제 → `/opt/ros/noetic` 누락
- `logs/glim/build.cmake.000.log:177`: `Using CMAKE_PREFIX_PATH: /home/unicorn3/.../kiss_matcher_install` (정상 패키지는 `/opt/ros/noetic`)
- `_setup_util.py` mtime ≡ glim cmake 로그 mtime → glim이 마지막으로 박제했다는 직접 증거
- `unset PYTHONPATH && source devel/setup.bash` 후 `import genmsg` 실패 재현됨

### 적용한 패치 (사용자 직접 적용 형태 — 채택안)

[glim/CMakeLists.txt](../../state_estimation/GLIL_unicorn_racing/glim/CMakeLists.txt) 라인 1~5에 **두 줄만** 삽입. `project()` 바로 뒤, KISS-Matcher PREPEND보다 **앞** 위치가 핵심.

```cmake
cmake_minimum_required(VERSION 3.16)
project(glim VERSION 1.1.0 LANGUAGES C CXX)

find_package(catkin REQUIRED)        # ← 추가 (인자 없음)
catkin_package()                     # ← 추가 (인자 없음)

add_compile_options(-std=c++17)
```

#### 채택 이유 — 의도적으로 "인자 없는" 호출을 위에 추가하는 패턴
- 위쪽의 `catkin_package()`는 **`_setup_util.py` 박제값을 깨끗하게 만들기 위한 호출**. catkin이 `all.cmake:55`에서 `CMAKE_PREFIX_PATH_AS_IS`를 캡처하는 시점이 이 시점이므로, **이 시점의 `CMAKE_PREFIX_PATH`가 KISS-Matcher로 오염되기 전 상태**여야 한다는 게 본 패치의 전부.
- 라인 64~72의 **기존 ROS 블록은 손대지 않음** — 거기서는 `catkin_package(INCLUDE_DIRS include LIBRARIES glim)`로 다운스트림(`glim_ros1`, `glim_ext`)을 위한 export를 정상 수행. 이걸 위로 옮기면 위쪽 시점에 `add_library(glim ...)`가 아직 선언 안 됐기 때문에 export 의도가 흐려질 수 있음.
- 이중 `catkin_package()` 호출은 cmake 동작상 **두 번째 호출이 export를 덮어씀** → 다운스트림은 정상 작동, 박제는 첫 번째 호출이 결정. **빌드/링크/다운스트림 모두 회귀 없음** 확인.
- 사용자는 이 두 줄 추가 패턴을 과거에도 동일 환경에서 검증 → 같은 형태 그대로 채택.

#### 비교: 다른 옵션들
| 안 | 형태 | 채택 여부 | 이유 |
|---|---|---|---|
| **A (채택)** | 위쪽에 인자 없는 `find_package(catkin)` + `catkin_package()` 추가, 아래쪽 ROS 블록 유지 | ✅ | 최소 diff, 사용자 검증 이력, 다운스트림 export 보존 |
| B | 아래쪽 ROS 블록을 위로 통째로 이동 | ❌ | upstream merge 충돌 위험, 글림 원본 구조 변경 |
| C | KISS-Matcher 블록 끝에 `list(REMOVE_ITEM CMAKE_PREFIX_PATH ...)` | ❌ | `find_package(kiss_matcher)`가 `CMAKE_PREFIX_PATH`를 통째로 덮어쓰는 것으로 관측됨 → REMOVE_ITEM이 효과 없을 수 있음 |
| D | `glim/package.xml`을 plain cmake로 바꾸기 | ❌ | catkin tools에서 패키지 분류 자체가 바뀌어 글림 빌드/설치 흐름에 광범위 영향 |

### 복구 절차 (실제 실행한 형태)
컨테이너 내부에서:
```bash
cd $HOME/catkin_ws
# build/devel/logs는 mount라 디렉터리 자체 삭제 불가 → 내용물만 비움 (catkin clean -y 등가)
find build -mindepth 1 -delete
find devel -mindepth 1 -delete
find logs  -mindepth 1 -delete

unset PYTHONPATH
source /opt/ros/noetic/setup.bash
echo $CMAKE_PREFIX_PATH        # /opt/ros/noetic 만 보여야 정상

catkin build
```

> 주의: `catkin clean -y`는 mount된 디렉터리에서 `OSError: Device or resource busy`로 실패함. 위처럼 `find -delete`로 우회.

### 검증 결과 (2026-05-03)
```
PYTHONPATH=/home/unicorn3/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
_setup_util.py:271 = r'/home/unicorn3/catkin_ws/devel;/opt/ros/noetic'   ✅
import genmsg, catkin → OK                                                ✅
glim cmake log: Using CMAKE_PREFIX_PATH: /home/unicorn3/catkin_ws/devel;/opt/ros/noetic   ✅
```

### 한 번에 검증 가능한 명령
```bash
unset PYTHONPATH && source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && \
  echo "PYTHONPATH=$PYTHONPATH" && \
  grep -n "CMAKE_PREFIX_PATH = r" ~/catkin_ws/devel/_setup_util.py && \
  python3 -c "import genmsg, catkin; print('genmsg/catkin OK')" && \
  grep -h "Using CMAKE_PREFIX_PATH" ~/catkin_ws/logs/glim/build.cmake.*.log | tail -3
```

---

## 후속 이슈 #1 — `2.5d_detection` (mapconversion) 빌드 실패

glim 박제 문제 해결 후 전체 재빌드 과정에서 별개의 빌드 실패 두 개가 연달아 발생. **glim 이슈와 무관**하지만 같은 빌드 사이클에서 발견됐으니 같이 정리.

### 1-A. Frenet 3D API 시그니처 미스매치

**증상**:
```
no matching function for call to 'frenet_conversion::FrenetConverter::GetFrenetPoint(double&, double&, double*, double*, int*, bool)'
candidate: void GetFrenetPoint(const double x, const double y, const double z, ...)
note: candidate expects 7 arguments, 6 provided
```

**원인**: `f110_utils/libs/frenet_conversion`이 3D로 확장되며 `GetFrenetPoint`/`GetFrenetOdometry` 시그니처에 `z` 파라미터가 추가됐는데(커밋 `30d30ec`, `454b341` 라인업), `perception/2.5d_detection` submodule은 그 시점에 같이 업데이트되지 않음. 커밋 `fb0a918`에서 `abp-detection` 쪽은 3D 대응됐으나 `2.5d_detection`은 빠져 있었음.

**적용한 패치** (모두 z=0.0 디폴트 — 다른 2D 호출부의 컨벤션과 동일, [perception/abp-detection/src/detect.cpp:635,688](../../perception/abp-detection/src/detect.cpp#L635)):
- [perception/2.5d_detection/src/tracking_node.cpp:486](../../perception/2.5d_detection/src/tracking_node.cpp#L486) — `GetFrenetPoint(px, py, 0.0, &s, &d, &idx, true)`
- [perception/2.5d_detection/src/tracking_node.cpp:490](../../perception/2.5d_detection/src/tracking_node.cpp#L490) — `GetFrenetOdometry(px, py, 0.0, yaw, v, 0.0, ...)`
- [perception/2.5d_detection/src/detection_node.cpp:338](../../perception/2.5d_detection/src/detection_node.cpp#L338) — `GetFrenetPoint(cx, cy, 0.0, &s, &d, &idx, true)`

### 1-B. `-lfrenet_conversion` 링크 실패

**증상**:
```
/usr/bin/ld: cannot find -lfrenet_conversion
```

**원인**: `mapconversion` (=2.5d_detection 패키지)의 `target_link_libraries`에서 `frenet_conversion`을 **이름(string)** 으로 추가:
```cmake
target_link_libraries(detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} frenet_conversion)
```
같은 빌드 안의 cmake 타깃이 아니라서 cmake가 `-lfrenet_conversion` 으로 링커에 그대로 전달 → 링커는 `-L` 경로에 frenet_conversion 라이브러리 디렉터리가 없어서 실패. (실제 라이브러리는 `find_package(catkin COMPONENTS ... frenet_conversion)`을 통해 `${catkin_LIBRARIES}`에 **절대 경로**로 이미 포함되어 있었음.)

**적용한 패치** ([perception/2.5d_detection/CMakeLists.txt](../../perception/2.5d_detection/CMakeLists.txt)):
- 라인 76, 93의 `target_link_libraries(...)`에서 중복된 `frenet_conversion` 인자 제거.
- 라인 28의 `find_package(catkin REQUIRED COMPONENTS ... frenet_conversion)`은 그대로 유지 → `${catkin_LIBRARIES}`에 절대경로로 포함.

### 검증
```bash
catkin build mapconversion --force-cmake
ls ~/catkin_ws/devel/lib/mapconversion/   # detection_node, tracking_node 가 보여야 정상
```

### Submodule 미커밋 변경 안내
위 1-A, 1-B 패치 + glim/CMakeLists 패치 모두 **submodule 안의 변경**:

| Submodule | Detached HEAD | 변경 파일 |
|---|---|---|
| `state_estimation/GLIL_unicorn_racing` | `de669ab` | `glim/CMakeLists.txt` |
| `perception/2.5d_detection` | `accbaba` | `CMakeLists.txt`, `src/detection_node.cpp`, `src/tracking_node.cpp` |

호스트 repo에서는 submodule 포인터 변경(`m`)으로만 보임. 영구화하려면 각 submodule 안에서 별도 브랜치 만들어 커밋/푸시한 뒤 호스트 repo에서 submodule 포인터 업데이트 커밋 필요. (CLAUDE.md의 git 규칙상 submodule 커밋은 사용자 명시 승인 후에만 진행.)
