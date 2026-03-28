# Global Racing Line - Usage Guide

## Overview

3D 트랙에서 시간 최적(time-optimal) 글로벌 레이싱 라인을 생성하고, 시각화 및 export하는 모듈.

## Prerequisites

- 사전에 `track_processing` 단계를 완료하여 `data/smoothed_track_data/`에 스무딩된 트랙 CSV가 존재해야 함
- 사전에 `gg_diagram_generation` 단계를 완료하여 `data/gg_diagrams/`에 GG 다이어그램이 존재해야 함
- 차량 파라미터 파일: `data/vehicle_params/params_{vehicle_name}.yml`

## Workflow

### 1. 레이싱 라인 생성

```bash
# 기본 시간 최적 레이싱 라인 생성
python global_racing_line/gen_global_racing_line.py
```

스크립트 상단의 `params` dict에서 설정 변경:

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `track_name` | 입력 트랙 CSV 파일명 | - |
| `raceline_name` | 출력 레이싱 라인 CSV 파일명 | - |
| `vehicle_name` | 차량 파라미터 이름 | `rc_car_10th` |
| `safety_distance` | 트랙 경계 안전 여유 [m] | `0.2` |
| `gg_mode` | GG 다이어그램 모드 (`polar` / `diamond`) | `diamond` |
| `V_guess` | 초기 속도 추정값 [m/s] | - |
| `w_T` | 시간 비용 가중치 | `1.0` |
| `w_jx`, `w_jy` | 저크 비용 가중치 (정규화) | - |
| `w_dOmega_z` | 곡률 비용 가중치 | `0.0` |

출력: `data/global_racing_lines/{raceline_name}`

### 2. (선택) 섹터별 튜닝

두 가지 방법 중 택일:

```bash
# 방법 A: NLP 재풀이 없이 속도만 스케일링 (빠름)
python global_racing_line/apply_sector_velocity.py

# 방법 B: 섹터별 마찰/속도 반영하여 NLP 재풀이 (정확함)
python global_racing_line/gen_global_racing_line_sector_tuned.py
```

섹터 설정 파일:
- `data/sector_config/sector_friction.yaml` - 섹터별 마찰 스케일링
- `data/sector_config/sector_velocity.yaml` - 섹터별 속도 스케일링

### 3. (선택) 시각화

```bash
# 레이싱 라인 상세 플롯 (속도, 가속도, 저크 등)
python global_racing_line/plot_global_racing_line.py

# 논문용 고품질 플롯 (2D/3D + 속도/가속도 프로파일)
python global_racing_line/plot_racing_line_full.py
```

`plot_racing_line_full.py`에서 `save_fig = True`로 설정하면 `data/figure/`에 PNG 저장.

### 4. Waypoint Export (UNICORN 포맷)

```bash
python global_racing_line/export_global_waypoints.py
```

레이싱 라인을 curvilinear 좌표에서 Cartesian 좌표로 변환하고, 균일 간격(`waypoint_spacing`, 기본 0.1m)으로 재보간하여 JSON으로 export.

출력: `data/global_racing_lines/{output_name}` (JSON, RViz 마커 포함)

## 파일 목록

| 파일 | 용도 |
|-----|------|
| `gen_global_racing_line.py` | 시간 최적 레이싱 라인 생성 (CasADi/IPOPT) |
| `gen_global_racing_line_iy.py` | 실험용 변형 (다른 트랙/파라미터) |
| `gen_global_racing_line_sector_tuned.py` | 섹터별 마찰/속도 튜닝 적용 NLP |
| `apply_sector_velocity.py` | 사후 속도 스케일링 (NLP 재풀이 없음) |
| `plot_global_racing_line.py` | 레이싱 라인 상세 시각화 |
| `plot_racing_line_full.py` | 논문용 고품질 시각화 |
| `export_global_waypoints.py` | UNICORN JSON 포맷 export |
