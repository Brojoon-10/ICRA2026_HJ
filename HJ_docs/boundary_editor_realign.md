# Boundary Editor — Full Realign 알고리즘 변경 (2026-05-19)

## 최종 알고리즘 (v9: Newton-direct + width shrink)

[stack_master/scripts/boundary_editor_qt.py](../stack_master/scripts/boundary_editor_qt.py) `_full_realign`.

### 사용자 의도

> "Newton-direct로 max=0° 먼저 만들고, boundary poly만 cross 안 생기게 센터까지 거리를 조절해서"

### Phase 1 — Newton-direct (max → 0°)
각 인덱스 i에서 rib를 midpoint 기준으로 회전해 centerline tangent에 수직으로 정렬. step=0.5, max 30 iter, max<0.001° 시 조기 종료. Cross는 hairpin에서 발생 가능.

### Phase 2 — Width shrink (cross → 0)
중심선 C, normal n 고정. rib을 (반-width)로 분해:
```
hL[i] = (L[i] - C[i]) · n[i]
hR[i] = (R[i] - C[i]) · n[i]
L[i]  = C[i] + hL[i] * n[i]
R[i]  = C[i] + hR[i] * n[i]
```
Rib 방향이 항상 ±n이므로 **width 조절은 perpendicular 깨지 않음**.

Cross 발생 인접 인덱스 (i, i+1)의 `hL`, `hR`을 0.7배로 곱함. Cross 없을 때까지 반복 (max 50 pass).

## 결과 (3개 트랙)

| 트랙 | N | Before max / mean / cross | After max / mean / cross | 시간 |
|---|---|---|---|---|
| test_kd_0518_v1 | 284 | 28.26° / 4.41° / 1 | **0.000° / 0.000° / 0** | 4.4ms |
| gazebo_wall_2 | 353 | 40.56° / 7.93° / 0 | **0.000° / 0.000° / 0** | 6.4ms |
| kd_0518_v1 | 284 | 3.18° / 0.78° / 0 | **0.000° / 0.000° / 0** | 3.9ms |

모든 트랙에서 **cross=0 + max=0°** 동시 달성. baseline에 cross 있던 트랙도 해소.

### Side effect

cross 발생 인덱스 부근의 boundary가 centerline 쪽으로 좁아짐. test_kd_0518_v1의 경우 idx 115-116 부근 13개 인덱스의 half-width가 줄어듦 (cross 영역의 boundary 모양 살짝 좁아짐). 다른 인덱스의 boundary는 그대로.

## Trade-off

Newton-direct는 boundary polyline 형상을 변경 (L, R 점 위치 이동). 단 **cross-aware iteration 추가로 cross 절대 안 늘림**:
- 베이스라인 cross 0 → 결과 cross 0 (gazebo_wall_2, kd_0518_v1)
- 베이스라인 cross > 0 → 동등 cross 보존 또는 reject

사용자 의도("바운더리는 좀 최대한 꼬이지 않게 유지하면서 지금 방식") 정확히 충족.

## 원리

```python
# 각 인덱스 i에 대해
tang_i = (C[i+1] - C[i-1]) / norm   # centerline tangent (wrap-aware central diff)
normal_i = perp(tang_i)              # rib가 향해야 할 방향
rib_i = R[i] - L[i]
width_i = |rib_i|
sign_i = sign(rib_i · normal_i)      # rib가 normal 양/음 방향?
rib_target = normal_i * sign_i * width_i  # 폭 유지하며 수직 방향으로

mid_i = (L[i] + R[i]) / 2
L_target = mid_i - 0.5 * rib_target
R_target = mid_i + 0.5 * rib_target

# Damped update (step=0.5)
L_new[i] = (1-step) * L[i] + step * L_target
R_new[i] = (1-step) * R[i] + step * R_target
```

## 시도 + 폐기된 접근들

세션 중 시도하고 폐기된 방향 (HJ_docs/debug/ 참고):
- **v1 (basic 3-stage)**: center source 추가만 — max 12.55°까지
- **v2 (dense linear interp)**: max 15.5° 한계 (boundary tangent != center tangent)
- **v3 (brute-force coord descent)**: 너무 느림 (수십 분)
- **v4 (Newton + snap-to-polyline)**: max 7.78° (oscillation)
- **v5 (composite multi-stage)**: max 25.94°, polyline 거의 보존 (deviation 9-33cm)
- **v6 (cubic spline + redistribution)**: standalone max 7.9° (cross 2), editor에 옮길 때 결과 변형 — 정밀 디버그 필요했으나 사용자가 Newton-direct로 갈아탐
- **v7 (sequential monotonic walk)**: monotonic만으론 cross 방지 불충분 (ribbon self-intersection 가능). max 88°로 망함

## 다음 세션 복귀 가이드

1. 현재 patch가 Newton-direct로 동작. max < 0.01° 보장
2. cross 발생 우려가 다시 생기면 v5 backup ([boundary_editor_qt_backup_20260519_v5.py](../stack_master/scripts/boundary_editor_qt_backup_20260519_v5.py)) 또는 v6 standalone ([HJ_docs/debug/strong_realign_v6_spline.py](debug/strong_realign_v6_spline.py)) 복귀 가능
3. v6의 editor 통합 디버그 (cross=2 vs cross=10 차이 원인) 미완 — 필요 시 다음 세션에서 진행
4. Deviation을 줄이려면 Newton 결과를 polyline에 snap-back (v4 방식). 단 snap이 cross를 키울 수 있음
