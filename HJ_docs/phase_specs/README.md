---
name: Phase Specs — folder convention
description: Phase 진입 직전 spec sketch 모음 + 사용 규약
created: 2026-05-01
---

# `phase_specs/` 폴더 사용 규약

각 Phase 진입 직전 1~2시간 작성하는 **단기 작업 spec sketch** 보관.

## 왜 별도 폴더?

- vision / master plan = 장기 reference (수개월).
- phase spec = 단기 작업 문서 (해당 phase 끝나면 implemented 상태로 reference 만).
- vision 문서가 spec 디테일로 비대해지지 않게.

## 명명 규칙

```
phase{N}_{snake_case_phase_name}_spec_{YYYYMMDD}.md
```

예시:
- `phase1_obstacle_horizon_spec_20260502.md`
- `phase4_plan_aware_mpc_spec_20260510.md`
- `phase7_strategy_planner_module_spec_20260520.md`

## 각 spec 문서 템플릿

```markdown
---
name: Phase N — <name> spec
description: 한 줄 요약
phase: N
created: YYYY-MM-DD
status: draft | in-progress | implemented | superseded
related_phases: [N-1, N+1]   # 선행 / 후행
---

# Phase N — <name> spec sketch

## 목표 (한 줄)

## 의존성
- 선행 phase: ...
- 외부 모듈: ...
- 토픽 / 메시지: ...

## 수정 파일 / 함수 (path:line)
| 파일 | 함수 / 위치 | 변경 내용 |
|------|-----------|----------|

## 데이터 구조
이전:
```python
...
```
신규:
```python
...
```

## 알고리즘 의사코드 (10~30줄)

```python
def ...
    ...
```

## 실패 케이스 + 처리

| 케이스 | 트리거 | 처리 |
|-------|-------|------|

## 튜닝 파라미터 + 초기값

| 파라미터 | 단위 | 초기값 | 의미 |
|---------|------|-------|------|

## 검증

### 시나리오
- bag 1: ...
- bag 2: ...

### 분석 스크립트
- `HJ_docs/debug/.../analyze_phaseN.py`

### 통과 기준 (정량)
- 수치 1: ...
- 수치 2: ...

## 변경 history (구현 중 발견된 결정)

| 날짜 | 결정 | 사유 |
|------|------|------|
```

## 라이프사이클

| 상태 | 의미 | 전환 조건 |
|------|------|----------|
| draft | spec 작성 중 | 신규 |
| in-progress | spec 따라 구현 진행 | spec 작성 완료 후 |
| implemented | 구현 + 검증 끝, commit 됨 | 통과 기준 충족 시 |
| superseded | 다음 phase 가 무효화 | 후속 phase 진입 시 |

상태는 frontmatter 의 `status` 필드.

## vision / TODO 와의 cross-link

각 spec 작성 후 두 군데 cross-reference 추가:

1. **`HJ_docs/strategic_overtake_vision_20260501.md` §10 Phase 로드맵 표** 에 link 칸:
   `[spec](phase_specs/phaseN_..._spec_<date>.md)`
2. **`TODO_HJ.md` §1.5 Phase N milestone 박스 위** 한 줄:
   `> spec: [phase_specs/phaseN_..._spec_<date>.md](HJ_docs/phase_specs/...)`

## 작업 흐름

phase 진입 직전:

1. `phase_specs/` 안에 신규 spec sketch 작성 (status=draft).
2. vision / TODO 에 cross-link 추가.
3. spec 검토 후 status=in-progress.
4. spec 따라 코드 짜기. 결정 변경 시 spec 의 "변경 history" 에 기록 (덮어쓰지 않고 append).
5. 구현 + 빌드 + 검증 통과 → status=implemented + commit.
6. 다음 phase 진입 시 이 spec 은 reference 로만. 덮어쓰지 않음.

## 현재 spec 목록

| Phase | 이름 | 상태 | 파일 |
|-------|------|------|------|
| 1 | ObstacleHorizon | in-progress (사용자 검증 대기) | [phase1_obstacle_horizon_spec_20260502.md](phase1_obstacle_horizon_spec_20260502.md) |
