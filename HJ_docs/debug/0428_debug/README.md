# 0428 Debug Workspace

작업: Stage 1 of `HJ_docs/overtake_pipeline_redesign_20260427.md` v3.
Plan: `../../overtake_pipeline_redesign_20260427.md`

## 디렉토리 구조

```
0428_debug/
├── README.md                    # 본 파일 (인덱스)
├── work_log/                    # 진행 기록
│   └── stage1_work_log_20260428.md   # 매 S1-* 작업의 변경/빌드/검증/결과
├── scripts/                     # 본 세션에서 만든 분석/유틸 스크립트
│   ├── _tmp_pose_teleport.py        # ego /initialpose → set_model_state (model: unicorn)
│   ├── _tmp_analyze_bags.py         # bag1/bag2 tick_json 통계 (1차 분석)
│   ├── _tmp_deep_dive.py            # 심화: kappa spike, multi-obs runs, hold_last chains
│   ├── _tmp_extra_topics.py         # published wpnts 형태, predictor, tracking truth, SM
│   ├── _tmp_fail_chain_dive.py      # bag1 51-tick fail chain timeline
│   └── _tmp_pre_collision.py        # bag1 충돌 (t=519.83) 직전 분석
├── analysis_output/             # 분석 스크립트 산출 텍스트
│   ├── bag_analysis.txt
│   ├── bag_deep.txt
│   ├── bag_extra.txt
│   ├── fail_chain.txt
│   └── pre_collision.txt
├── bags/                        # 본 세션 라이브 sim 녹화 bag
│   └── (S1-*_<timestamp>.bag)
└── prev_session_scripts/        # 이전 세션 흔적 (참고용, 본 세션 작업과 분리)
    └── _tmp_*.py (다수)
```

## 진행 추적
- **plan**: `../../overtake_pipeline_redesign_20260427.md`
- **work log**: `work_log/stage1_work_log_20260428.md`
- **backup**: `planner/mpc_planner/{node,src,config}/*_backup_20260428.{py,yaml}` (워크스페이스)
- **commit 금지** (사용자 명시).

## Bag 녹화 (이 폴더 안 저장)
```bash
docker exec -e ROS_MASTER_URI=http://192.168.70.2:11311 -e ROS_HOSTNAME=nuc3 -e ROS_IP=192.168.70.2 icra2026 \
  bash -c 'cd /home/nuc3/catkin_ws/src/race_stack/HJ_docs/debug/0428_debug/bags && \
           rosbag record -O <S1-name>_$(date +%H%M%S).bag \
             /mpc_auto/debug/tick_json /planner/mpc/wpnts /car_state/odom_frenet \
             /tracking/obstacles /tracking/obstacles_truth /opponent_prediction/obstacles \
             /collision_marker /behavior_strategy &'
```
