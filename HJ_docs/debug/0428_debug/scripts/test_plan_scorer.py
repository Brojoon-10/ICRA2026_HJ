import sys, os
sys.path.insert(0, os.path.expanduser('~/catkin_ws/src/race_stack/planner/mpc_planner/src'))
from plan_scorer import filter_feasible_plans, score_plan
from plan_library import PLAN_LEFT_PASS, PLAN_RIGHT_PASS, PLAN_TRAIL, PLAN_RACELINE

# Feasibility filter
all_plans = [PLAN_LEFT_PASS, PLAN_RIGHT_PASS, PLAN_TRAIL]
print('ego_n=+0.30 (LEFT committed):', [p['name'] for p in filter_feasible_plans(all_plans, 0.30)])
print('ego_n=-0.30 (RIGHT committed):', [p['name'] for p in filter_feasible_plans(all_plans, -0.30)])
print('ego_n=+0.05 (centered):', [p['name'] for p in filter_feasible_plans(all_plans, 0.05)])
print()

# Smooth risk shape
side_scores = {'d_free_L': 0.20, 'd_free_R': 0.20, 'dv': 0.5, 'v_obs': 1.5}
print('d_free_L  risk  total  (LEFT_PASS, ego_n=0)')
for d in [0.40, 0.30, 0.25, 0.20, 0.15, 0.10, 0.05, 0.00, -0.05]:
    side_scores['d_free_L'] = d
    s, bd = score_plan(PLAN_LEFT_PASS, ego_n=0.0, side_scores=side_scores, obs_in_horizon=True)
    print('  %5.2f  %6.2f  %6.2f' % (d, bd['risk'], bd['total']))
print()

# Tracking-noise sensitivity: d_free swings ±0.05 around 0.20
print('Sensitivity to ±0.05 noise:')
for base in [0.10, 0.20, 0.30]:
    side_scores['d_free_L'] = base - 0.05
    s_lo, _ = score_plan(PLAN_LEFT_PASS, ego_n=0.0, side_scores=side_scores, obs_in_horizon=True)
    side_scores['d_free_L'] = base + 0.05
    s_hi, _ = score_plan(PLAN_LEFT_PASS, ego_n=0.0, side_scores=side_scores, obs_in_horizon=True)
    print('  base=%.2f: low=%.2f high=%.2f delta=%.2f' % (base, s_lo, s_hi, s_lo - s_hi))
