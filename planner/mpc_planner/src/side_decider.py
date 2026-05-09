#!/usr/bin/env python3
"""### HJ : External side-of-passing decision (LEFT / RIGHT / TRAIL / CLEAR).

v4 (2026-04-27): ego-aware multi-obstacle.
  - Each obstacle in obs_list contributes its own d_free_L / d_free_R; a
    side is feasible only if EVERY obstacle can be passed on that side
    (AND combination). This stops the "obs_list[0]-only" blindspot where
    a second obstacle on the opposite side was ignored.
  - Lateral reach feasibility: ego at current n must be able to reach the
    target side within the longitudinal distance to the obstacle. Uses a
    simple kinematic budget (lateral_budget = ego_v · sin(mu_max) · ds/v)
    so a side stays infeasible when ego is too lateral-far to swing in
    time.
  - Switching safety: if the new decision flips ego's prev_side
    (LEFT↔RIGHT) AND ego is alongside / too close to obstacle to swing
    safely, force SIDE_TRAIL. ego brakes and waits until the swing is
    geometrically achievable, then commits.

v3 (2026-04-21): feasibility-aware. "Can the ego physically fit past the
obstacle on this side?" is checked BEFORE the NLP. If neither side fits,
TRAIL is forced; the MPC then sees v_max capped by the obstacle's speed
and the continuity cost pulls the trajectory smoothly back to centerline.

Inputs per tick:
  - ego_v (m/s), ego_n (m), ego_s (m)
  - obs list, each with (s0, n0, v_s_obs, half_width, d_L_at_obs, d_R_at_obs)
  - ref_v_at_obs (target speed at obstacle station)

Output:
  side_int   int  (SIDE_{CLEAR,LEFT,RIGHT,TRAIL})
  side_str   str  ("clear"/"left"/"right"/"trail")
  scores     dict — d_free_L/R per side (worst across obstacles),
                    can_pass_L/R, can_reach_L/R, dv, reason, ...

Hysteresis: chosen side persists `hold_ticks` before a LEFT↔RIGHT flip.
TRAIL entry uses `trail_entry_ticks` (smaller) to react faster to narrow
corridors — "못 갈 것 같으면 바로 감속" semantics the user asked for.
"""

import math

SIDE_CLEAR = 0
SIDE_LEFT = 1
SIDE_RIGHT = 2
SIDE_TRAIL = 3

_NAME = {SIDE_CLEAR: 'clear', SIDE_LEFT: 'left',
         SIDE_RIGHT: 'right', SIDE_TRAIL: 'trail'}


class SideDecider:
    def __init__(self,
                 ego_half_width=0.15,
                 gap_lat=0.25,
                 trail_dv_thresh=0.5,
                 hold_ticks=5,
                 min_pass_margin=0.10,
                 trail_entry_ticks=3,
                 wall_safe=0.15,
                 inflation=0.05,
                 mu_max_for_reach=0.9,
                 swing_safe_distance=2.0,
                 immediate_decision_range=5.0,
                 sequential_pair_window=2.0):
        self.ego_half = float(ego_half_width)
        self.gap_lat = float(gap_lat)
        self.trail_dv_thresh = float(trail_dv_thresh)
        self.hold_ticks = int(hold_ticks)
        self.min_pass_margin = float(min_pass_margin)
        self.trail_entry_ticks = int(trail_entry_ticks)
        self.wall_safe = float(wall_safe)
        self.inflation = float(inflation)
        # ### HJ : v4 — ego-aware reach feasibility + switching safety.
        # mu_max_for_reach: kinematic limit used to compute lateral budget
        #   (ego can move v·sin(mu) laterally per second).
        # swing_safe_distance: minimum forward distance to obstacle for a
        #   safe LEFT↔RIGHT swing. Below this, force SIDE_TRAIL during
        #   the switch.
        self.mu_max_for_reach = float(mu_max_for_reach)
        self.swing_safe_distance = float(swing_safe_distance)
        # ### HJ : v4b — immediate-decision range. Only obstacles within
        # this forward distance affect the immediate side decision; far
        # obstacles get re-evaluated when ego approaches them.
        # sequential_pair_window: if a far obstacle is within this distance
        # of the closest one, treat them as a "pair" (joint decision) so
        # ego doesn't pick a side that immediately blocks for the next.
        self.immediate_decision_range = float(immediate_decision_range)
        self.sequential_pair_window = float(sequential_pair_window)

        self._prev_side = SIDE_CLEAR
        self._pending_side = SIDE_CLEAR
        self._pending_streak = 0
        self.last_scores = {}

    def _per_obstacle_d_free(self, o):
        """Geometric d_free_L / d_free_R for ONE obstacle (raceline-frame).
        Returns (d_free_L, d_free_R) — how much lateral room remains on
        each side past this obstacle, after subtracting ego_half + gap_lat
        and the solver's effective wall margin (wall_safe + inflation).
        """
        n_o = float(o['n0'])
        w_o = float(o['half_width'])
        dL = float(o['d_L'])
        dR = float(o['d_R'])
        eff_dL = dL - self.wall_safe - self.inflation
        eff_dR = dR - self.wall_safe - self.inflation
        d_free_L = eff_dL - (n_o + w_o) - (self.ego_half + self.gap_lat)
        d_free_R = (n_o - w_o) - (-eff_dR) - (self.ego_half + self.gap_lat)
        return d_free_L, d_free_R

    def _ego_reach(self, ego_n, ego_v, ego_s, o):
        """### HJ : v4 — can ego REACH the lateral target side in time?
        Estimates lateral budget = v · sin(mu_max) · t_avail where
        t_avail = ds_forward / max(ego_v, ε). If lateral_required exceeds
        that budget, the side is unreachable regardless of d_free.
        Returns (can_reach_L, can_reach_R, lat_req_L, lat_req_R, lat_budget).
        """
        n_o = float(o['n0'])
        w_o = float(o['half_width'])
        s_o = float(o['s0'])
        # forward distance — clamp to small positive so alongside obstacles
        # also get a (small) reach window rather than divide-by-zero.
        ds_forward = max(s_o - float(ego_s), 0.05)
        v_eff = max(float(ego_v), 0.5)
        t_avail = ds_forward / v_eff
        # lateral budget: v · sin(mu_max) · t_avail
        lat_budget = v_eff * math.sin(self.mu_max_for_reach) * t_avail
        # target n for each side (where ego center must be at obstacle's s)
        target_n_L = n_o + w_o + self.ego_half + self.gap_lat
        target_n_R = n_o - w_o - self.ego_half - self.gap_lat
        # ego needs to MOVE this much laterally
        lat_req_L = max(target_n_L - float(ego_n), 0.0)
        lat_req_R = max(float(ego_n) - target_n_R, 0.0)
        can_reach_L = lat_req_L <= lat_budget + 1e-3
        can_reach_R = lat_req_R <= lat_budget + 1e-3
        return can_reach_L, can_reach_R, lat_req_L, lat_req_R, lat_budget

    def _swing_safe(self, ego_s, obs_list, new_side):
        """### HJ : v4 — is it safe to swing to the new side right now?
        If we're switching from prev_side to its opposite (LEFT↔RIGHT),
        AND any forward obstacle is closer than swing_safe_distance, the
        swing path crosses through obstacle's keep-out — bad.
        Returns True if switching is safe (or no switch).
        """
        if self._prev_side not in (SIDE_LEFT, SIDE_RIGHT):
            return True
        if new_side not in (SIDE_LEFT, SIDE_RIGHT):
            return True
        if new_side == self._prev_side:
            return True
        # We're flipping LEFT→RIGHT or RIGHT→LEFT. Check forward obstacle proximity.
        for o in obs_list:
            ds_forward = float(o['s0']) - float(ego_s)
            if 0.0 <= ds_forward < self.swing_safe_distance:
                return False  # obstacle alongside or too close — swing dangerous
        return True

    def decide(self, ego_v, obs_list, ego_n=None, ego_s=None):
        # Caller may not pass ego_n/ego_s yet — fall back to legacy mode in
        # that case (no ego-reach check, no swing-safe check).
        ego_n_val = float(ego_n) if ego_n is not None else None
        ego_s_val = float(ego_s) if ego_s is not None else None

        if not obs_list:
            raw = SIDE_CLEAR
            scores = {'reason': 'no_obstacle'}
        else:
            # ### HJ : v4b — sequential vs immediate obstacle aggregation.
            # Bag scenarios with 2+ obstacles showed: AND across ALL
            # obstacles (one needs LEFT, another needs RIGHT) = always
            # TRAIL even when ego could pass them sequentially. Fix:
            # - Closest forward obstacle = primary decision driver
            # - Other obstacles in `sequential_pair_window` of closest =
            #   joint decision (paired AND)
            # - Far obstacles (beyond `immediate_decision_range`) ignored
            #   for THIS tick — they'll trigger their own decision when
            #   ego approaches.
            o0 = obs_list[0]
            s0_first = float(o0['s0'])
            ego_s_ref = (ego_s_val if ego_s_val is not None
                         else s0_first - 0.1)
            relevant = []
            for o in obs_list:
                ds_obs = float(o['s0']) - ego_s_ref
                if ds_obs > self.immediate_decision_range:
                    continue   # far ahead → not yet
                # Either: alongside ego (ds≤0 small), or within pair window
                # of the closest one
                ds_from_first = float(o['s0']) - s0_first
                if ds_from_first <= self.sequential_pair_window:
                    relevant.append(o)
            if not relevant:
                relevant = [o0]   # safety: at least the closest one

            ### HJ : 2026-04-27 (v4c reverted) — static/dynamic distinction
            ###      is NOT applied at decision time. Reason (per user):
            ###      "동적이 먼저 있고 정적이 그 뒤" 케이스에서 정적-only
            ###      결정은 동적을 무시 → ego 가 정적의 쪽으로 가다가 동적
            ###      박음. 올바른 policy 는 둘 다 AND. 정적/동적 구분은
            ###      TRAIL semantics (감속 vs 회피) 에서만.
            ###
            ###      Below feasibility loop iterates `relevant` (already
            ###      range/pair filtered). Static & dynamic both contribute.
            ###      If static says "must go RIGHT" but dynamic on RIGHT in
            ###      between → AND fails → TRAIL → ego brakes → dynamic
            ###      moves → re-decide with only static remaining → commit.
            decision_obs = relevant
            ### HJ : end

            can_pass_L_all = True
            can_pass_R_all = True
            can_reach_L_all = True
            can_reach_R_all = True
            d_free_L_min = float('inf')
            d_free_R_min = float('inf')
            lat_budget_min = float('inf')

            for o in relevant:
                df_L, df_R = self._per_obstacle_d_free(o)
                if df_L < d_free_L_min: d_free_L_min = df_L
                if df_R < d_free_R_min: d_free_R_min = df_R
                if df_L < self.min_pass_margin: can_pass_L_all = False
                if df_R < self.min_pass_margin: can_pass_R_all = False
                if ego_n_val is not None and ego_s_val is not None:
                    rL, rR, _lrL, _lrR, lb = self._ego_reach(
                        ego_n_val, ego_v, ego_s_val, o)
                    if not rL: can_reach_L_all = False
                    if not rR: can_reach_R_all = False
                    if lb < lat_budget_min: lat_budget_min = lb

            # First obstacle drives the dv check (closest forward obstacle).
            v_obs = float(o0.get('v_s_obs', 0.0))
            ref_v = float(o0.get('ref_v', ego_v))
            dv = ref_v - v_obs

            # Combined feasibility: corridor + ego reach.
            can_pass_L = can_pass_L_all and can_reach_L_all
            can_pass_R = can_pass_R_all and can_reach_R_all

            scores = {'d_free_L': round(d_free_L_min, 3),
                      'd_free_R': round(d_free_R_min, 3),
                      'can_pass_L_corr': bool(can_pass_L_all),
                      'can_pass_R_corr': bool(can_pass_R_all),
                      'can_reach_L': bool(can_reach_L_all),
                      'can_reach_R': bool(can_reach_R_all),
                      'can_pass_L': bool(can_pass_L),
                      'can_pass_R': bool(can_pass_R),
                      'dv': round(dv, 3),
                      'n_o': round(float(o0['n0']), 3),
                      'v_obs': round(v_obs, 3),
                      'd_L_raw': round(float(o0['d_L']), 3),
                      'd_R_raw': round(float(o0['d_R']), 3),
                      'half_w': round(float(o0['half_width']), 3),
                      'min_pass_margin': round(self.min_pass_margin, 3),
                      'n_obs_seen': int(len(obs_list)),
                      'n_obs_relevant': int(len(relevant)),
                      'lat_budget': round(lat_budget_min, 3)
                                    if lat_budget_min != float('inf') else None}

            if dv < self.trail_dv_thresh:
                raw = SIDE_TRAIL
                scores['reason'] = 'dv_small'
            elif not can_pass_L and not can_pass_R:
                # ### HJ : 2026-04-28 — best_effort_left/right REVERTED to
                # SIDE_TRAIL. The earlier best_effort branch was forcing
                # LEFT/RIGHT even when both sides reported can_pass=False
                # (both d_free < gap_lat), so the solver was pushed
                # laterally toward an obstacle that did not actually have
                # room. User log: "SIDE → LEFT ... d_free_L=-0.195
                # d_free_R=-0.200 reason=best_effort_left" → solver tried
                # to pass through the obstacle. Stage 2 plan_library
                # already has a proper TRAIL plan (target_n=ego_n hold +
                # obs.vs - margin ref_v) that handles the no-pass case
                # safely; route the side_decider through it.
                raw = SIDE_TRAIL
                scores['reason'] = 'no_side_fits'
            elif can_pass_L and not can_pass_R:
                raw = SIDE_LEFT
                scores['reason'] = 'only_left_fits'
            elif can_pass_R and not can_pass_L:
                raw = SIDE_RIGHT
                scores['reason'] = 'only_right_fits'
            elif d_free_L_min >= d_free_R_min:
                raw = SIDE_LEFT
                scores['reason'] = 'left_wider'
            else:
                raw = SIDE_RIGHT
                scores['reason'] = 'right_wider'

            ### HJ : v4 — switching-safety override.
            ### If the raw decision flips prev_side (LEFT↔RIGHT) AND ego
            ### is too close to obstacle to swing safely, force TRAIL. ego
            ### brakes until swing is achievable; then SideDecider commits
            ### the new side cleanly.
            if (ego_s_val is not None
                    and raw in (SIDE_LEFT, SIDE_RIGHT)
                    and not self._swing_safe(ego_s_val, obs_list, raw)):
                scores['raw_pre_swing_guard'] = _NAME.get(raw, '?')
                raw = SIDE_TRAIL
                scores['reason'] = 'swing_unsafe'

        # Hysteresis. TRAIL entry fires faster (trail_entry_ticks) to
        # bail out of infeasible overtakes quickly; LEFT↔RIGHT uses the
        # longer hold_ticks to damp flips.
        if raw == self._prev_side:
            self._pending_side = raw
            self._pending_streak = 0
            final = raw
        else:
            if raw == self._pending_side:
                self._pending_streak += 1
            else:
                self._pending_side = raw
                self._pending_streak = 1

            gate = (self.trail_entry_ticks if raw == SIDE_TRAIL
                    else self.hold_ticks)
            if self._pending_streak >= gate:
                final = raw
                self._prev_side = raw
                self._pending_streak = 0
            else:
                final = self._prev_side
                scores['held'] = True
                scores['pending'] = _NAME.get(raw, '?')
                scores['pending_streak'] = self._pending_streak
                scores['gate'] = gate

        self.last_scores = scores
        return final, _NAME.get(final, '?'), scores
