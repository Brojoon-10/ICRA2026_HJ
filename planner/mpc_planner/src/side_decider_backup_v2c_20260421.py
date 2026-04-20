#!/usr/bin/env python3
"""### HJ : External side-of-passing decision (LEFT / RIGHT / TRAIL / CLEAR).

Kept OUT of the NLP because isotropic cost in Frenet makes side ambiguous
(Gaussian obstacle penalty gives equivalent cost for +epsilon or -epsilon
detours around the same obstacle, so the solver oscillates). Rule-based
decision with a small hysteresis window removes that instability and lets
the MPC enforce the chosen side as a one-sided half-plane hard constraint.

Inputs per tick:
  - ego_v (m/s)
  - obs list, each with (s0, n0, v_s_obs, half_width, d_L_at_obs, d_R_at_obs)
  - ref_v_at_obs (target speed at obstacle station)

Output:
  side_int   int  (SIDE_{CLEAR,LEFT,RIGHT,TRAIL})
  side_str   str  ("clear"/"left"/"right"/"trail")
  scores     dict {'d_free_L', 'd_free_R', 'dv', 'reason'}

Hysteresis: the chosen side must persist for `hold_ticks` before flipping.
"""

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
                 trail_dv_thresh=0.5,     # ego must be >=0.5 m/s faster
                 hold_ticks=5):
        self.ego_half = float(ego_half_width)
        self.gap_lat = float(gap_lat)
        self.trail_dv_thresh = float(trail_dv_thresh)
        self.hold_ticks = int(hold_ticks)

        self._prev_side = SIDE_CLEAR
        self._pending_side = SIDE_CLEAR
        self._pending_streak = 0
        self.last_scores = {}

    def decide(self, ego_v, obs_list):
        """
        obs_list: list of dicts with keys:
           's0', 'n0', 'v_s_obs', 'half_width', 'd_L', 'd_R', 'ref_v'
        Empty/None → CLEAR.
        """
        if not obs_list:
            raw = SIDE_CLEAR
            scores = {'reason': 'no_obstacle'}
        else:
            # pick closest (smallest s0 forward-distance — caller already sorts)
            o = obs_list[0]
            n_o = float(o['n0'])
            w_o = float(o['half_width'])
            dL = float(o['d_L'])
            dR = float(o['d_R'])
            v_obs = float(o.get('v_s_obs', 0.0))
            ref_v = float(o.get('ref_v', ego_v))
            dv = ref_v - v_obs  # ego can overtake only if clearly faster

            # free lateral width to each side (positive = can fit)
            # left side: from obstacle's +n edge up to wall
            d_free_L = (dL) - (n_o + w_o) - (self.ego_half + self.gap_lat)
            # right side: from wall up to obstacle's -n edge
            d_free_R = (n_o - w_o) - (-dR) - (self.ego_half + self.gap_lat)

            scores = {'d_free_L': round(d_free_L, 3),
                      'd_free_R': round(d_free_R, 3),
                      'dv': round(dv, 3),
                      'n_o': round(n_o, 3),
                      'v_obs': round(v_obs, 3)}

            if dv < self.trail_dv_thresh:
                raw = SIDE_TRAIL
                scores['reason'] = 'dv_small'
            elif max(d_free_L, d_free_R) < 0.0:
                raw = SIDE_TRAIL
                scores['reason'] = 'no_side_fits'
            elif d_free_L >= d_free_R:
                raw = SIDE_LEFT
                scores['reason'] = 'left_wider'
            else:
                raw = SIDE_RIGHT
                scores['reason'] = 'right_wider'

        # Hysteresis
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
            if self._pending_streak >= self.hold_ticks:
                final = raw
                self._prev_side = raw
                self._pending_streak = 0
            else:
                final = self._prev_side
                scores['held'] = True
                scores['pending'] = _NAME.get(raw, '?')
                scores['pending_streak'] = self._pending_streak

        self.last_scores = scores
        return final, _NAME.get(final, '?'), scores
