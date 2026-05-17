#!/usr/bin/env python3
# IY 2026-05-17 : FBGA-enabled state_machine.
# Subclass of 3d_state_machine_node.StateMachine. Overrides update_velocity()
# so RECOVERY / SMART_STATIC use the real C++ FBGA (libs/FBGA) for vx_mps;
# other states keep the parent legacy 2.5d velocity profile.

import os
import sys
import importlib.util

import numpy as np
import rospy
from rospkg import RosPack


# Load 3d_state_machine_node.py via importlib (filename starts with a digit).
_SM_SRC = os.path.join(RosPack().get_path('state_machine'), 'src')
if _SM_SRC not in sys.path:
    sys.path.insert(0, _SM_SRC)

_sm_path = os.path.join(_SM_SRC, '3d_state_machine_node.py')
_spec = importlib.util.spec_from_file_location('sm3d', _sm_path)
sm3d = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sm3d)

from states_types import StateType
from fbga_runner import FBGARunner


class FBGAStateMachine(sm3d.StateMachine):
    """Recomputes vx_mps for RECOVERY + SMART_STATIC via the real C++ FBGA.

    Parent state transitions, waypoint multiplexing and publish are untouched.
    Only update_velocity is overridden.
    """

    def __init__(self, name):
        # Initialise FBGA attributes BEFORE super().__init__ because the
        # parent constructor registers ROS subscribers that may fire while
        # we are still inside __init__; if our overridden update_velocity
        # is called before these attributes exist we get AttributeError.
        self._fbga = None
        self.fbga_states = set()
        self.v_scale_sm = 1.0
        self._fbga_max_fail_streak = 3
        self._fbga_fail = {}
        self._fbga_disabled_states = set()

        super().__init__(name)

        ## IY 2026-05-17 : params
        states_param = rospy.get_param('~fbga_states',
                                       ['RECOVERY', 'OVERTAKE', 'START'])
        self.fbga_states = {StateType[s] for s in states_param}
        self.v_scale_sm = float(rospy.get_param('~v_scale', 1.0))
        # While diagnosing, default to a very high streak so failures don't
        # permanently disable a state -- we want to keep seeing the reason.
        self._fbga_max_fail_streak = int(rospy.get_param('~fbga_max_fail_streak', 100000))

        # FBGARunner init (graceful fallback to legacy on failure).
        # Empty-string rosparam is treated as unset and replaced by the
        # built-in default; this lets launch files leave fbga_*_bin blank
        # and still get the canonical race_stack paths.
        # _SM_SRC = <race_stack>/state_machine/src  ->  ../.. = <race_stack>
        race_stack = os.path.abspath(os.path.join(_SM_SRC, '..', '..'))
        default_fbga_bin = os.path.join(race_stack, 'f110_utils', 'libs',
            'FBGA', 'bin', 'GIGI_test_unicorn.exe')
        default_gg_bin = os.path.join(race_stack, 'planner', '3d_gb_optimizer',
            'global_line', 'data', 'gg_diagrams', 'rc_car_10th_latest',
            'velocity_frame', 'gg.bin')
        default_params_yml = os.path.join(race_stack, 'planner',
            '3d_gb_optimizer', 'global_line', 'data', 'vehicle_params',
            'params_rc_car_10th_latest.yml')

        fbga_bin   = rospy.get_param('~fbga_bin',        '') or default_fbga_bin
        gg_bin     = rospy.get_param('~fbga_gg_bin',     '') or default_gg_bin
        params_yml = rospy.get_param('~fbga_params_yml', '') or default_params_yml

        try:
            self._fbga = FBGARunner(
                fbga_bin=fbga_bin,
                gg_bin=gg_bin,
                params_yml=params_yml,
                enable_mu=bool(rospy.get_param('~fbga_enable_mu', True)),
                bridge_effect=float(rospy.get_param('~bridge_effect', 1.0)),
                alpha=float(rospy.get_param('~fbga_alpha', 0.5)),
                max_iter=int(rospy.get_param('~fbga_max_iter', 5)),
                tol=float(rospy.get_param('~fbga_tol', 0.05)),
                logger=rospy.loginfo,
            )
            rospy.loginfo(f'[FBGAStateMachine] FBGA runner ready '
                          f'(states={[s.name for s in self.fbga_states]}, '
                          f'v_scale={self.v_scale_sm:.2f})')
        except Exception as e:
            rospy.logwarn(f'[FBGAStateMachine] FBGA init failed; legacy '
                          f'for all states: {e}')
            self._fbga = None

    # ── override
    def update_velocity(self, wpnts_msg, safety_factor=1.0):
        st = getattr(self, 'cur_state', None)
        use_fbga = (
            self._fbga is not None
            and st in self.fbga_states
            and st not in self._fbga_disabled_states
        )
        if not use_fbga:
            super().update_velocity(wpnts_msg, safety_factor)
        else:
            ok = self._update_velocity_fbga(wpnts_msg, safety_factor, st)
            if not ok:
                self._fbga_fail[st] = self._fbga_fail.get(st, 0) + 1
                if self._fbga_fail[st] >= self._fbga_max_fail_streak:
                    rospy.logwarn(
                        f'[FBGAStateMachine] {st.name} FBGA failed '
                        f'{self._fbga_fail[st]}x -> disable for episode')
                    self._fbga_disabled_states.add(st)
                super().update_velocity(wpnts_msg, safety_factor)
            else:
                self._fbga_fail[st] = 0

        if self.v_scale_sm != 1.0:
            for w in wpnts_msg.wpnts:
                w.vx_mps = float(w.vx_mps) * self.v_scale_sm

    def _update_velocity_fbga(self, wpnts_msg, sf, st) -> bool:
        # logwarn (not loginfo) is used everywhere so messages survive
        # log_level=WARN set in rospy.init_node.
        wpnts = wpnts_msg.wpnts
        n = len(wpnts)
        if n < 5:
            rospy.logwarn_throttle(1.0,
                f'[FBGA-DIAG] {st.name} skip: n={n} too small')
            return False

        s     = np.array([w.s_m         for w in wpnts])
        kappa = np.array([w.kappa_radpm for w in wpnts])
        mu    = np.array([getattr(w, 'mu_rad', 0.0) for w in wpnts])
        v_seed = np.array([w.vx_mps     for w in wpnts])

        # Unwrap s if the path crosses the start/finish line.
        # wpnt.s_m comes from frenet coords and wraps at track_length;
        # FBGA needs strictly-monotone s. Detect large negative jumps and
        # add track_length to every point after the jump.
        track_length = float(getattr(self, 'track_length', 0.0)) or 0.0
        if track_length <= 0.0 and self.gb_wpnts is not None:
            track_length = float(self.gb_wpnts.wpnts[-1].s_m) + self.wpnt_dist
        ds_raw = np.diff(s)
        jumps = np.where(ds_raw < -track_length * 0.5)[0]
        if len(jumps) > 0 and track_length > 0.0:
            for j in jumps:
                s[j + 1:] = s[j + 1:] + track_length

        ds = np.diff(s)
        rospy.logwarn_throttle(1.0,
            f'[FBGA-DIAG] {st.name} input n={n} '
            f's=[{s.min():.2f},{s.max():.2f}] ds=[{ds.min():.3f},{ds.max():.3f}] '
            f'kappa=[{kappa.min():.3f},{kappa.max():.3f}] '
            f'mu=[{mu.min():.3f},{mu.max():.3f}] '
            f'v_seed=[{v_seed.min():.2f},{v_seed.max():.2f}] '
            f'unwrapped_jumps={len(jumps)}')

        # Sanity: s must be monotone-increasing after unwrap.
        if ds.min() <= 0:
            rospy.logwarn_throttle(1.0,
                f'[FBGA-DIAG] {st.name} FAIL: s still not monotone after unwrap, '
                f'ds.min()={ds.min():.4f} at idx={int(np.argmin(ds))} '
                f'(track_length={track_length:.2f})')
            return False

        t0 = rospy.Time.now()
        try:
            res = self._fbga.solve_open(
                s, kappa, mu,
                v_seed=v_seed if np.any(v_seed > 0.1) else None,
            )
        except Exception as exc:
            rospy.logwarn(
                f'[FBGA-DIAG] {st.name} FAIL: solve_open raised {type(exc).__name__}: {exc}')
            return False
        solve_ms = (rospy.Time.now() - t0).to_sec() * 1000.0

        if res is None:
            rospy.logwarn_throttle(1.0,
                f'[FBGA-DIAG] {st.name} FAIL: solve_open returned None '
                f'(after {solve_ms:.0f} ms)')
            return False
        v_new, ax_new = res
        if v_new.shape[0] != n or ax_new.shape[0] != n:
            rospy.logwarn_throttle(1.0,
                f'[FBGA-DIAG] {st.name} FAIL: length mismatch '
                f'v={v_new.shape[0]} ax={ax_new.shape[0]} expected={n}')
            return False

        # v_max clamp (safety_factor applied).
        v_max = self.pars["veh_params"]["v_max"] * float(sf)
        v_new = np.minimum(v_new, v_max)

        # v_end clamp at the global-waypoint join (mirrors legacy L1537).
        try:
            gi = int(wpnts[-1].s_m / self.wpnt_dist)
            v_end_gb = self.gb_wpnts.wpnts[gi % len(self.gb_wpnts.wpnts)].vx_mps
            v_new[-1] = min(v_new[-1], v_end_gb)
        except Exception:
            pass  # gb_wpnts not yet received

        for i in range(n):
            wpnts[i].vx_mps  = float(v_new[i])
            wpnts[i].ax_mps2 = float(ax_new[i])

        rospy.logwarn_throttle(1.0,
            f'[FBGA-DIAG] {st.name} OK solve_ms={solve_ms:.0f} '
            f'v=[{v_new.min():.2f},{v_new.max():.2f}] '
            f'ax=[{ax_new.min():.2f},{ax_new.max():.2f}]')
        return True


if __name__ == "__main__":
    name = "state_machine"
    rospy.init_node(name, anonymous=False, log_level=rospy.WARN)

    sm = FBGAStateMachine(name)

    rospy.on_shutdown(sm.on_shutdown)

    loop_rate = rospy.Rate(sm.rate_hz)
    while not rospy.is_shutdown():
        sm.loop()
        loop_rate.sleep()
