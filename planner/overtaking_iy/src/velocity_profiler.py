"""GG-based forward-backward velocity profile for rolling-horizon overtake.

Thin wrapper around vel_planner.calc_vel_profile so the rolling planner can
compute v(s) along its refined d(s) with IC matching (v(s_0) = v_ego_current).

GG and machine limits are loaded from
    stack_master/config/<racecar>/veh_dyn_info/{ggv.csv, ax_max_machines.csv, b_ax_max_machines.csv}
following the same convention used by
[stack_master/scripts/global_velocity_planner_3d.py:43-52](stack_master/scripts/global_velocity_planner_3d.py#L43-L52).
"""

from __future__ import annotations

import os

import numpy as np
from vel_planner.vel_planner import calc_vel_profile
import trajectory_planning_helpers as tph


class VelocityProfiler:
    """Load GG once, reuse FB pass every cycle.

    Usage:
        vp = VelocityProfiler(stack_master_cfg_dir='.../stack_master/config',
                              racecar_version='SRX1')
        v = vp.profile(kappa=..., el_lengths=..., v_start=v_ego, v_max=...)
    """

    def __init__(self,
                 stack_master_cfg_dir: str,
                 racecar_version: str,
                 m_veh: float = 3.0,
                 drag_coeff: float = 0.01,
                 dyn_model_exp: float = 1.0):
        self.m_veh = m_veh
        self.drag_coeff = drag_coeff
        self.dyn_model_exp = dyn_model_exp

        veh_dyn_dir = os.path.join(stack_master_cfg_dir, racecar_version,
                                   'veh_dyn_info')
        ggv_path = os.path.join(veh_dyn_dir, 'ggv.csv')
        ax_path = os.path.join(veh_dyn_dir, 'ax_max_machines.csv')
        b_ax_path = os.path.join(veh_dyn_dir, 'b_ax_max_machines.csv')

        self.ggv, self.ax_max_machines = tph.import_veh_dyn_info.\
            import_veh_dyn_info(ggv_import_path=ggv_path,
                                ax_max_machines_import_path=ax_path)
        _, self.b_ax_max_machines = tph.import_veh_dyn_info.\
            import_veh_dyn_info(ggv_import_path=ggv_path,
                                ax_max_machines_import_path=b_ax_path)

        # default v_max covered by tables
        self.v_max_default = float(min(self.ggv[-1, 0],
                                       self.ax_max_machines[-1, 0]))

    def profile(self,
                kappa: np.ndarray,
                el_lengths: np.ndarray,
                v_start: float,
                v_end: float | None = None,
                v_max: float | None = None,
                filt_window: int | None = None) -> np.ndarray:
        """Run FB pass on an unclosed path.

        Contract (matches vel_planner.calc_vel_profile):
          len(kappa) = len(el_lengths) + 1
          v_start is IC at first knot.

        Returns vx profile of shape (len(kappa),).
        """
        if v_max is None:
            v_max = self.v_max_default
        if v_start is None or v_start < 0.0:
            v_start = 0.0
        return calc_vel_profile(
            ggv=self.ggv,
            ax_max_machines=self.ax_max_machines,
            b_ax_max_machines=self.b_ax_max_machines,
            kappa=kappa,
            el_lengths=el_lengths,
            closed=False,
            drag_coeff=self.drag_coeff,
            m_veh=self.m_veh,
            v_max=v_max,
            v_start=v_start,
            v_end=v_end,
            dyn_model_exp=self.dyn_model_exp,
            filt_window=filt_window,
        )
