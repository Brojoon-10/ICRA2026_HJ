from __future__ import annotations

import os
from typing import TYPE_CHECKING, Tuple, List

# add state_machine src to path
import rospkg as _rospkg
import sys as _sys
_sm_src = os.path.join(_rospkg.RosPack().get_path('state_machine'), 'src')
if _sm_src not in _sys.path:
    _sys.path.insert(0, _sm_src)

from states_types import StateType
import rospy
from f110_msgs.msg import Wpnt
import states


if TYPE_CHECKING:
    from state_machine_node import StateMachine

_debug_log_cache = {}
DEBUG_LOGGING_ENABLED = False

def debug_log_on_change(tag, **kwargs):
    global _debug_log_cache
    if not DEBUG_LOGGING_ENABLED:
        return
    cache_key = tag
    prev_values = _debug_log_cache.get(cache_key, None)
    if prev_values != kwargs:
        msg_parts = [f"{k}={v}" for k, v in kwargs.items()]
        rospy.logwarn(f"[DEBUG {tag}] " + ", ".join(msg_parts))
        _debug_log_cache[cache_key] = kwargs.copy()


def GlobalTrackingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.GB_TRACK`"""
    close_to_gb = state_machine._check_close_to_raceline(state_machine._get_adaptive_close_threshold()) * state_machine._check_close_to_raceline_heading(20)

    if len(state_machine.cur_obstacles_in_interest) == 0:
        return NonObstacleTransition(state_machine, close_to_gb)
    else:
        return ObstacleTransition(state_machine, close_to_gb)


def RecoveryTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    close_to_gb = state_machine._check_close_to_raceline(state_machine._get_adaptive_close_threshold()) * state_machine._check_close_to_raceline_heading(20)
    recovery_sustainability = state_machine._check_sustainability(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)

    if recovery_sustainability and not close_to_gb:
        return StateType.RECOVERY, StateType.RECOVERY
    return GlobalTrackingTransition(state_machine)


def TrailingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.TRAILING`"""
    close_to_gb = state_machine._check_close_to_raceline(state_machine._get_adaptive_close_threshold()) * state_machine._check_close_to_raceline_heading(20)

    if len(state_machine.cur_obstacles_in_interest) == 0:
        return NonObstacleTransition(state_machine, close_to_gb)
    else:
        if state_machine._check_ftg():
            return StateType.FTGONLY, StateType.FTGONLY
        return ObstacleTransition(state_machine, close_to_gb)


def OvertakingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.OVERTAKE`"""
    enemy = state_machine._check_enemy_in_front()
    path_collision = state_machine._check_ot_path_collision()
    too_close = state_machine._opponent_dist < state_machine._opponent_dist_threshold

    if path_collision or too_close:
        state_machine.overtaking_ttl_count = 0
        return StateType.TRAILING, StateType.GB_TRACK

    if enemy:
        state_machine.overtaking_ttl_count = 0
        return StateType.OVERTAKE, StateType.OVERTAKE

    if state_machine.overtaking_ttl_count < state_machine.overtaking_ttl_count_threshold:
        state_machine.overtaking_ttl_count += 1
        return StateType.OVERTAKE, StateType.OVERTAKE

    state_machine.overtaking_ttl_count = 0
    return GlobalTrackingTransition(state_machine)


def StartTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    start_free = state_machine._check_free_cartesian(state_machine.cur_start_wpnts)
    on_spline = state_machine._check_on_spline(state_machine.cur_start_wpnts)

    if start_free and on_spline:
        return StateType.START, StateType.START
    else:
        state_machine.cur_start_wpnts.is_init = False
        return GlobalTrackingTransition(state_machine)


def FTGOnlyTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    close_to_gb = state_machine._check_close_to_raceline(state_machine._get_adaptive_close_threshold()) * state_machine._check_close_to_raceline_heading(20)

    if len(state_machine.cur_obstacles_in_interest) == 0:
        return NonObstacleTransition(state_machine, close_to_gb)
    else:
        if close_to_gb and state_machine._check_free_frenet(state_machine.cur_gb_wpnts):
            return StateType.GB_TRACK, StateType.GB_TRACK

        recovery_availability = state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
        if recovery_availability and state_machine._check_free_frenet(state_machine.cur_recovery_wpnts):
            return StateType.RECOVERY, StateType.RECOVERY

        if state_machine._check_static_overtaking_mode():
            return StateType.OVERTAKE, StateType.OVERTAKE
        if state_machine._check_overtaking_mode():
            return StateType.OVERTAKE, StateType.OVERTAKE
        else:
            return StateType.FTGONLY, StateType.FTGONLY


def NonObstacleTransition(state_machine: StateMachine, close_to_gb: bool) -> Tuple[StateType, StateType]:
    """No obstacles — try GB_TRACK, else recovery, else lostline."""
    if close_to_gb:
        return StateType.GB_TRACK, StateType.GB_TRACK

    if state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts):
        if state_machine._check_on_spline(state_machine.cur_recovery_wpnts):
            return StateType.RECOVERY, StateType.RECOVERY

    return StateType.LOSTLINE, StateType.GB_TRACK


def ObstacleTransition(state_machine: StateMachine, close_to_gb: bool) -> Tuple[StateType, StateType]:
    """Obstacles present — try GB if free, recovery, overtake, or trailing."""
    gb_path_free = state_machine._check_free_frenet(state_machine.cur_gb_wpnts)

    # Priority 1: GB path close and free
    if close_to_gb and gb_path_free:
        return StateType.GB_TRACK, StateType.GB_TRACK

    # Priority 2: Recovery (only if not close to GB)
    recovery_availability = False
    if not close_to_gb:
        recovery_availability = state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
        if recovery_availability and state_machine._check_free_frenet(state_machine.cur_recovery_wpnts):
            return StateType.RECOVERY, StateType.RECOVERY

    # Priority 3: Overtaking — ot_sector + overtaking_iy path required
    if (state_machine._check_ot_sector()
            and state_machine._ot_iy_wpnts is not None
            and not state_machine._check_ot_path_collision()):
        return StateType.OVERTAKE, StateType.OVERTAKE

    # Priority 4: TRAILING (overtake conditions not met)
    if close_to_gb:
        return StateType.TRAILING, StateType.GB_TRACK
    elif recovery_availability:
        return StateType.TRAILING, StateType.RECOVERY
    elif gb_path_free:
        return StateType.TRAILING, StateType.GB_TRACK
    else:
        return StateType.TRAILING, StateType.GB_TRACK
