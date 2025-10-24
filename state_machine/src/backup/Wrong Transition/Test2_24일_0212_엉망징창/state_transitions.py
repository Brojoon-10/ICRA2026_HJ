from __future__ import annotations

from typing import TYPE_CHECKING, Tuple, List

from states_types import StateType
import rospy
from f110_msgs.msg import Wpnt
import states


if TYPE_CHECKING:
    from state_machine_node import StateMachine
    from state_helper_for_smart import SmartStaticChecker  # ===== HJ ADDED =====
else:
    # ===== HJ ADDED: Runtime import =====
    from state_helper_for_smart import SmartStaticChecker
    # ===== HJ ADDED END =====

# ===== HJ ADDED: Global Smart Static checker instance =====
_smart_static_checker = None
# ===== HJ ADDED END =====

close_threshold_smart = 0.05

"""
Transitions should loosely follow the following template (basically a match-case)

if (logic sum of bools obtained by methods of state_machine):   
    return StateType.<DESIRED STATE>
elif (e.g. state_machine.obstacles are near):
    return StateType.<ANOTHER DESIRED STATE>
...

NOTE: ideally put the most common cases on top of the match-case

NOTE 2: notice that, when implementing new states, if an attribute/condition in the 
    StateMachine is not available, your IDE will tell you, but only if you have a smart 
    enough IDE. So use vscode, pycharm, fleet or whatever has specific python syntax highlights.

NOTE 3: transistions must not have side effects on the state machine! 
    i.e. any attribute of the state machine should not be modified in the transitions.
"""
def GlobalTrackingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.GB_TRACK`"""
    rospy.logwarn("========== GlobalTrackingTransition ENTERED ==========")
    # ===== HJ MODIFIED: Calculate both GB and Smart proximity, pass appropriate one based on smart_active =====
    close_to_gb = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
    close_to_smart = state_machine._check_close_to_smart_static_path(close_threshold_smart) * state_machine._check_close_to_smart_static_path_heading(20)
    smart_active = state_machine.smart_static_active
    rospy.logwarn(f"[GlobalTracking] close_to_gb={close_to_gb}, close_to_smart={close_to_smart}, smart_active={smart_active}, num_obs={len(state_machine.cur_obstacles_in_interest)}")

    if len(state_machine.cur_obstacles_in_interest) == 0:
        if smart_active:
            return NonObstacleTransition(state_machine, close_to_raceline=close_to_smart, smart_active=True)
        else:
            return NonObstacleTransition(state_machine, close_to_raceline=close_to_gb, smart_active=False)
    else:
        if smart_active:
            return ObstacleTransition(state_machine, close_to_raceline=close_to_smart, smart_active=True)
        else:
            return ObstacleTransition(state_machine, close_to_raceline=close_to_gb, smart_active=False)
    # ===== HJ MODIFIED END =====
   
def RecoveryTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.RECOVERY`"""
    rospy.logwarn("========== RecoveryTransition ENTERED ==========")
    recovery_sustainability = state_machine._check_sustainability(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)

    # ===== HJ MODIFIED: Calculate both GB and Smart proximity separately =====
    close_to_gb = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
    close_to_smart = state_machine._check_close_to_smart_static_path(close_threshold_smart) * state_machine._check_close_to_smart_static_path_heading(20)
    smart_active = state_machine.smart_static_active

    # Check if should stay in recovery (not close to reference path)
    close_to_ref = close_to_smart if smart_active else close_to_gb
    if recovery_sustainability and not close_to_ref:
        return StateType.RECOVERY, StateType.RECOVERY

    # Recovery ended - delegate to appropriate transition based on smart_active
    if smart_active:
        return SmartStaticTransition(state_machine)
    else:
        return GlobalTrackingTransition(state_machine)
    # ===== HJ MODIFIED END =====

def TrailingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.TRAILING`"""
    rospy.logwarn("========== TrailingTransition ENTERED ==========")
    # ===== HJ MODIFIED: Calculate both GB and Smart proximity, pass appropriate one based on smart_active =====
    close_to_gb = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
    close_to_smart = state_machine._check_close_to_smart_static_path(close_threshold_smart) * state_machine._check_close_to_smart_static_path_heading(20)
    smart_active = state_machine.smart_static_active

    if len(state_machine.cur_obstacles_in_interest) == 0:
        if smart_active:
            return NonObstacleTransition(state_machine, close_to_raceline=close_to_smart, smart_active=True)
        else:
            return NonObstacleTransition(state_machine, close_to_raceline=close_to_gb, smart_active=False)
    else:
        if state_machine._check_ftg():
            return StateType.FTGONLY, StateType.FTGONLY
        if smart_active:
            return ObstacleTransition(state_machine, close_to_raceline=close_to_smart, smart_active=True)
        else:
            return ObstacleTransition(state_machine, close_to_raceline=close_to_gb, smart_active=False)
    # ===== HJ MODIFIED END =====
            
def OvertakingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.OVERTAKE`"""
    rospy.logwarn("========== OvertakingTransition ENTERED ==========")
    ot_sustainability = state_machine._check_overtaking_mode_sustainability()
    enemy_in_front = state_machine._check_enemy_in_front()

    if ot_sustainability and enemy_in_front:
        state_machine.overtaking_ttl_count = 0
        return StateType.OVERTAKE, StateType.OVERTAKE
    if ot_sustainability and state_machine.overtaking_ttl_count < state_machine.overtaking_ttl_count_threshold:
        state_machine.overtaking_ttl_count += 1
        return StateType.OVERTAKE, StateType.OVERTAKE
    state_machine.overtaking_ttl_count = 0

    # ===== HJ MODIFIED: Overtaking ended - return to appropriate transition based on smart_active =====
    smart_active = state_machine.smart_static_active
    if smart_active:
        return SmartStaticTransition(state_machine)
    else:
        return GlobalTrackingTransition(state_machine)
    # ===== HJ MODIFIED END =====

def StartTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.START`"""
    start_free = state_machine._check_free_cartesian(state_machine.cur_start_wpnts)
    on_spline = state_machine._check_on_spline(state_machine.cur_start_wpnts)

    if start_free and on_spline:
        return StateType.START, StateType.START
    else:
        close_to_raceline = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
        state_machine.cur_start_wpnts.is_init = False
        return GlobalTrackingTransition(state_machine, close_to_raceline)
            
def FTGOnlyTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.FTGONLY`"""
    close_to_raceline = state_machine._check_close_to_raceline(close_threshold_smart) * state_machine._check_close_to_raceline_heading(20)

    if len(state_machine.cur_obstacles_in_interest) == 0:
        return NonObstacleTransition(state_machine, close_to_raceline)

    else:
        if close_to_raceline and state_machine._check_free_frenet(state_machine.cur_gb_wpnts):
            return StateType.GB_TRACK, StateType.GB_TRACK

        recovery_availability = state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
        if (recovery_availability and state_machine._check_free_frenet(state_machine.cur_recovery_wpnts)):
            return StateType.RECOVERY, StateType.RECOVERY

        if state_machine._check_overtaking_mode() or state_machine._check_static_overtaking_mode():
            return StateType.OVERTAKE, StateType.OVERTAKE

        else:
            return StateType.FTGONLY, StateType.FTGONLY

# ===== HJ ADDED: Smart Static Fixed Path Transition =====
def SmartStaticTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.SMART_STATIC`

    Follows Smart Static Fixed Path (GB Optimizer output).
    Uses GB Frenet coordinates (from topics).
    Checks proximity to Smart Static path (not GB raceline).
    """
    rospy.logwarn("========== SmartStaticTransition ENTERED ==========")
    # ===== HJ MODIFIED: Calculate both GB and Smart proximity, pass appropriate one =====
    close_to_gb = state_machine._check_close_to_raceline(0.05) * state_machine._check_close_to_raceline_heading(20)
    close_to_smart = state_machine._check_close_to_smart_static_path(close_threshold_smart) * state_machine._check_close_to_smart_static_path_heading(20)
    smart_active = state_machine.smart_static_active

    num_obstacles = len(state_machine.cur_obstacles_in_interest)

    rospy.loginfo_throttle(1.0,
        f"[SMART_STATIC→?] Transition Check:\n"
        f"  smart_active: {smart_active}\n"
        f"  close_to_gb: {close_to_gb}\n"
        f"  close_to_smart: {close_to_smart}\n"
        f"  num_obstacles: {num_obstacles}")

    # Delegate to standard transitions with smart_active flag
    if num_obstacles == 0:
        if smart_active:
            return NonObstacleTransition(state_machine, close_to_raceline=close_to_smart, smart_active=True)
        else:
            return NonObstacleTransition(state_machine, close_to_raceline=close_to_gb, smart_active=False)
    else:
        if smart_active:
            return ObstacleTransition(state_machine, close_to_raceline=close_to_smart, smart_active=True)
        else:
            return ObstacleTransition(state_machine, close_to_raceline=close_to_gb, smart_active=False)
    # ===== HJ MODIFIED END =====

##################################################################################################################
##################################################################################################################

def NonObstacleTransition(state_machine: StateMachine, close_to_raceline=False, smart_active=False) -> Tuple[StateType, StateType]:
    """Handle no obstacles case

    Args:
        close_to_raceline: True if close to Smart path (when smart_active=True) or GB raceline (when smart_active=False)
        smart_active: True if Smart Static mode is active
    """
    rospy.logwarn(f">>> NonObstacleTransition: close_to_raceline={close_to_raceline}, smart_active={smart_active}")
    # ===== HJ MODIFIED: Use smart_active parameter, check wpnts validity =====
    wpnts_valid = state_machine._check_latest_wpnts(
        state_machine.smart_static_wpnts,
        state_machine.cur_smart_static_avoidance_wpnts)

    rospy.loginfo_throttle(1.0,
        f"[NonObstacleTransition] Check:\n"
        f"  smart_active (param): {smart_active}\n"
        f"  wpnts_valid: {wpnts_valid}\n"
        f"  close_to_raceline: {close_to_raceline}")

    # Check if Smart Static path is available and close enough to use
    if smart_active and wpnts_valid and close_to_raceline:
        # Smart Static available and close to it - follow it
        rospy.logwarn(f"[NonObstacleTransition→SMART_STATIC] ✓ All conditions met!")
        return StateType.SMART_STATIC, StateType.SMART_STATIC
    # ===== HJ MODIFIED END =====

    # No Smart Static or not close - use GB (only if close_to_raceline AND not smart_active)
    if not smart_active and close_to_raceline:
        rospy.logwarn_throttle(1.0, f"[NonObstacleTransition→GB_TRACK] Using GB track")
        return StateType.GB_TRACK, StateType.GB_TRACK

    if state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts):
        if state_machine._check_on_spline(state_machine.cur_recovery_wpnts):
            rospy.loginfo_throttle(1.0, f"[NonObstacleTransition→RECOVERY] Using recovery")
            return StateType.RECOVERY, StateType.RECOVERY

    rospy.loginfo_throttle(1.0, f"[NonObstacleTransition→LOSTLINE] Lost line")
    return StateType.LOSTLINE, StateType.GB_TRACK
    
def ObstacleTransition(state_machine: StateMachine, close_to_raceline=False, smart_active=False) -> Tuple[StateType, StateType]:
    """Handle obstacles present case

    Args:
        close_to_raceline: True if close to Smart path (when smart_active=True) or GB raceline (when smart_active=False)
        smart_active: True if Smart Static mode is active
    """
    rospy.logwarn(f">>> ObstacleTransition: close_to_raceline={close_to_raceline}, smart_active={smart_active}, num_obs={len(state_machine.cur_obstacles_in_interest)}")
    # ===== HJ MODIFIED: Use smart_active parameter, check wpnts validity and path free =====
    wpnts_valid = state_machine._check_latest_wpnts(
        state_machine.smart_static_wpnts,
        state_machine.cur_smart_static_avoidance_wpnts)
    smart_path_free = state_machine._check_free_frenet(state_machine.cur_smart_static_avoidance_wpnts)
    gb_path_free = state_machine._check_free_frenet(state_machine.cur_gb_wpnts)

    rospy.loginfo_throttle(1.0,
        f"[ObstacleTransition] Check:\n"
        f"  smart_active (param): {smart_active}\n"
        f"  wpnts_valid: {wpnts_valid}\n"
        f"  close_to_raceline: {close_to_raceline}\n"
        f"  smart_path_free: {smart_path_free}\n"
        f"  gb_path_free: {gb_path_free}")

    # Check if Smart Static path is available, close, and free
    if smart_active and wpnts_valid and close_to_raceline and smart_path_free:
        # Smart Static path available, close, and free - use it
        rospy.logwarn(f"[ObstacleTransition→SMART_STATIC] ✓ All conditions met!")
        return StateType.SMART_STATIC, StateType.SMART_STATIC
    # ===== HJ MODIFIED END =====

    # GB path check (only if not smart_active, close to GB raceline, and GB path free)
    if not smart_active and close_to_raceline and gb_path_free:
        rospy.loginfo_throttle(1.0, f"[ObstacleTransition→GB_TRACK] GB path free (smart_active=False)")
        return StateType.GB_TRACK, StateType.GB_TRACK

    # Check recovery availability (initialize here for use in TRAILING block later)
    recovery_availability = False
    if not close_to_raceline:
        recovery_availability = state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
        if (recovery_availability and state_machine._check_free_frenet(state_machine.cur_recovery_wpnts)):
            rospy.loginfo_throttle(1.0, f"[ObstacleTransition→RECOVERY] Recovery path available")
            return StateType.RECOVERY, StateType.RECOVERY

    if state_machine._check_overtaking_mode() or state_machine._check_static_overtaking_mode():
        rospy.loginfo_throttle(1.0, f"[ObstacleTransition→OVERTAKE] Overtaking mode")
        return StateType.OVERTAKE, StateType.OVERTAKE

    else:
        # ===== HJ MODIFIED: TRAILING state - trajectory selection based on smart_active =====
        if smart_active:
            # Smart active - prioritize Smart path
            if wpnts_valid and close_to_raceline:
                # Smart path valid and close - use it
                rospy.logwarn(f"[ObstacleTransition→TRAILING+SMART_STATIC] Smart valid & close")
                return StateType.TRAILING, StateType.SMART_STATIC
            elif wpnts_valid:
                # Smart path valid but not close - still use Smart (don't fallback to GB!)
                rospy.logwarn(f"[ObstacleTransition→TRAILING+SMART_STATIC] Smart valid but not close")
                return StateType.TRAILING, StateType.SMART_STATIC
            elif recovery_availability:
                # Smart waypoint invalid - fallback to recovery
                rospy.loginfo_throttle(1.0, f"[ObstacleTransition→TRAILING+RECOVERY] Smart invalid, using Recovery")
                return StateType.TRAILING, StateType.RECOVERY
            elif gb_path_free:
                # Recovery unavailable - use GB if free
                rospy.loginfo_throttle(1.0, f"[ObstacleTransition→TRAILING+GB] GB path free")
                return StateType.TRAILING, StateType.GB_TRACK
            else:
                # No Smart, no recovery - fallback to GB
                rospy.loginfo_throttle(1.0, f"[ObstacleTransition→TRAILING+GB] Smart invalid, no recovery, using GB")
                return StateType.TRAILING, StateType.GB_TRACK
        else:
            # Not smart_active - use GB/Recovery
            if close_to_raceline:
                # Close to GB raceline - use GB
                rospy.loginfo_throttle(1.0, f"[ObstacleTransition→TRAILING+GB] Close to GB")
                return StateType.TRAILING, StateType.GB_TRACK
            elif recovery_availability:
                # GB blocked - use recovery if available
                rospy.loginfo_throttle(1.0, f"[ObstacleTransition→TRAILING+RECOVERY] GB blocked, using Recovery")
                return StateType.TRAILING, StateType.RECOVERY
            elif gb_path_free:
                # Not close but GB path free - use GB (safer)
                rospy.loginfo_throttle(1.0, f"[ObstacleTransition→TRAILING+GB] GB path free")
                return StateType.TRAILING, StateType.GB_TRACK
            else:
                # Default fallback
                rospy.loginfo_throttle(1.0, f"[ObstacleTransition→TRAILING+GB] Default")
                return StateType.TRAILING, StateType.GB_TRACK
        # ===== HJ MODIFIED END =====
        