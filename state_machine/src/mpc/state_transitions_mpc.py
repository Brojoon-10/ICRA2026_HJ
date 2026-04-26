from __future__ import annotations

# ### HJ : Phase X — file lives in state_machine/src/mpc/; states_types is in
# the parent state_machine/src/. Prepend parent so `from states_types import`
# resolves without requiring catkin_python_setup() / scripts-dir install.
import os as _os
import sys as _sys
_sys.path.insert(
    0, _os.path.abspath(_os.path.join(_os.path.dirname(__file__), _os.pardir)))

from typing import TYPE_CHECKING, Tuple, List

from states_types import StateType
import rospy
from f110_msgs.msg import Wpnt
import states_mpc as states


if TYPE_CHECKING:
    from state_machine_node import StateMachine

close_threshold_smart = 0.5
# close_threshold_gb replaced with state_machine._get_adaptive_close_threshold() (speed-adaptive)

# ===== HJ ADDED: Debug logging helper - only logs when values change =====
_debug_log_cache = {}
DEBUG_LOGGING_ENABLED = False  # Set to False to disable all debug logging

def debug_log_on_change(tag, **kwargs):
    """Log only when any of the kwargs values change

    Can be globally enabled/disabled via DEBUG_LOGGING_ENABLED flag.

    Usage:
        debug_log_on_change("MyTag", value1=x, value2=y, value3=z)
    """
    global _debug_log_cache

    # Skip if debug logging is disabled
    if not DEBUG_LOGGING_ENABLED:
        return

    # Create cache key
    cache_key = tag

    # Get previous values
    prev_values = _debug_log_cache.get(cache_key, None)

    # Check if any value changed
    if prev_values != kwargs:
        # Build log message
        msg_parts = [f"{k}={v}" for k, v in kwargs.items()]
        rospy.logwarn(f"[DEBUG {tag}] " + ", ".join(msg_parts))

        # Update cache
        _debug_log_cache[cache_key] = kwargs.copy()
# ===== HJ ADDED END =====

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

# ===== HJ MODIFIED: Complete mode separation - each mode has its own closed loop =====
def GlobalTrackingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.GB_TRACK`

    Routes to completely separate Smart or GB mode transitions.
    """
    smart_active = state_machine.smart_static_active

    # Complete mode switching - call separate function sets
    if smart_active:
        smart_helper = state_machine.smart_helper
        close_to_smart = smart_helper._check_close_to_raceline(close_threshold_smart) * smart_helper._check_close_to_raceline_heading(20)
        num_obs = len(smart_helper.cur_obstacles_in_interest)

        debug_log_on_change("GlobalTracking_SMART",
                           close=close_to_smart,
                           num_obs=num_obs,
                           cur_s=round(smart_helper.cur_s, 2),
                           cur_d=round(smart_helper.cur_d, 3))

        if num_obs == 0:
            return NonObstacleTransition_SmartMode(state_machine, close_to_smart)
        else:
            return ObstacleTransition_SmartMode(state_machine, close_to_smart)
    else:
        close_to_gb = state_machine._check_close_to_raceline(state_machine._get_adaptive_close_threshold()) * state_machine._check_close_to_raceline_heading(20)
        # rospy.logwarn(f"[GlobalTracking] GB MODE: close_to_gb={close_to_gb}, num_obs={len(state_machine.cur_obstacles_in_interest)}")

        if len(state_machine.cur_obstacles_in_interest) == 0:
            return NonObstacleTransition_GBMode(state_machine, close_to_gb)
        else:
            return ObstacleTransition_GBMode(state_machine, close_to_gb)


def RecoveryTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.RECOVERY`

    Recovery operates within the mode's closed loop.
    """
    smart_active = state_machine.smart_static_active

    # ### HJ : 2026-04-24 — user directive: "tracking 정보 있을 때 recovery
    # 뜰 일 절대 없어야 함". If SM has any obstacle of interest this tick,
    # RECOVERY semantics don't apply — we must route through GlobalTracking
    # / Obstacle branches to pick OVERTAKE or TRAILING (obstacle-aware).
    # Smart mode has its own closed loop; skip this early exit there.
    if not smart_active and len(getattr(state_machine, 'cur_obstacles_in_interest', [])) > 0:
        state_machine._rc_exit_streak = 0
        return GlobalTrackingTransition(state_machine)

    # ### HJ : 2026-04-24 — force-refresh cur_recovery_wpnts from the latest
    # MPC publish every tick we're in RECOVERY. _check_sustainability alone
    # only calls initialize_traj when killing_timer_sec (~1 s) has elapsed,
    # which means the first (possibly wall-punching) trajectory MPC emits on
    # spawn gets latched and followed for up to 1 second. Forcing
    # _check_latest_wpnts here restores "MPC publishes every tick, SM
    # consumes every tick" semantics. Cheap: the check also happens inside
    # the sustainability path, so this just primes the cache.
    try:
        if smart_active and hasattr(state_machine, 'smart_helper'):
            state_machine.smart_helper._check_latest_wpnts(
                state_machine.recovery_wpnts,
                state_machine.cur_recovery_wpnts)
        else:
            state_machine._check_latest_wpnts(
                state_machine.recovery_wpnts,
                state_machine.cur_recovery_wpnts)
    except Exception:
        pass

    # Use appropriate Frenet coordinate system based on mode
    # In Smart mode, recovery waypoints are also Fixed Frenet based
    if smart_active:
        smart_helper = state_machine.smart_helper
        recovery_sustainability = smart_helper._check_sustainability(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
        close_to_smart = smart_helper._check_close_to_raceline(close_threshold_smart) * smart_helper._check_close_to_raceline_heading(20)

        debug_log_on_change("Recovery_SMART",
                           sustainable=recovery_sustainability,
                           close=close_to_smart,
                           continuing=recovery_sustainability and not close_to_smart)

        if recovery_sustainability and not close_to_smart:
            return StateType.RECOVERY, StateType.RECOVERY
        # Recovery ended - return to Smart mode closed loop
        return SmartStaticTransition(state_machine)
    else:
        recovery_sustainability = state_machine._check_sustainability(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)

        # ### HJ : 2026-04-24 — asymmetric exit hysteresis.
        # Entry (GB → RECOVERY) is triggered on `|ego_n| > n_recovery_trigger`
        # (~0.15 m) which is intentionally easy — we want to start the
        # convergent solve quickly. Exit (RECOVERY → GB) was previously
        # just "close_to_raceline single-tick" → snapped the car onto GB
        # the moment |ego_n| dropped below ~0.5 m, steer twitched.
        #
        # New exit rule: require TIGHT proximity (|n| < 0.05 m, heading
        # error < 10°) AND K ticks continuous before handing to GB. This
        # gives the NO_OBS MPC a few cycles of "already on raceline"
        # convergence so the SM switch happens when MPC path and GB path
        # are already visually identical → no controller L1 jump.
        tight_threshold = getattr(state_machine, '_rc_exit_tight_m', 0.10)
        tight_heading_deg = getattr(state_machine, '_rc_exit_tight_deg', 10)
        exit_hysteresis_K = getattr(state_machine, '_rc_exit_hyst_ticks', 8)

        close_tight = (state_machine._check_close_to_raceline(tight_threshold)
                       * state_machine._check_close_to_raceline_heading(tight_heading_deg))
        if close_tight:
            state_machine._rc_exit_streak = getattr(state_machine, '_rc_exit_streak', 0) + 1
        else:
            state_machine._rc_exit_streak = 0

        debug_log_on_change("Recovery_exit",
                            sustain=recovery_sustainability,
                            close_tight=int(close_tight),
                            streak=state_machine._rc_exit_streak,
                            K=exit_hysteresis_K)

        if (recovery_sustainability
                and state_machine._rc_exit_streak < exit_hysteresis_K):
            return StateType.RECOVERY, StateType.RECOVERY
        # Recovery ended — stable-on-GB streak met OR path sustainability lost.
        state_machine._rc_exit_streak = 0
        return GlobalTrackingTransition(state_machine)


def TrailingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.TRAILING`"""
    smart_active = state_machine.smart_static_active

    if smart_active:
        smart_helper = state_machine.smart_helper
        close_to_smart = smart_helper._check_close_to_raceline(close_threshold_smart) * smart_helper._check_close_to_raceline_heading(20)
        num_obs = len(smart_helper.cur_obstacles_in_interest)
        ftg_check = state_machine._check_ftg()

        debug_log_on_change("Trailing_SMART",
                           close=close_to_smart,
                           num_obs=num_obs,
                           ftg=ftg_check)

        if num_obs == 0:
            return NonObstacleTransition_SmartMode(state_machine, close_to_smart)
        else:
            if ftg_check:
                return StateType.FTGONLY, StateType.FTGONLY
            return ObstacleTransition_SmartMode(state_machine, close_to_smart)
    else:
        close_to_gb = state_machine._check_close_to_raceline(state_machine._get_adaptive_close_threshold()) * state_machine._check_close_to_raceline_heading(20)
        # rospy.logwarn(f"[Trailing] GB MODE: close_to_gb={close_to_gb}")

        if len(state_machine.cur_obstacles_in_interest) == 0:
            return NonObstacleTransition_GBMode(state_machine, close_to_gb)
        else:
            if state_machine._check_ftg():
                return StateType.FTGONLY, StateType.FTGONLY
            return ObstacleTransition_GBMode(state_machine, close_to_gb)


def OvertakingTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.OVERTAKE`"""
    smart_active = state_machine.smart_static_active

    # ### HJ : 2026-04-24 — force-refresh cur_avoidance_wpnts from the latest
    # MPC publish every tick we're in OVERTAKE. Same rationale as
    # RecoveryTransition: without this, the killing_timer snapshot sticks
    # and the car follows a stale MPC path while fresher ones arrive.
    try:
        if smart_active and hasattr(state_machine, 'smart_helper'):
            state_machine.smart_helper._check_latest_wpnts(
                state_machine.avoidance_wpnts,
                state_machine.cur_avoidance_wpnts)
        else:
            state_machine._check_latest_wpnts(
                state_machine.avoidance_wpnts,
                state_machine.cur_avoidance_wpnts)
    except Exception:
        pass

    # Use appropriate Frenet coordinate system based on mode
    # In Smart mode, overtaking waypoints are also Fixed Frenet based
    if smart_active:
        smart_helper = state_machine.smart_helper
        ot_sustainability = smart_helper._check_overtaking_mode_sustainability()
        enemy_in_front = smart_helper._check_enemy_in_front()
    else:
        ot_sustainability = state_machine._check_overtaking_mode_sustainability()
        enemy_in_front = state_machine._check_enemy_in_front()

    debug_log_on_change("Overtaking",
                       mode="SMART" if smart_active else "GB",
                       sustainable=ot_sustainability,
                       enemy=enemy_in_front,
                       ttl_count=state_machine.overtaking_ttl_count,
                       continuing=ot_sustainability and (enemy_in_front or state_machine.overtaking_ttl_count < state_machine.overtaking_ttl_count_threshold))

    if ot_sustainability and enemy_in_front:
        state_machine.overtaking_ttl_count = 0
        return StateType.OVERTAKE, StateType.OVERTAKE
    if ot_sustainability and state_machine.overtaking_ttl_count < state_machine.overtaking_ttl_count_threshold:
        state_machine.overtaking_ttl_count += 1
        return StateType.OVERTAKE, StateType.OVERTAKE
    state_machine.overtaking_ttl_count = 0

    # ### HJ : Phase X — OT exit routed through RECOVERY when ego is still
    # off the GB line AND a valid recovery path is fresh. This pairs with the
    # MPC internal FSM's TRANSITION_OT2RC which streams a continuous trajectory
    # to /planner/avoidance/otwpnts first, then hands off to
    # /planner/recovery/wpnts; the SM state now rides that RECOVERY channel
    # instead of snapping GB on the same tick. Smart-mode exits unchanged
    # (Smart already has its own closed loop via fixed-frenet).
    if not smart_active:
        try:
            close_to_gb = state_machine._check_close_to_raceline(close_threshold_smart)
        except Exception:
            close_to_gb = True
        try:
            rec_fresh = state_machine._check_latest_wpnts(
                state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
            rec_clear = rec_fresh and state_machine._check_free_frenet(
                state_machine.cur_recovery_wpnts)
        except Exception:
            rec_clear = False
        if rec_clear and not close_to_gb:
            return StateType.RECOVERY, StateType.RECOVERY

    # Overtaking ended - return to appropriate mode's closed loop
    if smart_active:
        return SmartStaticTransition(state_machine)
    else:
        return GlobalTrackingTransition(state_machine)


def StartTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.START`"""
    start_free = state_machine._check_free_cartesian(state_machine.cur_start_wpnts)
    on_spline = state_machine._check_on_spline(state_machine.cur_start_wpnts)

    if start_free and on_spline:
        return StateType.START, StateType.START
    else:
        state_machine.cur_start_wpnts.is_init = False
        return GlobalTrackingTransition(state_machine)


def FTGOnlyTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.FTGONLY`"""

    smart_active = state_machine.smart_static_active

    # Use appropriate Frenet coordinate system based on mode
    if smart_active:
        smart_helper = state_machine.smart_helper
        close_to_raceline = smart_helper._check_close_to_raceline(close_threshold_smart) * smart_helper._check_close_to_raceline_heading(20)

        if len(smart_helper.cur_obstacles_in_interest) == 0:
            return NonObstacleTransition_SmartMode(state_machine, close_to_raceline)
        else:
            if close_to_raceline and smart_helper._check_free_frenet(state_machine.cur_smart_static_avoidance_wpnts):
                return StateType.SMART_STATIC, StateType.SMART_STATIC

            recovery_availability = smart_helper._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
            if (recovery_availability and smart_helper._check_free_frenet(state_machine.cur_recovery_wpnts)):
                return StateType.RECOVERY, StateType.RECOVERY

            # ===== HJ MODIFIED: Disable dynamic overtaking in Smart mode =====
            # In Smart mode: only static overtaking allowed (dynamic overtaking disabled)
            if smart_helper._check_static_overtaking_mode():
                return StateType.OVERTAKE, StateType.OVERTAKE
            # Note: _check_overtaking_mode() (dynamic) is intentionally disabled in Smart mode
            # ===== HJ MODIFIED END =====
            else:
                return StateType.FTGONLY, StateType.FTGONLY
    else:
        close_to_raceline = state_machine._check_close_to_raceline(close_threshold_smart) * state_machine._check_close_to_raceline_heading(20)

        if len(state_machine.cur_obstacles_in_interest) == 0:
            return NonObstacleTransition_GBMode(state_machine, close_to_raceline)
        else:
            if close_to_raceline and state_machine._check_free_frenet(state_machine.cur_gb_wpnts):
                return StateType.GB_TRACK, StateType.GB_TRACK

            recovery_availability = state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
            if (recovery_availability and state_machine._check_free_frenet(state_machine.cur_recovery_wpnts)):
                return StateType.RECOVERY, StateType.RECOVERY

            # ===== HJ MODIFIED: Disable dynamic overtaking when smart_static_active=true =====
            if state_machine._check_static_overtaking_mode():
                return StateType.OVERTAKE, StateType.OVERTAKE
            # Dynamic overtaking only when smart_static is NOT active
            if state_machine._check_overtaking_mode() and not state_machine.smart_static_active:
                return StateType.OVERTAKE, StateType.OVERTAKE
            # ===== HJ MODIFIED END =====
            else:
                return StateType.FTGONLY, StateType.FTGONLY


def SmartStaticTransition(state_machine: StateMachine) -> Tuple[StateType, StateType]:
    """Transitions for being in `StateType.SMART_STATIC`

    Entry point for Smart mode's closed loop.
    When flag is active: uses Smart Static path only (GB raceline ignored).
    When flag turns off: returns to GB mode transitions.
    """
    # ===== HJ ADDED: Handle smart_static_active flag off - return to GB mode =====
    if not state_machine.smart_static_active:
        # Flag turned off - need to safely return to GB mode
        rospy.logwarn("[SmartStaticTransition] Flag off, returning to GB mode")

        # Clear all closest_targets to prevent Frenet coordinate mismatch
        # (smart_helper may have set these with Fixed Frenet coordinates)
        state_machine.cur_gb_wpnts.closest_target = None
        state_machine.cur_recovery_wpnts.closest_target = None
        state_machine.cur_avoidance_wpnts.closest_target = None
        state_machine.cur_static_avoidance_wpnts.closest_target = None

        # Check if we're close enough to GB raceline for direct transition
        close_to_gb = state_machine._check_close_to_raceline(state_machine._get_adaptive_close_threshold()) * \
                      state_machine._check_close_to_raceline_heading(20)

        if close_to_gb:
            # Close enough to GB - safe to return immediately
            rospy.logwarn("[SmartStaticTransition→GB_TRACK] Close to GB, direct return")
            return StateType.GB_TRACK, StateType.GB_TRACK
        else:
            # Not close to GB - delegate to GB mode transitions
            # GB transitions will handle recovery/overtaking/trailing logic
            rospy.logwarn("[SmartStaticTransition] Not close to GB, using GB transitions")
            if len(state_machine.cur_obstacles_in_interest) == 0:
                return NonObstacleTransition_GBMode(state_machine, close_to_gb)
            else:
                return ObstacleTransition_GBMode(state_machine, close_to_gb)
    # ===== HJ ADDED END =====

    # Original Smart mode logic (flag still active)
    smart_helper = state_machine.smart_helper
    close_to_smart = smart_helper._check_close_to_raceline(close_threshold_smart) * smart_helper._check_close_to_raceline_heading(20)
    num_obstacles = len(smart_helper.cur_obstacles_in_interest)

    debug_log_on_change("SmartStatic",
                       close=close_to_smart,
                       num_obs=num_obstacles)

    # Delegate to Smart mode transitions only
    if num_obstacles == 0:
        return NonObstacleTransition_SmartMode(state_machine, close_to_smart)
    else:
        return ObstacleTransition_SmartMode(state_machine, close_to_smart)


##################################################################################################################
##################################################################################################################
# ===== SMART MODE CLOSED LOOP - Only considers Smart Static path =====

def NonObstacleTransition_SmartMode(state_machine: StateMachine, close_to_smart: bool) -> Tuple[StateType, StateType]:
    """Handle no obstacles case in Smart Static mode

    CLOSED LOOP: Only considers Smart Static path.
    GB raceline is completely ignored.

    Args:
        close_to_smart: True if close to Smart Static path (already calculated with Fixed Frenet)
    """
    smart_helper = state_machine.smart_helper

    wpnts_valid = smart_helper._check_latest_wpnts(
        state_machine.smart_static_wpnts,
        state_machine.cur_smart_static_avoidance_wpnts)

    debug_log_on_change("NonObstacle_SMART",
                       close=close_to_smart,
                       wpnts_valid=wpnts_valid,
                       num_wpnts=len(state_machine.cur_smart_static_avoidance_wpnts.list))

    # Priority 1: Smart path available and close - use it
    if wpnts_valid and close_to_smart:
        # rospy.logwarn(f"[NonObstacle_Smart→SMART_STATIC] ✓ Valid & close")
        return StateType.SMART_STATIC, StateType.SMART_STATIC

    # Priority 2: Smart path valid but not close - stay in Smart, use recovery to return
    if wpnts_valid:
        # rospy.logwarn(f"[NonObstacle_Smart→SMART_STATIC] ✓ Valid (not close)")
        return StateType.SMART_STATIC, StateType.SMART_STATIC

    # Priority 3: Smart path invalid - use recovery to get back
    if smart_helper._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts):
        if smart_helper._check_on_spline(state_machine.cur_recovery_wpnts):
            # rospy.logwarn(f"[NonObstacle_Smart→RECOVERY] Smart invalid, recovering")
            return StateType.RECOVERY, StateType.RECOVERY

    # Priority 4: No valid path - lost line (still return SMART_STATIC trajectory to stay in loop)
    # rospy.logwarn(f"[NonObstacle_Smart→LOSTLINE] Lost line")
    return StateType.LOSTLINE, StateType.SMART_STATIC


def ObstacleTransition_SmartMode(state_machine: StateMachine, close_to_smart: bool) -> Tuple[StateType, StateType]:
    """Handle obstacles present case in Smart Static mode

    CLOSED LOOP: Only considers Smart Static path with Fixed Frenet.
    GB raceline and GB path free status are completely ignored.

    Args:
        close_to_smart: True if close to Smart Static path (already calculated with Fixed Frenet)
    """
    smart_helper = state_machine.smart_helper

    wpnts_valid = smart_helper._check_latest_wpnts(
        state_machine.smart_static_wpnts,
        state_machine.cur_smart_static_avoidance_wpnts)
    smart_path_free = smart_helper._check_free_frenet(state_machine.cur_smart_static_avoidance_wpnts)

    # Check overtaking conditions
    ot_mode = smart_helper._check_overtaking_mode()
    static_ot_mode = smart_helper._check_static_overtaking_mode()

    debug_log_on_change("Obstacle_SMART",
                       close=close_to_smart,
                       wpnts_valid=wpnts_valid,
                       path_free=smart_path_free,
                       ot_mode=ot_mode,
                       static_ot=static_ot_mode,
                       num_obs=len(smart_helper.cur_obstacles_in_interest))

    # ===== HJ ADDED: Periodic debug logging when blocked =====
    if not smart_path_free and not ot_mode and not static_ot_mode:
        rospy.logwarn_throttle(2.0,
            f"[DEBUG Obstacle_SMART BLOCKED] Path blocked but no overtaking! "
            f"close={close_to_smart}, wpnts_valid={wpnts_valid}, "
            f"static_ot={static_ot_mode}, ot={ot_mode}, num_obs={len(smart_helper.cur_obstacles_in_interest)}")
    # ===== HJ ADDED END =====

    # Priority 1: Smart path available, close, and free - use it
    if wpnts_valid and close_to_smart and smart_path_free:
        return StateType.SMART_STATIC, StateType.SMART_STATIC

    # Priority 2: Check recovery availability (only if not close to Smart path)
    recovery_availability = False
    if not close_to_smart:
        recovery_availability = smart_helper._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts)
        if (recovery_availability and smart_helper._check_free_frenet(state_machine.cur_recovery_wpnts)):
            return StateType.RECOVERY, StateType.RECOVERY

    # Priority 3: Overtaking check (use smart_helper for Fixed Frenet based checks)
    # ===== HJ MODIFIED: Disable dynamic overtaking in Smart mode =====
    # In Smart mode: only static overtaking allowed (dynamic overtaking disabled)
    if static_ot_mode:
        return StateType.OVERTAKE, StateType.OVERTAKE
    # Note: ot_mode (dynamic overtaking) is intentionally disabled in Smart mode
    # ===== HJ MODIFIED END =====

    # Priority 4: TRAILING state - Smart mode always uses Smart path
    if wpnts_valid and close_to_smart:
        # rospy.logwarn(f"[Obstacle_Smart→TRAILING+SMART] Valid & close")
        return StateType.TRAILING, StateType.SMART_STATIC
    elif wpnts_valid:
        # Smart path valid but not close - STILL use Smart (don't fallback!)
        # rospy.logwarn(f"[Obstacle_Smart→TRAILING+SMART] Valid (not close) - staying in Smart")
        return StateType.TRAILING, StateType.SMART_STATIC
    elif recovery_availability:
        # rospy.logwarn(f"[Obstacle_Smart→TRAILING+RECOVERY] Smart invalid, using recovery")
        return StateType.TRAILING, StateType.RECOVERY
    else:
        # Last resort - Smart invalid, no recovery, still stay in Smart mode
        # rospy.logwarn(f"[Obstacle_Smart→TRAILING+SMART] Fallback to Smart (no alternatives)")
        return StateType.TRAILING, StateType.SMART_STATIC


##################################################################################################################
# ===== GB MODE CLOSED LOOP - Only considers GB raceline =====

def NonObstacleTransition_GBMode(state_machine: StateMachine, close_to_gb: bool) -> Tuple[StateType, StateType]:
    """Handle no obstacles case in GB tracking mode

    CLOSED LOOP: Only considers GB raceline.
    Smart Static path is completely ignored.

    Args:
        close_to_gb: True if close to GB raceline
    """
    # rospy.logwarn(f">>> NonObstacleTransition_GBMode: close_to_gb={close_to_gb}")

    # Priority 1: Close to GB raceline - use it
    if close_to_gb:
        # rospy.logwarn(f"[NonObstacle_GB→GB_TRACK] ✓ Close to GB")
        return StateType.GB_TRACK, StateType.GB_TRACK

    # Priority 2: Not close to GB - use recovery to get back
    if state_machine._check_latest_wpnts(state_machine.recovery_wpnts, state_machine.cur_recovery_wpnts):
        if state_machine._check_on_spline(state_machine.cur_recovery_wpnts):
            # rospy.logwarn(f"[NonObstacle_GB→RECOVERY] Not close, recovering")
            return StateType.RECOVERY, StateType.RECOVERY

    # Priority 3: No valid path - lost line
    # rospy.logwarn(f"[NonObstacle_GB→LOSTLINE] Lost line")
    return StateType.LOSTLINE, StateType.GB_TRACK


def ObstacleTransition_GBMode(state_machine: StateMachine, close_to_gb: bool) -> Tuple[StateType, StateType]:
    """Handle obstacles present case in GB tracking mode

    CLOSED LOOP: Only considers GB raceline and GB path.
    Smart Static path is completely ignored.

    Args:
        close_to_gb: True if close to GB raceline
    """
    # rospy.logwarn(f">>> ObstacleTransition_GBMode: close_to_gb={close_to_gb}, num_obs={len(state_machine.cur_obstacles_in_interest)}")

    gb_path_free = state_machine._check_free_frenet(state_machine.cur_gb_wpnts)

    # Priority 1: GB path close and free → use GB directly (no obstacle in way).
    if close_to_gb and gb_path_free:
        return StateType.GB_TRACK, StateType.GB_TRACK

    # Priority 2: Overtaking check (static > dynamic).
    if state_machine._check_static_overtaking_mode():
        return StateType.OVERTAKE, StateType.OVERTAKE
    if state_machine._check_overtaking_mode() and not state_machine.smart_static_active:
        return StateType.OVERTAKE, StateType.OVERTAKE

    # Priority 3: TRAILING state — path source priority.
    # ### HJ : 2026-04-24 (bag diagnosis): `_check_free_frenet` uses a
    # `lateral_width_m` threshold that disagrees with the MPC solver's own
    # obstacle clearance (SideDecider.gap_lat=0.25 + ego_half=0.15 + soft
    # bubble). Borderline obstacle clearances (pass margin ~0.3 m) toggle
    # SM between "free" and "not free" per tick → TRAILING flips between
    # MPC_OT (free) and GB (not free) — user observation: "trailing
    # 될때 가끔 gb 경로 나오면서 trailing".
    # Phase X philosophy: MPC is the path authority. Solver's corridor +
    # obstacle cost already enforces clearance; SM's independent check is
    # redundant and creates artifacts when thresholds disagree. So for
    # MPC-origin paths we ONLY check freshness; we trust the solver.
    # Legacy / non-MPC paths (from_mpc==False) keep the full free check.
    avoidance_fresh = state_machine._check_latest_wpnts(
        state_machine.avoidance_wpnts, state_machine.cur_avoidance_wpnts)
    if avoidance_fresh:
        from_mpc = getattr(state_machine.cur_avoidance_wpnts, 'from_mpc', False)
        if from_mpc:
            # Trust MPC solver's own collision awareness.
            return StateType.TRAILING, StateType.OVERTAKE
        # Non-MPC source — keep legacy free check.
        if state_machine._check_free_frenet(state_machine.cur_avoidance_wpnts):
            return StateType.TRAILING, StateType.OVERTAKE
    # MPC stale or legacy path blocked. Fall back to GB.
    return StateType.TRAILING, StateType.GB_TRACK
