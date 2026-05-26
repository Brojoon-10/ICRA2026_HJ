#!/usr/bin/env python3

import csv
import datetime as _dt
import json
import math
import os
import threading
import rospkg
import rospy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64, String
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from copy import deepcopy
### HJ : dynamic_reconfigure for live launch tuning via rqt_reconfigure
from dynamic_reconfigure.server import Server as DynRecServer
from stack_master.cfg import SimpleMuxConfig

### HJ : per-session log root. In docker container, $HOME/catkin_ws/src/race_stack maps to host's HJ_docs.
_REC_LOG_ROOT = os.path.expanduser("~/catkin_ws/src/race_stack/HJ_docs/debug/launch_logs")
_REC_PLOT_HORIZON_SEC = 2.5  # window length rendered in PNG. CSV is full session.
_REC_NAN = float('nan')
class SimpleMuxNode:

    def __init__(self):
        """
        Initialize the node, subscribe to topics, create publishers and set up member variables.
        """

        # Initialize the node
        self.name = "simple_mux"
        rospy.init_node(self.name, anonymous=True)

        self.out_topic  = rospy.get_param("/vesc/out_topic", "low_level/ackermann_cmd_mux/output")
        self.in_topic  = rospy.get_param("/vesc/in_topic", "high_level/ackermann_cmd_mux/input/nav_1")
        self.joy_topic  = rospy.get_param("/vesc/joy_topic", "/joy")
        self.rate_hz = rospy.get_param("/vesc/rate_hz", 50.0)
        self.max_speed = rospy.get_param("/vesc/joy_max_speed", 4.0)
        self.max_steer = rospy.get_param("/vesc/joy_max_steer", 0.4)
        self.joy_freshness_threshold = rospy.get_param("/vesc/joy_freshness_threshold", 1.0)


        vesc_servo_min = rospy.get_param("/vesc/vesc_driver/servo_min", 1.0)
        vesc_servo_max = rospy.get_param("/vesc/vesc_driver/servo_max", 1.0)
        steering_angle_to_servo_offset = rospy.get_param("/vesc/steering_angle_to_servo_offset", 1.0)
        steering_angle_to_servo_gain = rospy.get_param("/vesc/steering_angle_to_servo_gain", 1.0)

        servo_max_rad = (vesc_servo_max - steering_angle_to_servo_offset) / steering_angle_to_servo_gain
        servo_min_rad = (vesc_servo_min - steering_angle_to_servo_offset) / steering_angle_to_servo_gain

        self.servo_max_abs = min(abs(servo_max_rad), abs(servo_min_rad))

        self.current_host = None
        self.release_time = None  ### HJ : timestamp when buttons released
        self.autodrive_latched = False  ### HJ : RB edge-triggered latch for autodrive
        self.rb_prev = 0  ### HJ : previous RB state for edge detection

        self.human_drive = None
        self.autodrive = None
        self.zero_msg = AckermannDriveStamped()
        self.zero_msg.header.stamp = rospy.Time.now()
        self.zero_msg.drive.steering_angle = 0
        self.zero_msg.drive.speed = 0
        self.cur_v = 0
        self.prev_del_v = 0
        self.vel_planner = 0

        ### HJ : launch control parameters (overridable via rosparam, live-tunable via dyn_reconfigure)
        # Two profile modes share the same arm/fire/abort logic:
        #   CONSTANT (mode=0): flat target_I = launch_current_A  (v1 single-knob)
        #   CURVE    (mode=1): 4-phase exponential profile (v2, see _launch_i_cmd)
        self.launch_arm_button = int(rospy.get_param("~launch_arm_button", 0))   # A button on most pads
        self.launch_t_total = float(rospy.get_param("~launch_t_total", 1.0))     # launch active duration [s]
        self.launch_current_A = float(rospy.get_param("~launch_current_A", 15.0))  # [CONSTANT mode] flat current
        self.launch_safety_floor_margin = float(rospy.get_param("~launch_safety_floor_margin", 0.1))  # m/s; speed >= measured + this after launch hand-off
        ### HJ : v2 curve-shape params (CURVE mode). Defaults follow exp-rise design (weight transfer tau).
        self.launch_profile_mode = int(rospy.get_param("~launch_profile_mode", 1))   # 0=CONSTANT, 1=CURVE
        self.launch_I_back = float(rospy.get_param("~launch_I_back", 30.0))         # phase 1 end [A]
        self.launch_I_peak = float(rospy.get_param("~launch_I_peak", 75.0))         # phase 2/3 peak [A]
        self.launch_t_back = float(rospy.get_param("~launch_t_back", 0.05))         # phase 1 end [s]
        self.launch_t_rise_end = float(rospy.get_param("~launch_t_rise_end", 0.30)) # phase 2 end [s]
        self.launch_tau = float(rospy.get_param("~launch_tau", 0.08))               # exp rise time const [s]
        # cpp jerk==512 path constants (custom_ackermann_to_vesc.cpp:130-131). Mirror them here for inverse mapping.
        self.a2c_gain = float(rospy.get_param("/vesc/acceleration_to_current_gain", 10.0))
        self.a2c_baseline = 10.0
        self.v_ff_coeff = 0.9
        self.v_ff_deadzone = 0.3

        ### HJ : launch state
        self.launch_armed = False
        self.launch_active = False
        self.launch_t0 = None
        self.launch_post_done = False  # one-shot: first tick after DONE applies safety floor
        self.a_prev = 0  # arm-button edge tracking
        self.lb_prev = 0  # LB edge tracking (for explicit launch abort logging)

        ### HJ : yaml path for save/load triggers. Broad except so init never dies
        # because of yaml path quirks — save/load just disables itself.
        self.launch_yaml_path = None
        try:
            pkg_path = rospkg.RosPack().get_path("stack_master")
            car_name = rospy.get_param('/racecar_version', os.environ.get('CAR_NAME', ''))
            if car_name:
                self.launch_yaml_path = os.path.join(
                    pkg_path, 'config', car_name, 'devices', 'launch_control.yaml')
        except Exception as e:
            rospy.logwarn("[simple_mux] launch_yaml_path resolve failed: %s", e)
        if self.launch_yaml_path is None:
            rospy.logwarn("[simple_mux] launch_yaml_path unresolved — save/load disabled. "
                          "Set /racecar_version or CAR_NAME and respawn.")
        else:
            rospy.loginfo("[simple_mux] launch_yaml_path = %s", self.launch_yaml_path)

        # Subscribe to the topics
        rospy.Subscriber(self.in_topic, AckermannDriveStamped, self.drive_callback)
        rospy.Subscriber("/vesc/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/vel_planner", Float64, self.planner_callback)

        # Do not use filtered velocity
        # rospy.Subscriber("/car_state/odom", Odometry, self.odom_callback)

        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)
        ### HJ : external abort trigger (e.g. test_control_publisher duration cutoff)
        # Same effect as LB rising: drop autodrive latch, reset launch state, force release_time so timer
        # broadcasts 1s of zero ackermann then idles.
        rospy.Subscriber("/launch_controller/abort", Empty, self._abort_cb, queue_size=1)

        ### HJ : optional state_machine subscription. simple_mux must run standalone
        # (without 3d_headtohead.launch), so the topic may never publish — in that
        # case state stays None and launch-override behaves as before. When
        # state_machine is up and emits TRAILING, the launch current-injection is
        # force-aborted so the controller's speed command reaches VESC.
        self.state_topic = rospy.get_param("~state_topic", "/state_machine")
        self.state_stale_sec = float(rospy.get_param("~state_stale_sec", 0.5))
        self._cur_state = None       # last string received, e.g. 'TRAILING'
        self._state_stamp = None     # rospy.Time of last reception; None == never received
        self._trailing_abort_logged = False  # log-once per trailing-abort event
        rospy.Subscriber(self.state_topic, String, self._state_cb, queue_size=1)

        self.drive_pub = rospy.Publisher(self.out_topic, AckermannDriveStamped, queue_size=10)
        self.current_pub = rospy.Publisher("/vesc/commands/motor/current", Float64, queue_size=10)
        self.launch_debug_pub = rospy.Publisher("/launch_controller/debug", String, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_callback)

        ### HJ : dynamic_reconfigure server for live launch tuning (rqt_reconfigure → /simple_mux)
        self.dyn_srv = DynRecServer(SimpleMuxConfig, self._dyn_cb)

        rospy.loginfo("[simple_mux] launch ctrl ready: mode=%d arm_btn=%d t_total=%.2fs I_const=%.1fA "
                      "I_back=%.1fA I_peak=%.1fA t_back=%.3f t_rise_end=%.3f tau=%.3f",
                      self.launch_profile_mode, self.launch_arm_button, self.launch_t_total,
                      self.launch_current_A, self.launch_I_back, self.launch_I_peak,
                      self.launch_t_back, self.launch_t_rise_end, self.launch_tau)

    ### HJ : dynamic_reconfigure callback. Refuses to update mid-launch to avoid mid-flight value jumps.
    def _dyn_cb(self, config, level):
        if self.launch_active:
            rospy.logwarn("[simple_mux] dynamic update ignored: launch active. Wait for DONE.")
            # Echo current values back so rqt shows actual state
            config.launch_profile_mode = self.launch_profile_mode
            config.launch_current_A = self.launch_current_A
            config.launch_I_back = self.launch_I_back
            config.launch_I_peak = self.launch_I_peak
            config.launch_t_back = self.launch_t_back
            config.launch_t_rise_end = self.launch_t_rise_end
            config.launch_tau = self.launch_tau
            config.launch_t_total = self.launch_t_total
            config.launch_safety_floor_margin = self.launch_safety_floor_margin
            config.save_params = False
            config.load_params = False
            return config

        ### HJ : save/load triggers — checkbox flips in rqt_reconfigure act as one-shot
        # buttons. Reset to False so the next tick clears the checkmark in rqt UI.
        if config.save_params:
            self._save_launch_yaml(config)
            config.save_params = False
        if config.load_params:
            config.load_params = False
            loaded = self._load_launch_yaml()
            if loaded is not None:
                config.update(loaded)

        self.launch_profile_mode = int(config.launch_profile_mode)
        self.launch_current_A = float(config.launch_current_A)
        self.launch_I_back = float(config.launch_I_back)
        self.launch_I_peak = float(config.launch_I_peak)
        self.launch_t_back = float(config.launch_t_back)
        self.launch_t_rise_end = float(config.launch_t_rise_end)
        self.launch_tau = float(config.launch_tau)
        self.launch_t_total = float(config.launch_t_total)
        self.launch_safety_floor_margin = float(config.launch_safety_floor_margin)
        # sanity: t_back <= t_rise_end <= t_total. clamp silently — slider order during drags causes
        # transient violations otherwise.
        if self.launch_t_back > self.launch_t_rise_end:
            self.launch_t_rise_end = self.launch_t_back
            config.launch_t_rise_end = self.launch_t_rise_end
        if self.launch_t_rise_end > self.launch_t_total:
            self.launch_t_total = self.launch_t_rise_end
            config.launch_t_total = self.launch_t_total
        rospy.loginfo("[simple_mux] dyn updated: mode=%d I_const=%.1fA I_back=%.1fA I_peak=%.1fA "
                      "t_back=%.3f t_rise_end=%.3f tau=%.3f t_total=%.2f floor=%.2f",
                      self.launch_profile_mode, self.launch_current_A, self.launch_I_back,
                      self.launch_I_peak, self.launch_t_back, self.launch_t_rise_end,
                      self.launch_tau, self.launch_t_total, self.launch_safety_floor_margin)
        return config

    ### HJ : keys we round-trip through yaml. Subset of SimpleMux.cfg (save/load triggers themselves excluded).
    _LAUNCH_YAML_KEYS = (
        'launch_profile_mode',
        'launch_current_A',
        'launch_I_back',
        'launch_I_peak',
        'launch_t_back',
        'launch_t_rise_end',
        'launch_tau',
        'launch_t_total',
        'launch_safety_floor_margin',
    )

    ### HJ : dump current dyn config to launch_control.yaml (same path launch reads).
    def _save_launch_yaml(self, config):
        if not self.launch_yaml_path:
            rospy.logerr("[simple_mux] save_params: yaml path unresolved, abort")
            return
        try:
            os.makedirs(os.path.dirname(self.launch_yaml_path), exist_ok=True)
            out = {}
            for k in self._LAUNCH_YAML_KEYS:
                v = getattr(config, k)
                # Coerce so PyYAML emits clean scalars (no numpy / dyn_reconfigure types).
                if isinstance(v, bool):
                    out[k] = bool(v)
                elif isinstance(v, int) and not isinstance(v, bool):
                    out[k] = int(v)
                else:
                    out[k] = float(v)
            with open(self.launch_yaml_path, 'w') as f:
                yaml.dump(out, f, default_flow_style=False, sort_keys=False)
            rospy.loginfo("[simple_mux] saved -> %s", self.launch_yaml_path)
        except (OSError, yaml.YAMLError) as e:
            rospy.logerr("[simple_mux] save failed: %s", e)

    ### HJ : read launch_control.yaml and return a dict of only the known launch keys.
    # Returns None on error (so caller leaves config untouched).
    def _load_launch_yaml(self):
        if not self.launch_yaml_path:
            rospy.logerr("[simple_mux] load_params: yaml path unresolved, abort")
            return None
        try:
            with open(self.launch_yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            rospy.logerr("[simple_mux] load failed: %s", e)
            return None
        if not isinstance(data, dict):
            rospy.logerr("[simple_mux] load failed: yaml root not a mapping")
            return None
        out = {k: data[k] for k in self._LAUNCH_YAML_KEYS if k in data}
        rospy.loginfo("[simple_mux] loaded %d params from %s", len(out), self.launch_yaml_path)
        return out

    ### HJ : i_cmd(t) — the launch profile itself.
    # CONSTANT mode = flat I_const (v1 behavior).
    # CURVE mode = 4-phase exp profile:
    #   P1 [0, t_back]            : linear 0 -> I_back   (backlash absorb)
    #   P2 [t_back, t_rise_end]   : I_back + (I_peak-I_back)*(1-exp(-(t-t_back)/tau))  (weight transfer)
    #   P3 [t_rise_end, t_total]  : I_peak                                              (traction plateau)
    # Phase id is returned alongside for debug/visualization.
    def _launch_i_cmd(self, t):
        if self.launch_profile_mode == 0:
            return (self.launch_current_A if 0.0 <= t < self.launch_t_total else 0.0), \
                   (1 if 0.0 <= t < self.launch_t_total else 0)
        # CURVE
        if t < 0.0:
            return 0.0, 0
        if t < self.launch_t_back:
            tb = max(self.launch_t_back, 1e-6)
            return self.launch_I_back * (t / tb), 1
        if t < self.launch_t_rise_end:
            tau = max(self.launch_tau, 1e-6)
            return self.launch_I_back + (self.launch_I_peak - self.launch_I_back) * \
                   (1.0 - math.exp(-(t - self.launch_t_back) / tau)), 2
        if t < self.launch_t_total:
            return self.launch_I_peak, 3
        return 0.0, 0

    def check_uptodate(self, drive_msg):
        # return True
        if drive_msg is None:
            return False

        if abs(drive_msg.header.stamp.to_sec() - rospy.Time.now().to_sec()) < self.joy_freshness_threshold:
            return True
        else:
            return False

    def clip_servo(self, in_drive_msg):
        drive_msg = AckermannDriveStamped()
        drive_msg = deepcopy(in_drive_msg)

        if drive_msg.drive.steering_angle > 0 and drive_msg.drive.steering_angle > self.servo_max_abs:
            drive_msg.drive.steering_angle = self.servo_max_abs
        elif drive_msg.drive.steering_angle < 0 and drive_msg.drive.steering_angle < - self.servo_max_abs:
            drive_msg.drive.steering_angle = -self.servo_max_abs

        return drive_msg

    ### HJ : invert cpp's i_cmd = a2c_gain*accel + 0.9*v + 10 to make cpp publish exactly target_I.
    def _accel_for_target_current(self, target_I):
        v_ff = self.v_ff_coeff * self.cur_v if self.cur_v > self.v_ff_deadzone else 0.0
        return (target_I - self.a2c_baseline - v_ff) / max(self.a2c_gain, 1e-6)

    ### HJ : apply launch override on top of an autodrive ackermann.
    # Steering is NEVER overridden. Only speed/jerk/acceleration get touched.
    # During launch active: jerk=512 with inverse-mapped accel so cpp emits exactly launch_current_A.
    # First tick after DONE: pass-through but enforce safety floor (speed >= measured + small margin)
    # to defend against the edge case where controller's first speed cmd happens to be below measured.
    def _apply_launch_override(self, drive_msg):
        ### HJ : safety gate. If state_machine reports TRAILING, abort launch and
        # fall through so the controller's speed command reaches VESC. No-op when
        # state_machine is absent or state is fresh-but-non-TRAILING.
        self._maybe_abort_launch_for_trailing()
        if self.launch_active and self.launch_t0 is not None:
            t = (rospy.Time.now() - self.launch_t0).to_sec()
            if t >= self.launch_t_total:
                # DONE: hand off to controller, ENABLE one-shot safety floor for next tick(s)
                self.launch_active = False
                self.launch_t0 = None
                self.launch_post_done = True
                self._launch_done_until = rospy.Time.now() + rospy.Duration(0.10)  # 100ms safety window
                self._publish_launch_debug(t, "DONE", drive_msg, target_I=0.0)
                # fall through to safety-floor block below
            else:
                target_I, phase = self._launch_i_cmd(t)
                accel = self._accel_for_target_current(target_I)
                drive_msg.drive.acceleration = accel
                drive_msg.drive.jerk = 512
                drive_msg.drive.speed = 0  # cpp ignores speed when jerk==512
                self._publish_launch_debug(t, "ACTIVE", drive_msg, target_I=target_I, phase=phase)
                return drive_msg

        # Post-launch safety window: enforce speed floor so VESC speed PID gets positive error
        if self.launch_post_done:
            if rospy.Time.now() < self._launch_done_until:
                drive_msg.drive.speed = max(drive_msg.drive.speed, self.cur_v + self.launch_safety_floor_margin)
                self._publish_launch_debug(0.0, "POST", drive_msg, target_I=0.0)
            else:
                self.launch_post_done = False
        return drive_msg

    def _publish_launch_debug(self, t, mode, drive_msg, target_I=0.0, phase=0):
        d = {
            "t": round(float(t), 3),
            "armed": bool(self.launch_armed),
            "active": bool(self.launch_active),
            "post_done": bool(self.launch_post_done),
            "mode": mode,
            "profile": int(self.launch_profile_mode),
            "phase": int(phase),
            "jerk": int(drive_msg.drive.jerk),
            "speed_cmd": round(float(drive_msg.drive.speed), 3),
            "accel_cmd": round(float(drive_msg.drive.acceleration), 4),
            "target_I_A": round(float(target_I), 2),
            "measured_v": round(float(self.cur_v), 3),
            "sm_state": self._active_state() or "NONE",
        }
        self.launch_debug_pub.publish(String(data=json.dumps(d)))

    ### HJ : always-on heartbeat for armed/active state, even when no autodrive ackermann is flowing.
    # Without this, /launch_controller/debug only ticks during launch active, leaving any subscriber's
    # cached `armed` value stale until the first launch fire.
    def _publish_idle_state(self):
        d = {
            "t": 0.0,
            "armed": bool(self.launch_armed),
            "active": bool(self.launch_active),
            "post_done": bool(self.launch_post_done),
            "mode": "IDLE",
            "profile": int(self.launch_profile_mode),
            "phase": 0,
            "jerk": 0,
            "speed_cmd": 0.0,
            "accel_cmd": 0.0,
            "target_I_A": 0.0,
            "measured_v": round(float(self.cur_v), 3),
            "sm_state": self._active_state() or "NONE",
        }
        self.launch_debug_pub.publish(String(data=json.dumps(d)))

    def timer_callback(self, event):
        ### HJ : always publish armed/active heartbeat at 50Hz so test_publisher / debug subs never go stale.
        # _apply_launch_override path will publish a richer message during launch ACTIVE/POST.
        if not self.launch_active and not self.launch_post_done:
            self._publish_idle_state()

        ### HJ : when no button is held, publish zero for 1s then stop
        if self.current_host is None:
            if self.release_time is not None and (rospy.Time.now() - self.release_time).to_sec() < 1.0:
                self.zero_msg.header.stamp = rospy.Time.now()
                self.drive_pub.publish(self.zero_msg)
            return
        if self.current_host == "autodrive" and self.check_uptodate(self.autodrive):
            drive_msg = self.clip_servo(self.autodrive)
            ### HJ : was *= 1.1 (hidden understeer band-aid); set to 1.0 to expose true linkage,
            ### HJ : nonlinear servo cal (steering_servo_poly_coeffs) will absorb the linkage shape.
            drive_msg.drive.steering_angle *= 1.0
            ### HJ : launch override (only when armed+launched). pass-through otherwise.
            drive_msg = self._apply_launch_override(drive_msg)
            self.drive_pub.publish(drive_msg)
        elif self.current_host == "humandrive" and self.check_uptodate(self.human_drive):
            drive_msg = self.clip_servo(self.human_drive)
            # if drive_msg.drive.speed > 0 and self.cur_v < 3.0:
            #     # current_msg = Float64()
            #     # current_msg.data = 50.0
            #     # self.current_pub.publish(current_msg)
            #     # rospy.logwarn(f"joy_command : {drive_msg.drive.speed}" )

            #     drive_msg.drive.speed = 6.0
            #     # self.drive_pub.publish(drive_msg)
            # else:
            self.drive_pub.publish(drive_msg)
            # self.human_drive = None
        # else:
        #     self.drive_pub.publish(self.zero_msg)
    ### HJ : state_machine subscriber. Stores raw string + timestamp.
    def _state_cb(self, msg):
        self._cur_state = msg.data
        self._state_stamp = rospy.Time.now()

    ### HJ : returns active state string or None.
    # None when: (a) state_machine never published (standalone simple_mux), or
    # (b) last message older than state_stale_sec (state_machine died mid-run).
    def _active_state(self):
        if self._state_stamp is None:
            return None
        if (rospy.Time.now() - self._state_stamp).to_sec() > self.state_stale_sec:
            return None
        return self._cur_state

    ### HJ : if active state == TRAILING, kill any in-flight or armed launch.
    # Called every tick from _apply_launch_override before injecting current.
    # Returns True if a launch was just aborted (so override path bails out and
    # falls through to controller speed command).
    def _maybe_abort_launch_for_trailing(self):
        if self._active_state() != 'TRAILING':
            self._trailing_abort_logged = False
            return False
        if not (self.launch_active or self.launch_armed):
            return False
        if not self._trailing_abort_logged:
            rospy.logwarn("[launch] ABORT(TRAILING) armed=%s active=%s -> speed control",
                          self.launch_armed, self.launch_active)
            self._trailing_abort_logged = True
        self.launch_armed = False
        self.launch_active = False
        self.launch_t0 = None
        self.launch_post_done = False
        return True

    ### HJ : equivalent of LB press from external client.
    def _abort_cb(self, _msg):
        rospy.logwarn("[launch] ABORT(external) armed=%s active=%s -> humandrive idle",
                      self.launch_armed, self.launch_active)
        # Reset all launch state
        self.launch_armed = False
        self.launch_active = False
        self.launch_t0 = None
        self.launch_post_done = False
        # Drop autodrive latch and switch to humandrive zero so motor coasts immediately
        self.autodrive_latched = False
        self.human_drive = self.zero_msg
        if self.current_host is not None:
            self.release_time = rospy.Time.now()
        self.current_host = None  # timer_callback's release_time path will broadcast 1s of zero ack

    def planner_callback(self, msg):
        self.vel_planner = msg.data

    def odom_callback(self, msg):
        self.cur_v = msg.twist.twist.linear.x

    def joy_callback(self, msg):
        # prev_host = deepcopy(self.current_host)
        use_human_drive = msg.buttons[4]
        rb_pressed = msg.buttons[5]

        ### HJ : RB rising edge → latch autodrive
        rb_edge_rising = rb_pressed and not self.rb_prev
        self.rb_prev = rb_pressed

        ### HJ : LB rising edge → explicitly abort any in-flight launch (defensive; humandrive already overrides via mux)
        lb_rising = use_human_drive and not self.lb_prev
        self.lb_prev = use_human_drive
        if lb_rising and (self.launch_active or self.launch_armed):
            rospy.logwarn("[launch] ABORT(LB) armed=%s active=%s -> reset both", self.launch_armed, self.launch_active)
            self.launch_armed = False
            self.launch_active = False
            self.launch_t0 = None
            self.launch_post_done = False

        ### HJ : arm button (default A=button[0]) — toggle armed only when not currently launching
        if len(msg.buttons) > self.launch_arm_button:
            a_pressed = msg.buttons[self.launch_arm_button]
            a_rising = a_pressed and not self.a_prev
            self.a_prev = a_pressed
            if a_rising and not self.launch_active:
                old = self.launch_armed
                self.launch_armed = not self.launch_armed
                ### HJ : full button snapshot to identify cross-channel triggers
                btn_snap = ",".join(str(b) for b in msg.buttons[:8])
                rospy.loginfo("[launch] A_TOGGLE %s->%s (a_pressed=%d a_prev_was=%d, btns0-7=[%s])",
                              old, self.launch_armed, int(a_pressed), int(not a_pressed) ^ 0, btn_snap)

        if use_human_drive:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.drive.steering_angle = msg.axes[3] * self.max_steer
            drive_msg.drive.speed = msg.axes[1] * self.max_speed


            # drive_msg.drive.jerk = 512.0
            # del_v = drive_msg.drive.speed -self.cur_v
            # drive_msg.drive.acceleration = del_v * 4.0 + (del_v - self.prev_del_v) * 0.1

            self.human_drive = drive_msg
            self.current_host = "humandrive"
            ### HJ : LB takeover clears autodrive latch; need RB re-press to resume
            self.autodrive_latched = False
        elif rb_edge_rising:
            ### HJ : RB pressed → latch autodrive ON
            self.autodrive_latched = True
            self.current_host = "autodrive"
            ### HJ : trace whether RB rising sees armed=True (fire) or False (no-fire)
            btn_snap = ",".join(str(b) for b in msg.buttons[:8])
            rospy.loginfo("[launch] RB_RISING armed=%s active=%s (btns0-7=[%s])",
                          self.launch_armed, self.launch_active, btn_snap)
            ### HJ : if armed, fire launch one-shot. armed -> False, active -> True at this exact tick.
            if self.launch_armed and not self.launch_active:
                self.launch_armed = False
                self.launch_active = True
                self.launch_t0 = rospy.Time.now()
                rospy.logwarn("[launch] FIRED at t=%.3f, current=%.1fA for %.2fs (one-shot armed->False)",
                              self.launch_t0.to_sec(), self.launch_current_A, self.launch_t_total)
        elif self.autodrive_latched:
            ### HJ : latch held → stay in autodrive even when RB released
            self.current_host = "autodrive"
        else:
            ### HJ : no active control → zero out for 1s then idle
            if self.current_host is not None:
                self.release_time = rospy.Time.now()
            self.human_drive = self.zero_msg
            self.current_host = None

    # def drive_callback(self, msg):
    #     self.autodrive = msg


    def drive_callback(self, msg):
        drive_msg = AckermannDriveStamped()
        drive_msg = msg
        # drive_msg.drive.speed = self.vel_planner
        # drive_msg.drive.jerk = 512.0
        # del_v = drive_msg.drive.speed -self.cur_v
        # drive_msg.drive.acceleration = del_v * 4.0 + (del_v - self.prev_del_v) * 0.1
        # if drive_msg.drive.acceleration >0:
        #     drive_msg.drive.steering_angle *= (1+drive_msg.drive.acceleration*0.4)
        # else:
        #     drive_msg.drive.steering_angle *= (1+drive_msg.drive.acceleration*0.2)

        # self.prev_del_v = del_v

        self.autodrive = drive_msg



if __name__ == '__main__':
    simple_mux = SimpleMuxNode()
    rospy.spin()
