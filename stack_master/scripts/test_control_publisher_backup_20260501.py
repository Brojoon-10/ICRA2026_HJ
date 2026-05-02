#!/usr/bin/env python3
### HJ : Manual launch test rig.
#
# Replaces the autonomy stack for launch-control bench tests.
# Publishes a constant ackermann (steering=0, speed=rqt-tunable) at 50Hz to
#   /vesc/high_level/ackermann_cmd_mux/input/nav_1
# so that pressing RB on the joypad latches simple_mux into autodrive mode and
# starts forwarding our command to custom_ackermann_to_vesc.
#
# Usage flow:
#   1. Launch: low_level.launch (vesc_driver + simple_mux + joy)
#   2. Run:    rosrun stack_master test_control_publisher.py
#   3. rqt:    Plugins > Configuration > Dynamic Reconfigure > /test_control_publisher
#              Set cmd_speed_mps to your post-launch target (e.g. 2.0).
#   4. A button -> arm launch (toggle, see /vesc/simple_mux logs)
#   5. RB       -> autodrive on; if armed, simple_mux fires launch (0.75s, 15A default)
#   6. This terminal prints REAL motor current from /vesc/sensors/core every tick
#      during launch + 0.5s after, then idles at 1Hz.
#
# Safety:
#   - Steering forced to 0 unless you change cmd_steering_rad in rqt.
#   - LB on joypad takes over instantly via simple_mux humandrive path.
#   - cmd_speed_mps>0 means car will roll forward when RB pressed even WITHOUT arming launch.
#     Keep wheels off the ground until you've verified the chain.
import csv
import datetime as _dt
import json
import math
import os
import threading
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from vesc_msgs.msg import VescStateStamped
from dynamic_reconfigure.server import Server as DynRecServer
from stack_master.cfg import TestCtrlConfig


PUB_TOPIC = "/vesc/high_level/ackermann_cmd_mux/input/nav_1"
CORE_TOPIC = "/vesc/sensors/core"
LAUNCH_DBG_TOPIC = "/launch_controller/debug"

### HJ : per-launch CSV log dir. Maps to host's HJ_docs/debug/launch_logs.
DEFAULT_LOG_DIR = os.path.expanduser("~/catkin_ws/src/race_stack/HJ_docs/debug/launch_logs")


class TestControlPublisher:
    def __init__(self):
        rospy.init_node("test_control_publisher", anonymous=False)

        # Tunable via rqt
        self.cmd_speed = 0.0
        self.cmd_steering = 0.0

        # Telemetry from VESC
        self.current_motor = 0.0
        self.current_input = 0.0
        self.duty_cycle = 0.0
        self.measured_erpm = 0.0
        self.voltage_input = 0.0

        # speed_to_erpm_gain to convert erpm -> m/s for log readability
        self.s2e_gain = float(rospy.get_param("/vesc/speed_to_erpm_gain", 1875.0))
        self.s2e_offset = float(rospy.get_param("/vesc/speed_to_erpm_offset", 0.0))

        # Launch state mirrored from simple_mux debug topic
        self.launch_active = False
        self.launch_armed = False
        self.launch_t = 0.0
        self.launch_mode = "INIT"
        self.launch_jerk = 0
        self.launch_speed_cmd = 0.0
        self.launch_accel_cmd = 0.0
        self.launch_target_I = 0.0
        self.launch_window_until = rospy.Time(0)  # high-freq print + CSV until this time
        self._was_in_window = False
        self._launch_seq = 0  # incremented per launch event for log filename uniqueness

        ### HJ : CSV logger (one file per launch event)
        self.log_dir = rospy.get_param("~log_dir", DEFAULT_LOG_DIR)
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            rospy.loginfo("[test_ctrl] CSV log dir: %s", self.log_dir)
        except OSError as e:
            rospy.logwarn("[test_ctrl] cannot create log dir %s: %s -- CSV disabled", self.log_dir, e)
            self.log_dir = None
        self._csv_file = None
        self._csv_writer = None
        self._csv_path = None

        # Publisher (replaces autonomy stack)
        self.pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=10)
        # Subscribers
        rospy.Subscriber(CORE_TOPIC, VescStateStamped, self._core_cb, queue_size=10)
        rospy.Subscriber(LAUNCH_DBG_TOPIC, String, self._launch_dbg_cb, queue_size=10)

        # rqt
        self.dyn_srv = DynRecServer(TestCtrlConfig, self._dyn_cb)

        # 50Hz publish + monitor tick
        rospy.Timer(rospy.Duration(1.0 / 50.0), self._tick)

        # ### HJ : guarantee CSV close on node shutdown (Ctrl+C, kill, launch teardown)
        rospy.on_shutdown(self._close_csv)

        rospy.loginfo("[test_ctrl] up. publishing %s @ 50Hz. Monitoring %s",
                      PUB_TOPIC, CORE_TOPIC)
        rospy.loginfo("[test_ctrl] WHEELS OFF GROUND until verified. cmd_speed=%.2f m/s",
                      self.cmd_speed)

    def _dyn_cb(self, config, level):
        self.cmd_speed = float(config.cmd_speed_mps)
        self.cmd_steering = float(config.cmd_steering_rad)
        rospy.loginfo("[test_ctrl] tuned: speed=%.2fm/s steer=%.3frad",
                      self.cmd_speed, self.cmd_steering)
        return config

    def _core_cb(self, msg):
        self.current_motor = float(msg.state.current_motor)
        self.current_input = float(msg.state.current_input)
        self.duty_cycle = float(msg.state.duty_cycle)
        self.measured_erpm = float(msg.state.speed)
        self.voltage_input = float(msg.state.voltage_input)

    def _launch_dbg_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        was_active = self.launch_active
        self.launch_armed = bool(d.get("armed", False))
        self.launch_active = bool(d.get("active", False))
        self.launch_t = float(d.get("t", 0.0))
        self.launch_mode = str(d.get("mode", "INIT"))
        self.launch_jerk = int(d.get("jerk", 0))
        self.launch_speed_cmd = float(d.get("speed_cmd", 0.0))
        self.launch_accel_cmd = float(d.get("accel_cmd", 0.0))
        self.launch_target_I = float(d.get("target_I_A", 0.0))

        # Rising edge: open new CSV file
        if not was_active and self.launch_active:
            self._open_csv()

        if self.launch_active:
            # Extend window for 0.5s after launch ends (covers POST handoff phase)
            self.launch_window_until = rospy.Time.now() + rospy.Duration(0.5)

    def _open_csv(self):
        if self.log_dir is None:
            return
        self._launch_seq += 1
        ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(self.log_dir, "launch_{}_seq{:03d}.csv".format(ts, self._launch_seq))
        try:
            self._csv_file = open(self._csv_path, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                "wall_t",            # ROS time epoch sec
                "launch_t",          # since launch start (from simple_mux dbg)
                "active", "armed", "mode",
                "cmd_speed_pub",     # what test_publisher publishes (controller proxy)
                "cmd_steer_pub",
                "mux_jerk",          # what simple_mux actually sent to cpp (511/512/0)
                "mux_speed_cmd",     # what simple_mux actually sent for speed
                "mux_accel_cmd",
                "mux_target_I_A",    # intended current
                "I_motor_actual",    # /sensors/core ground truth
                "I_input_actual",
                "duty_actual",
                "erpm_actual",
                "v_mps_actual",
                "V_input_actual",
            ])
            self._csv_file.flush()
            rospy.loginfo("[test_ctrl] CSV opened: %s", self._csv_path)
        except OSError as e:
            rospy.logwarn("[test_ctrl] CSV open failed: %s", e)
            self._csv_file = None
            self._csv_writer = None

    def _close_csv(self):
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
                rospy.loginfo("[test_ctrl] CSV closed: %s", self._csv_path)
            except Exception as e:
                rospy.logwarn("[test_ctrl] CSV close error: %s", e)
            self._csv_file = None
            self._csv_writer = None

    def _measured_v_mps(self):
        # Inverse of speed_to_erpm: v = (erpm - offset) / gain
        if abs(self.s2e_gain) < 1e-6:
            return 0.0
        return (self.measured_erpm - self.s2e_offset) / self.s2e_gain

    def _tick(self, _evt):
        # 1. Publish ackermann at 50Hz (constant unless rqt-changed)
        now = rospy.Time.now()
        m = AckermannDriveStamped()
        m.header.stamp = now
        m.drive.steering_angle = self.cmd_steering
        m.drive.speed = self.cmd_speed
        m.drive.acceleration = 0.0
        m.drive.jerk = 0.0
        self.pub.publish(m)

        # 2. Monitor + CSV log — high freq during launch + 0.5s after, low freq otherwise
        v_mps = self._measured_v_mps()
        in_window = now < self.launch_window_until

        # CSV row every tick while in window (50Hz full)
        if in_window and self._csv_writer is not None:
            try:
                self._csv_writer.writerow([
                    "{:.4f}".format(now.to_sec()),
                    "{:.4f}".format(self.launch_t),
                    int(self.launch_active),
                    int(self.launch_armed),
                    self.launch_mode,
                    "{:.3f}".format(self.cmd_speed),
                    "{:.4f}".format(self.cmd_steering),
                    self.launch_jerk,
                    "{:.3f}".format(self.launch_speed_cmd),
                    "{:.4f}".format(self.launch_accel_cmd),
                    "{:.2f}".format(self.launch_target_I),
                    "{:.3f}".format(self.current_motor),
                    "{:.3f}".format(self.current_input),
                    "{:.4f}".format(self.duty_cycle),
                    "{:.0f}".format(self.measured_erpm),
                    "{:.3f}".format(v_mps),
                    "{:.2f}".format(self.voltage_input),
                ])
            except Exception as e:
                rospy.logwarn_throttle(1.0, "[test_ctrl] CSV write error: %s", e)

        # Window-end falling edge: flush + close CSV
        if self._was_in_window and not in_window:
            self._close_csv()
        self._was_in_window = in_window

        # Print: window=10Hz (readable), idle=1Hz
        if in_window:
            tag = "[LAUNCH]" if self.launch_active else "[POST  ]"
            rospy.loginfo_throttle(
                0.10,
                "%s t=%.2f mode=%s cmd_pub=%.2fm/s | mux: jerk=%d sp=%.2f acc=%.3f tgtI=%5.2fA "
                "| ACTUAL: I=%6.2fA Iin=%5.2fA duty=%5.3f erpm=%6.0f v=%5.2fm/s V=%4.1fV",
                tag, self.launch_t, self.launch_mode, self.cmd_speed,
                self.launch_jerk, self.launch_speed_cmd, self.launch_accel_cmd, self.launch_target_I,
                self.current_motor, self.current_input, self.duty_cycle,
                self.measured_erpm, v_mps, self.voltage_input,
            )
        else:
            rospy.loginfo_throttle(
                1.0,
                "[idle  ] cmd_pub=%.2fm/s | I=%6.2fA duty=%5.3f erpm=%6.0f v=%5.2fm/s V=%4.1fV armed=%s",
                self.cmd_speed,
                self.current_motor, self.duty_cycle,
                self.measured_erpm, v_mps, self.voltage_input,
                self.launch_armed,
            )


if __name__ == "__main__":
    try:
        TestControlPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
