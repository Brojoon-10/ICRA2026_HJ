#!/usr/bin/env python
"""
LiDAR-IMU time sync diagnostic.
Usage: python check_time_sync.py [duration_sec] [lidar_topic] [imu_topic]
Default: 10s, /livox/lidar, /ekf/imu/data
"""
import rospy
import roslib.message
from collections import deque
import statistics
import time
import sys
import bisect


class TimeSyncLogger:
    def __init__(self, lidar_topic, imu_topic, duration):
        self.lidar_topic = lidar_topic
        self.imu_topic = imu_topic
        self.duration = duration
        self.lidar_stamps = deque(maxlen=200000)
        self.imu_stamps = deque(maxlen=200000)

    @staticmethod
    def s(stamp):
        return stamp.secs + stamp.nsecs * 1e-9

    def lidar_cb(self, msg):
        self.lidar_stamps.append((self.s(msg.header.stamp), self.s(rospy.Time.now())))

    def imu_cb(self, msg):
        self.imu_stamps.append((self.s(msg.header.stamp), self.s(rospy.Time.now())))

    def get_msg_class(self, topic):
        for _ in range(50):
            topics = dict(rospy.get_published_topics())
            if topic in topics:
                return roslib.message.get_message_class(topics[topic]), topics[topic]
            time.sleep(0.1)
        return None, None

    def run(self):
        rospy.init_node('time_sync_logger', anonymous=True)
        lcls, ltype = self.get_msg_class(self.lidar_topic)
        icls, itype = self.get_msg_class(self.imu_topic)
        if not lcls or not icls:
            print("Topic not found. lidar=%s imu=%s" % (ltype, itype))
            return
        print("Sub: %s (%s)" % (self.lidar_topic, ltype))
        print("Sub: %s (%s)" % (self.imu_topic, itype))
        rospy.Subscriber(self.lidar_topic, lcls, self.lidar_cb, queue_size=500)
        rospy.Subscriber(self.imu_topic, icls, self.imu_cb, queue_size=2000)
        print("Logging %.1fs..." % self.duration)
        rospy.sleep(self.duration)
        self.report()

    def stat(self, label, vals_ms):
        if not vals_ms:
            print("  %s: no data" % label)
            return
        print("  %s: mean=%.3fms  std=%.3fms  min=%.3fms  max=%.3fms  n=%d" %
              (label, statistics.mean(vals_ms),
               statistics.stdev(vals_ms) if len(vals_ms) > 1 else 0.0,
               min(vals_ms), max(vals_ms), len(vals_ms)))

    def report(self):
        L = list(self.lidar_stamps)
        I = list(self.imu_stamps)
        print("\n========== RESULT ==========")
        print("LiDAR n=%d   IMU n=%d" % (len(L), len(I)))
        if not L or not I:
            print("Not enough samples"); return

        # rates
        if len(L) > 1:
            print("LiDAR rate: %.2f Hz" % ((len(L) - 1) / (L[-1][1] - L[0][1])))
        if len(I) > 1:
            print("IMU   rate: %.2f Hz" % ((len(I) - 1) / (I[-1][1] - I[0][1])))

        # epoch sanity
        print("\nFirst stamps (epoch sanity):")
        print("  LiDAR header.stamp = %.6f  (%s)" %
              (L[0][0], "WALL CLOCK" if L[0][0] > 1e9 else "DEVICE BOOT TIME (BAD)"))
        print("  IMU   header.stamp = %.6f  (%s)" %
              (I[0][0], "WALL CLOCK" if I[0][0] > 1e9 else "DEVICE BOOT TIME (BAD)"))

        # delay (recv - stamp), in ms
        ldelay = [(r - s) * 1000 for s, r in L]
        idelay = [(r - s) * 1000 for s, r in I]
        print("\nDelay (rospy.Time.now() - header.stamp):")
        self.stat("LiDAR", ldelay)
        self.stat("IMU  ", idelay)

        # cross diff: for each lidar stamp find nearest imu stamp
        imu_sorted = sorted([s for s, _ in I])
        cross = []
        for ls, _ in L:
            idx = bisect.bisect_left(imu_sorted, ls)
            cand = []
            if idx < len(imu_sorted): cand.append(imu_sorted[idx])
            if idx > 0: cand.append(imu_sorted[idx - 1])
            if cand:
                cross.append((ls - min(cand, key=lambda x: abs(x - ls))) * 1000)
        print("\nLiDAR stamp - nearest IMU stamp:")
        self.stat("cross", cross)

        # interpretation
        print("\n========== DIAGNOSIS ==========")
        warnings = []
        if L[0][0] < 1e9:
            warnings.append("LiDAR stamp is device boot time, not wall clock. Patch header.stamp = ros::Time::now() in lddc.cpp.")
        if I[0][0] < 1e9:
            warnings.append("IMU stamp is device boot time, not wall clock. Check IMU driver.")
        if ldelay and abs(statistics.mean(ldelay)) > 50:
            warnings.append("LiDAR delay > 50ms. Either stamp is wrong epoch or large network/processing latency.")
        if cross and abs(statistics.mean(cross)) > 20:
            warnings.append("LiDAR-IMU stamp offset > 20ms on average. SLAM IMU prediction will be off.")
        if not warnings:
            print("OK: timestamps look consistent with wall clock and aligned within 20ms.")
        else:
            for w in warnings:
                print("WARN: " + w)


if __name__ == '__main__':
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    lidar = sys.argv[2] if len(sys.argv) > 2 else '/livox/lidar'
    imu = sys.argv[3] if len(sys.argv) > 3 else '/ekf/imu/data'
    TimeSyncLogger(lidar, imu, duration).run()
