#!/usr/bin/env python3
# MOST: /joint_states (MoveIt) -> ramki CAN do serw MKS Servo (encoding z arctosgui)
# DRY_RUN=True -> tylko wypisuje ramki, nic nie wysyla na CAN.
import math
import rospy
import can
from sensor_msgs.msg import JointState

# ===== KONFIG =====
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]  # z SRDF, grupa "arm"
AXIS_IDS    = [1, 2, 3, 4, 5, 6]
GEAR_RATIOS = [6.75, 75, 75, 24, 33.91, 33.91]
INVERT      = [True, True, False, False, False, False]
SPEED = 500 # default 500
ACC   = 1   #default 2
USE_DEGREES = True
SCALE       = 100   #default 100
BUSTYPE = "slcan"          # 'slcan' (/dev/ttyACM0) lub 'socketcan' (can0)
CHANNEL = "/dev/ttyACM0"       # "/dev/ttyACM0"
BITRATE = 500000
RATE_HZ = 20
DEADBAND_PULSES = 1 #default 2
DRY_RUN = True
# ==================


def angle_to_pulses(angle_rad, gear, invert):
    angle = math.degrees(angle_rad) if USE_DEGREES else angle_rad
    pulses = angle * gear * SCALE
    if invert:
        pulses = -pulses
    return int(round(pulses))


def encode_mks_position_frame(axis_id, speed, acc, rel_pulses):
    pos = rel_pulses & 0xFFFFFF
    data = [
        0xF4,
        (speed >> 8) & 0xFF, speed & 0xFF,
        acc & 0xFF,
        (pos >> 16) & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF,
    ]
    crc = (axis_id + sum(data)) & 0xFF
    data.append(crc)
    return can.Message(arbitration_id=axis_id, data=data, is_extended_id=False)


class CanBridge:
    def __init__(self):
        self.initial = [None] * 6
        self.last_sent = [None] * 6
        self.latest = {}
        self.bus = None
        if not DRY_RUN:
            self.bus = can.interface.Bus(bustype=BUSTYPE, channel=CHANNEL, bitrate=BITRATE)
            rospy.loginfo("CAN otwarty: %s %s @ %d", BUSTYPE, CHANNEL, BITRATE)
        else:
            rospy.logwarn("DRY_RUN=True - ramki tylko wypisywane, NIC nie idzie na CAN.")
        rospy.Subscriber("/joint_states", JointState, self.cb, queue_size=10)

    def cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.latest[name] = pos

    def tick(self):
        for i, jname in enumerate(JOINT_NAMES):
            if jname not in self.latest:
                continue
            pulses = angle_to_pulses(self.latest[jname], GEAR_RATIOS[i], INVERT[i])
            if self.initial[i] is None:
                self.initial[i] = pulses
            rel = pulses - self.initial[i]
            if self.last_sent[i] is not None and abs(rel - self.last_sent[i]) < DEADBAND_PULSES:
                continue
            self.last_sent[i] = rel
            frame = encode_mks_position_frame(AXIS_IDS[i], SPEED, ACC, rel)
            hexd = " ".join(f"{b:02X}" for b in frame.data)
            rospy.loginfo("os %d (%s) rel=%d -> ID=%02X data=[%s]",
                          AXIS_IDS[i], jname, rel, frame.arbitration_id, hexd)
            if not DRY_RUN:
                try:
                    self.bus.send(frame)
                except Exception as e:
                    rospy.logerr("Blad wysylki CAN: %s", e)

    def spin(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            self.tick()
            rate.sleep()
        if self.bus is not None:
            self.bus.shutdown()


if __name__ == "__main__":
    rospy.init_node("can_bridge")
    CanBridge().spin()
