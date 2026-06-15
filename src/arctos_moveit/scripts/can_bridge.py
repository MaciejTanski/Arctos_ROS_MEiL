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
GEAR_RATIOS = [6.75, 30, 75, 24, 33.91, 33.91]
INVERT      = [True, True, False, False, False, False]
SPEED = 100 # default 500
ACC   = 1   #default 2
USE_DEGREES = True
SCALE       = 100   #default 100
BUSTYPE = "slcan"          # 'slcan' (/dev/ttyACM0) lub 'socketcan' (can0)
CHANNEL = "/dev/ttyACM0"       # "/dev/ttyACM0"
BITRATE = 500000
RATE_HZ = 20
DEADBAND_PULSES = 2 #default 2
DRY_RUN = False
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
        targets = self.compute_motor_targets()
        for i in range(6):
            pulses = targets[i]
            if pulses is None:
                continue

            if self.initial[i] is None:
                self.initial[i] = pulses
                self.last_sent[i] = 0
                continue

            rel   = pulses - self.initial[i]
            delta = rel - self.last_sent[i]
            if abs(delta) < DEADBAND_PULSES:
                continue

            self.last_sent[i] = rel
            frame = encode_mks_position_frame(AXIS_IDS[i], SPEED, ACC, delta)
            hexd = " ".join(f"{b:02X}" for b in frame.data)
            rospy.loginfo("os %d rel=%d delta=%d -> ID=%02X data=[%s]",
                          AXIS_IDS[i], rel, delta, frame.arbitration_id, hexd)
            if not DRY_RUN:
                try:
                    self.bus.send(frame)
                except Exception as e:
                    rospy.logerr("Blad wysylki CAN: %s", e)


    def compute_motor_targets(self):
        """Docelowe pulsy SILNIKow [m1..m6] z roznicowym sprzezeniem joint5/joint6."""
        targets = [None] * 6

        # Osie 1-4: niezalezne
        for i in range(4):
            jname = JOINT_NAMES[i]
            if jname in self.latest:
                targets[i] = angle_to_pulses(self.latest[jname], GEAR_RATIOS[i], INVERT[i])

        # Osie 5-6: rozniczka (zebatki stozkowe)
        if "joint5" in self.latest and "joint6" in self.latest:
            p5 = angle_to_pulses(self.latest["joint5"], GEAR_RATIOS[4], INVERT[4])
            p6 = angle_to_pulses(self.latest["joint6"], GEAR_RATIOS[5], INVERT[5])
            targets[4] = p6 + p5   # AXIS_ID 5
            targets[5] = p6 - p5   # AXIS_ID 6

        return targets


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
