#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
#  can_bridge.py — F5 streaming z real-time update + per-joint ACC + stall monitor
# ----------------------------------------------------------------------------
#  Predkosc per-tick odczytywana z trajektorii MoveIt (delta pozycji / okres tiku),
#  pozycja absolutna F5 (real-time update, dok. 11.4.4). Zachowuje ksztalt trajektorii.
#
#  ACC jest PER-JOINT, bo realizowalne przyspieszenie zalezy od bezwladnosci
#  obciazenia danej osi — j2 (gear 225, dzwiga ramie) zniesie inne acc niz lekki
#  nadgarstek j6. acc to NIE fizyczne przyspieszenie tylko tempo schodkowej zmiany
#  predkosci firmware (dok. 9.1: 1 RPM co (256-acc)*50us). Czy silnik za tym nadazy
#  mechanicznie — sprawdza sie EMPIRYCZNIE przez odczyt stall 0x3E.
#
#  PROCEDURA KALIBRACJI ACC (per os):
#    1. Zacznij od umiarkowanego acc (np. 120). NIE od maksimum.
#    2. Wykonaj ruch. Most automatycznie odczyta 0x3E po zakonczeniu.
#    3. Stall=0 i ruch plynny -> podnies acc o ~20, powtorz.
#    4. Stall=1 albo ruch wibruje/buczy -> za stromo. Cofnij sie o krok.
#    5. Wpisz bezpieczna wartosc (krok ponizej granicy) do ACC[i].
# ============================================================================

import math
import threading
import rospy
import can
from sensor_msgs.msg import JointState

# ============================ KONFIG =============================
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
AXIS_IDS    = [1,    2,    3,    4,    5,    6   ]
GEAR_RATIOS = [13.5, 150,  200,   48,   30, 30] #j5 80, j6 50
INVERT      = [True, True, True, False, True, True]

# --- ACC PER-JOINT (0-255). START OSTROZNY. Kalibruj wg procedury w naglowku. ---
#               j1    j2    j3    j4    j5    j6
ACC         =  [120,  100,  110,  120,  130,  130]

# Predkosc wysylana w ramce = tempo, z jakim MoveIt rusza osia (delta/tik -> RPM),
# ograniczona do [SPEED_MIN, SPEED_MAX] per os.
SPEED_MAX   =  [100,  140,  140,  140,  140,  140]   # gorny limit RPM (0-3000)
SPEED_MIN   =  [10,   20,   25,   30,   30,   30]    # podloga, by mikroramki ruszyly

DEADBAND_PULSES = 3     # filtr szumu pozycji

# --- Monitor stall (0x3E) ---
STALL_CHECK_ENABLED = True   # po wykryciu konca ruchu odczytaj stan stall
STALL_AUTO_RELEASE  = False  # True = automatyczne 0x3D po stall (uzywaj swiadomie)
SETTLE_EPS   = 20            # cel uznany za ustalony gdy zmiana < tylu pulsow
SETTLE_TICKS = 6             # przez tyle kolejnych tickow (6*50ms = 300ms ciszy)

BUSTYPE = "slcan"
CHANNEL = "/dev/ttyACM0"
BITRATE = 500000
RATE_HZ = 100
DRY_RUN = False
# =================================================================

CMD_F5       = 0xF5
CMD_STALL_RD = 0x3E   # odczyt stanu stall
CMD_STALL_RL = 0x3D   # zwolnienie ze stanu stall
POS_MIN, POS_MAX = -8_388_608, 8_388_607
PULSES_PER_REV = 16384


def angle_to_pulses(angle_rad, gear, invert):
    revolutions_motor = (angle_rad / (2.0 * math.pi)) * gear
    pulses = revolutions_motor * PULSES_PER_REV
    return int(round(-pulses if invert else pulses))


def pulses_per_tick_to_rpm(delta_pulses):
    """Predkosc [RPM] by pokonac delta_pulses w jednym okresie ramki."""
    revs = abs(delta_pulses) / float(PULSES_PER_REV)
    return revs * RATE_HZ * 60.0


def encode_f5_frame(axis_id, speed, acc, pos_pulses):
    pos = pos_pulses & 0xFFFFFF
    data = [CMD_F5,
            (speed >> 8) & 0xFF, speed & 0xFF,
            acc & 0xFF,
            (pos >> 16) & 0xFF, (pos >> 8) & 0xFF, pos & 0xFF]
    data.append((axis_id + sum(data)) & 0xFF)
    return can.Message(arbitration_id=axis_id, data=data, is_extended_id=False)


def encode_simple_cmd(axis_id, code):
    """2-bajtowa komenda [code, CRC] — np. odczyt 0x3E, release 0x3D."""
    crc = (axis_id + code) & 0xFF
    return can.Message(arbitration_id=axis_id, data=[code, crc], is_extended_id=False)


class CanBridge:
    def __init__(self):
        self.last_sent     = [None] * 6
        self.latest_pos    = {}
        self.bus           = None
        self.warned_range  = [False] * 6

        # detekcja konca ruchu (do wyzwolenia kontroli stall)
        self.settle_buf    = [None] * 6
        self.settle_count  = [0] * 6
        self.moved_since   = [False] * 6   # czy os ruszyla od ostatniego sprawdzenia
        self.stall_evt     = [threading.Event() for _ in range(6)]
        self.stall_state   = [None] * 6
        self.lock          = threading.Lock()

        if not DRY_RUN:
            self.bus = can.interface.Bus(bustype=BUSTYPE, channel=CHANNEL, bitrate=BITRATE)
            rospy.loginfo("CAN otwarty: %s %s @ %d bps (F5 streaming)", BUSTYPE, CHANNEL, BITRATE)
            rospy.loginfo("ACC per joint: %s", ACC)
            if STALL_CHECK_ENABLED:
                self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
                self.rx_thread.start()
                rospy.loginfo("Monitor stall (0x3E) aktywny.")
        else:
            rospy.logwarn("DRY_RUN=True — brak transmisji i odczytu stall")

        rospy.loginfo("ZAKLADAM ze 0x92 (zerowanie) zostal wyslany przed startem.")
        rospy.Subscriber("/joint_states", JointState, self.cb, queue_size=10)

    def cb(self, msg):
        with self.lock:
            for idx, name in enumerate(msg.name):
                if idx < len(msg.position):
                    self.latest_pos[name] = msg.position[idx]

    # ---- Odbior odpowiedzi z magistrali (stall) ----
    def _rx_loop(self):
        while not rospy.is_shutdown():
            try:
                m = self.bus.recv(timeout=0.2)
            except Exception:
                continue
            if m is None or m.dlc < 3:
                continue
            if m.arbitration_id not in AXIS_IDS:
                continue
            i = AXIS_IDS.index(m.arbitration_id)
            code = m.data[0]
            if code == CMD_STALL_RD:
                self.stall_state[i] = m.data[1]
                self.stall_evt[i].set()
            elif code == CMD_STALL_RL:
                rospy.loginfo("os %d: release stall status=%d", AXIS_IDS[i], m.data[1])

    def compute_targets(self):
        with self.lock:
            lp = dict(self.latest_pos)
        targets = [None] * 6
        for i in range(4):
            jn = JOINT_NAMES[i]
            if jn in lp:
                targets[i] = angle_to_pulses(lp[jn], GEAR_RATIOS[i], INVERT[i])
        if "joint5" in lp and "joint6" in lp:
            p5 = angle_to_pulses(lp["joint5"], GEAR_RATIOS[4], INVERT[4])
            p6 = angle_to_pulses(lp["joint6"], GEAR_RATIOS[5], INVERT[5])
            targets[4] = p6 - p5
            targets[5] = p6 + p5
        return targets

    # ---- Sprawdzenie stanu stall po zakonczeniu ruchu osi ----
    def check_stall(self, i):
        if DRY_RUN or self.bus is None:
            return
        self.stall_evt[i].clear()
        try:
            self.bus.send(encode_simple_cmd(AXIS_IDS[i], CMD_STALL_RD))
        except Exception as e:
            rospy.logerr("os %d: blad wyslania 0x3E: %s", AXIS_IDS[i], e)
            return
        if not self.stall_evt[i].wait(timeout=0.5):
            rospy.logwarn("os %d: brak odpowiedzi na 0x3E (sprawdz 8CH/okablowanie)", AXIS_IDS[i])
            return
        st = self.stall_state[i]
        if st == 1:
            rospy.logerr("os %d: STALL! acc=%d za strome dla obciazenia — zmniejsz ACC[%d]",
                         AXIS_IDS[i], ACC[i], i)
            if STALL_AUTO_RELEASE:
                self.bus.send(encode_simple_cmd(AXIS_IDS[i], CMD_STALL_RL))
                rospy.logwarn("os %d: wyslano release 0x3D", AXIS_IDS[i])
        else:
            rospy.loginfo("os %d: stall=0 (acc=%d OK dla tego ruchu)", AXIS_IDS[i], ACC[i])

    def tick(self):
        targets = self.compute_targets()

        for i in range(6):
            target = targets[i]
            if target is None:
                continue

            prev = self.last_sent[i]

            # --- detekcja konca ruchu (settle) do kontroli stall ---
            sb = self.settle_buf[i]
            self.settle_buf[i] = target
            if sb is not None and abs(target - sb) <= SETTLE_EPS:
                self.settle_count[i] += 1
            else:
                self.settle_count[i] = 0
            # ruch wlasnie sie zakonczyl i os faktycznie sie poruszala -> sprawdz stall
            if STALL_CHECK_ENABLED and self.moved_since[i] \
                    and self.settle_count[i] == SETTLE_TICKS:
                self.moved_since[i] = False
                threading.Thread(target=self.check_stall, args=(i,), daemon=True).start()

            if prev is not None and abs(target - prev) < DEADBAND_PULSES:
                continue

            if target > POS_MAX or target < POS_MIN:
                if not self.warned_range[i]:
                    rospy.logwarn("os %d: cel %d poza 24-bit", AXIS_IDS[i], target)
                    self.warned_range[i] = True
                target = max(POS_MIN, min(POS_MAX, target))

            # predkosc = tempo trajektorii MoveIt na tej osi
            if prev is None:
                speed = SPEED_MIN[i]
            else:
                rpm = pulses_per_tick_to_rpm(target - prev)
                speed = int(round(max(SPEED_MIN[i], min(SPEED_MAX[i], rpm))))

            self.last_sent[i] = target
            self.moved_since[i] = True
            frame = encode_f5_frame(AXIS_IDS[i], speed, ACC[i], target)
            hexd = " ".join(f"{b:02X}" for b in frame.data)
            rospy.loginfo("os %d target=%d speed=%d acc=%d -> [%s]",
                          AXIS_IDS[i], target, speed, ACC[i], hexd)

            if not DRY_RUN and self.bus is not None:
                try:
                    self.bus.send(frame)
                except Exception as e:
                    rospy.logerr("Blad wysylki CAN os %d: %s", AXIS_IDS[i], e)

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