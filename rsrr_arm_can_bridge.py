#!/usr/bin/env python3
"""
rsrr_arm_can_bridge.py  --  route-B hardware bridge for the REAL rsrr_arm.

Replaces the MuJoCo bridge. It speaks the SAME LCM topics the MATLAB
controller already uses, but instead of a simulator it drives the two
Damiao DM-J3507 motors over CAN (MIT mode) and republishes their feedback:

    MATLAB rsrr_arm_control_real.m
        | LCM rsrr_arm_control (torques) --> this bridge --> CAN --> motors
        | LCM rsrr_arm_state   (angles)  <-- this bridge <-- CAN <-- motors

Run (in the arcpy env, adapter plugged in, motors powered at 24 V):
    python rsrr_arm_can_bridge.py

Deps:  python-can  (pip install python-can);  lcm  (already in arcpy).

!!! FILL IN the CONFIG block below from your Damiao debugging assistant and
    your adapter BEFORE running. Wrong PMAX/VMAX/TMAX => wrong torque. !!!
"""
import time
import threading

import can
import lcm
from arc_bridge.lcm_msgs import rsrr_arm_state_t, rsrr_arm_control_t

# ======================= CONFIG (FILL THESE IN) =======================
# --- adapter (route B: serial USB-CAN adapter on macOS) ---
CAN_BUSTYPE = "slcan"                     # CANable/slcan.  (PCAN -> 'pcan', Kvaser -> 'kvaser')
CAN_CHANNEL = "/dev/tty.usbmodem1101"     # <-- your adapter: run  ls /dev/tty.usb*
CAN_BITRATE = 1_000_000                   # Damiao default 1 Mbps

# --- motor CAN send IDs (set in the Damiao GUI). Order = [theta1, theta2] ---
MOTOR_IDS = [0x01, 0x02]                  # <-- motor 1, motor 2 send IDs

# --- MIT scaling ranges: READ OFF THE DAMIAO GUI (PMAX / VMAX / TMAX) ---
P_MIN, P_MAX = -12.5, 12.5    # rad     <-- replace with your motor's PMAX
V_MIN, V_MAX = -30.0, 30.0    # rad/s   <-- replace with your motor's VMAX
T_MIN, T_MAX =  -5.0,  5.0    # N*m     <-- replace with your motor's TMAX
KP_MAX = 500.0                # fixed by firmware
KD_MAX = 500.0                # fixed by firmware

# --- motor zero / direction calibration (maps raw motor <-> model angle) ---
MOTOR_DIR    = [1, 1]         # +1 or -1 per motor; flip if model angle grows the wrong way
MOTOR_OFFSET = [0.0, 0.0]     # rad added after direction, so raw-home reads as model theta=0

# --- safety ---
TAU_LIMIT   = 0.5    # N*m hard clamp here too (independent of MATLAB). <= T_MAX
CMD_TIMEOUT = 0.2    # s: if no LCM command arrives, command zero torque

CTRL_TOPIC  = "rsrr_arm_control"
STATE_TOPIC = "rsrr_arm_state"
# ======================================================================


# --------------------- MIT-mode pack / unpack -------------------------
def _f2u(x, lo, hi, bits):
    x = max(min(x, hi), lo)
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))

def _u2f(i, lo, hi, bits):
    return i * (hi - lo) / ((1 << bits) - 1) + lo

def pack_cmd(pos, vel, kp, kd, tau):
    """Damiao MIT command: 8 bytes = p(16) v(12) kp(12) kd(12) t(12)."""
    p = _f2u(pos, P_MIN, P_MAX, 16)
    v = _f2u(vel, V_MIN, V_MAX, 12)
    kpi = _f2u(kp, 0.0, KP_MAX, 12)
    kdi = _f2u(kd, 0.0, KD_MAX, 12)
    t = _f2u(tau, T_MIN, T_MAX, 12)
    return bytes([
        (p >> 8) & 0xFF,
        p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kpi >> 8) & 0xF),
        kpi & 0xFF,
        (kdi >> 4) & 0xFF,
        ((kdi & 0xF) << 4) | ((t >> 8) & 0xF),
        t & 0xFF,
    ])

def parse_fb(data):
    """Damiao MIT feedback -> (motor_id, pos_rad, vel_rads, tau_Nm)."""
    mid = data[0] & 0x0F
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0xF) << 8) | data[5]
    return (mid,
            _u2f(p, P_MIN, P_MAX, 16),
            _u2f(v, V_MIN, V_MAX, 12),
            _u2f(t, T_MIN, T_MAX, 12))

# Damiao special command frames (8 data bytes to the motor's ID)
ENABLE  = bytes([0xFF] * 7 + [0xFC])
DISABLE = bytes([0xFF] * 7 + [0xFD])
SETZERO = bytes([0xFF] * 7 + [0xFE])   # sets current pose as zero -- use with care


class ArmCanBridge:
    def __init__(self):
        self.bus = can.interface.Bus(bustype=CAN_BUSTYPE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE)
        self.lc = lcm.LCM()
        self.tau_cmd = [0.0, 0.0]        # latest commanded torque per motor (model convention)
        self.last_cmd_t = 0.0
        self.pos = [0.0] * 8             # model-convention angles (idx 0,1 = motors)
        self.vel = [0.0] * 8
        self.lock = threading.Lock()
        self.running = True
        self.lc.subscribe(CTRL_TOPIC, self._on_cmd)

    def _on_cmd(self, channel, data):
        msg = rsrr_arm_control_t.decode(data)
        with self.lock:
            self.tau_cmd = [max(min(msg.qj_tau[i], TAU_LIMIT), -TAU_LIMIT) for i in range(2)]
            self.last_cmd_t = time.time()

    def _send(self, mid, payload):
        self.bus.send(can.Message(arbitration_id=mid, data=payload, is_extended_id=False))

    def enable(self):
        for mid in MOTOR_IDS:
            self._send(mid, ENABLE); time.sleep(0.003)

    def disable(self):
        for mid in MOTOR_IDS:
            self._send(mid, DISABLE); time.sleep(0.003)

    def _lcm_loop(self):
        while self.running:
            self.lc.handle_timeout(10)

    def run(self):
        print("Enabling motors..."); self.enable()
        threading.Thread(target=self._lcm_loop, daemon=True).start()
        print(f"Bridge up: {CTRL_TOPIC} -> CAN, CAN -> {STATE_TOPIC}. Ctrl-C to stop.")
        try:
            while True:
                # watchdog: stale command -> zero torque
                with self.lock:
                    if time.time() - self.last_cmd_t > CMD_TIMEOUT:
                        self.tau_cmd = [0.0, 0.0]
                    tau = list(self.tau_cmd)

                # pure-torque command to each motor (kp=kd=0), then read its reply
                for k, mid in enumerate(MOTOR_IDS):
                    tau_raw = MOTOR_DIR[k] * tau[k]           # model -> raw convention
                    self._send(mid, pack_cmd(0.0, 0.0, 0.0, 0.0, tau_raw))
                    rx = self.bus.recv(timeout=0.005)
                    if rx is not None:
                        fid, p, v, _t = parse_fb(rx.data)
                        if fid in MOTOR_IDS:
                            j = MOTOR_IDS.index(fid)
                            self.pos[j] = MOTOR_DIR[j] * p + MOTOR_OFFSET[j]   # raw -> model
                            self.vel[j] = MOTOR_DIR[j] * v

                # publish state (only the 2 motors matter to the controller)
                st = rsrr_arm_state_t()
                st.timestamp = time.time_ns()     # ns; controller does *1e-9
                st.qj_pos = list(self.pos)
                st.qj_vel = list(self.vel)
                st.qj_tau = [0.0] * 8
                st.ft_sensor = [0.0] * 6
                self.lc.publish(STATE_TOPIC, st.encode())

                time.sleep(0.002)                 # ~500 Hz service loop
        except KeyboardInterrupt:
            print("\nCtrl-C -> zero torque + disable")
        finally:
            self.running = False
            for mid in MOTOR_IDS:
                self._send(mid, pack_cmd(0, 0, 0, 0, 0))
            self.disable()
            self.bus.shutdown()


if __name__ == "__main__":
    ArmCanBridge().run()
