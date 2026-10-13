#!/usr/bin/env python3
"""
TORUS RECEIVER  (runs on the PC connected to the X / Y / Z axis boards)

Gets the joystick values over MQTT and writes each one to its axis board.
Each COM port is independent: connect one, two or all three -- whatever is
plugged in gets its data, missing ones are simply skipped and retried.

    python3 torus_receiver.py

Needs:  pip install paho-mqtt pyserial
"""
import time

import serial
import paho.mqtt.client as mqtt

# ============================ SETTINGS (edit here) ============================
BROKER_HOST = "192.168.1.23"     # IP of the PC running Mosquitto
BROKER_PORT = 1883

# Serial port of each axis board (Windows: "COM5" ...). Leave a board's port
# as is even if it is not connected right now -- it is retried automatically.
PORTS = {
    "x": "COM8",
    "y": "COM9",
    "z": "COM3",
}
BAUD = 115200

# X axis motor is driven by three ASCII commands instead of a raw number:
#   "F\n" = forward, "B\n" = backward, "S\n" = stop.
# STOP (2048) +/- FBS_BUFFER is the dead zone that maps to "S".
FBS_AXES   = {"x"}
FBS_BUFFER = 250

# There is no "stop if no message for N ms" timer here on purpose: matching
# any fixed timeout to a real, jittery data rate caused the motors to be
# stopped between normal updates ("toggling"). Axes are still stopped the
# moment the link actually drops -- see the sender's MQTT "last will" message
# (torus/joy/status = "offline") and on_disconnect() below, both of which
# fire on the real event rather than a guessed delay.
# ==============================================================================

TOPIC_JOY    = "torus/joy"
TOPIC_STATUS = "torus/joy/status"
STOP = 2048
RETRY_S = 2.0


class Axis:
    """One axis board on one serial port. Never raises; never blocks the others."""

    def __init__(self, name, port):
        self.name, self.port = name, port
        self.ser = None
        self.next_try = 0.0
        self.warned = False

    def _open(self):
        if time.time() < self.next_try:
            return
        self.next_try = time.time() + RETRY_S
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0, write_timeout=0.2)
            log(f"[{self.name}] {self.port} connected")
            self.warned = False
        except (serial.SerialException, OSError) as e:
            self.ser = None
            if not self.warned:
                log(f"[{self.name}] {self.port} not available -- skipping, will retry")
                self.warned = True

    def write(self, value):
        if self.ser is None:
            self._open()
            if self.ser is None:
                return False
        if self.name in FBS_AXES:
            if value > STOP + FBS_BUFFER:
                cmd = b"F\n"
            elif value < STOP - FBS_BUFFER:
                cmd = b"B\n"
            else:
                cmd = b"S\n"
        else:
            cmd = b"%d\n" % value
        try:
            self.ser.write(cmd)
            return True
        except (serial.SerialException, OSError, serial.SerialTimeoutException):
            log(f"[{self.name}] {self.port} lost")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return False


axes = {n: Axis(n, p) for n, p in PORTS.items()}
count = {n: 0 for n in "xyz"}
latest = {n: None for n in "xyz"}   # most recent value received per axis
rate = 0                            # messages per second (updated each second)
gap_ms = 0.0                        # time since the previous message, in ms (display only)
max_gap_ms = 0.0                    # worst gap seen in the last reporting second (display only)
_line_width = 0


def log(msg):
    """Print an event line without leaving pieces of the live display behind."""
    global _line_width
    print("\r" + " " * _line_width + "\r" + msg, flush=True)


def show():
    """Redraw the single live status line."""
    global _line_width
    parts = []
    for n in "xyz":
        if latest[n] is None:
            v = "----"
        elif n in FBS_AXES:
            if latest[n] > STOP + FBS_BUFFER:
                v = "F   "
            elif latest[n] < STOP - FBS_BUFFER:
                v = "B   "
            else:
                v = "S   "
            v += f"({latest[n]:4d})"
        else:
            v = f"{latest[n]:4d}"
        if n not in axes:
            port = "no port set"
        else:
            port = "port ok" if axes[n].ser is not None else "port MISSING"
        parts.append(f"{n.upper()}: {v} ({port})")
    age = "no data yet" if not last_msg_ever else f"{time.time() - last_msg_ever:4.1f}s ago"
    line = "  |  ".join(parts) + f"  |  {rate:3d} msg/s  |  gap {gap_ms:4.0f}ms (max {max_gap_ms:4.0f}ms)  |  last {age}"
    _line_width = max(_line_width, len(line))
    print("\r" + line.ljust(_line_width), end="", flush=True)


last_msg_ever = 0.0


def stop_all():
    for a in axes.values():
        a.write(STOP)


def on_message(client, userdata, msg):
    global last_msg_ever, gap_ms, max_gap_ms
    text = msg.payload.decode(errors="ignore").strip()

    if msg.topic == TOPIC_STATUS:
        log(f"[sender] {text}")
        if text == "offline":
            stop_all()          # sender crashed / lost link -> stop everything
        return

    try:
        values = [int(v) for v in text.split(",")]
        assert len(values) == 3 and all(0 <= v <= 4095 for v in values)
    except Exception:
        return                  # ignore malformed packet
    now = time.time()
    if last_msg_ever:
        gap_ms = (now - last_msg_ever) * 1000.0
        max_gap_ms = max(max_gap_ms, gap_ms)
    last_msg_ever = now
    for axis, v in zip("xyz", values):
        latest[axis] = v                      # shown even if that axis' port is missing
        count[axis] += 1
        if axis in axes:
            axes[axis].write(v)


def on_connect(client, userdata, *args):
    log("[mqtt] connected")
    client.subscribe([(TOPIC_JOY, 0), (TOPIC_STATUS, 1)])


def on_disconnect(client, userdata, *args):
    log("[mqtt] disconnected -- stopping axes, retrying")
    stop_all()


def main():
    try:  # paho-mqtt 2.x
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="torus-receiver")
    except AttributeError:  # paho-mqtt 1.x
        client = mqtt.Client(client_id="torus-receiver")
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.reconnect_delay_set(1, 5)

    for a in axes.values():
        a._open()
    log(f"[mqtt] connecting to {BROKER_HOST}:{BROKER_PORT} ...")
    client.connect_async(BROKER_HOST, BROKER_PORT, keepalive=10)
    client.loop_start()

    global rate
    t0 = time.time()
    try:
        while True:
            time.sleep(0.1)
            global max_gap_ms
            if time.time() - t0 >= 1.0:
                rate = count["x"]
                for n in count:
                    count[n] = 0
                t0 = time.time()
                max_gap_ms = 0.0   # start a fresh worst-case window
            show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()