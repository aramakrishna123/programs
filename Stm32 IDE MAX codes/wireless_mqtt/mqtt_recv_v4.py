#!/usr/bin/env python3
"""
TORUS SLAVE  (runs on the PC connected to the axis boards)

Receives joystick values over MQTT and writes each one, as a plain number,
to its axis board's serial port. Also reads back whatever each board prints
on that same port (telemetry, status lines, errors) and shows it live in a
single dashboard that updates in place.

Each port is independent: connect one, some, or all -- missing ones are
skipped and retried in the background without blocking the others.

    python3 torus_slave.py

Needs:  pip install paho-mqtt pyserial rich
"""
import re
import threading
import time

import serial
import paho.mqtt.client as mqtt
from rich.live import Live
from rich.table import Table

# ============================ SETTINGS (edit here) ============================
BROKER_HOST = "192.168.1.15"     # IP of the PC running the MQTT broker
BROKER_PORT = 1883

# One entry per board. "x", "y", "z" receive the joystick value from MQTT;
# "j" receives the joint-angle line "<q1>$<q3>$<q5>" (servo STM32). Any
# other name is monitor-only -- its raw output is shown but nothing is written.
PORTS = {
    "x": "COM10",
    "y": "COM15",
    "z": "COM57",
    "j": "COM46",     # joint-angle (servo) board
}
BAUD = 115200
# Per-port override. The servo board's firmware calls Serial.begin(250000);
# harmless if it is USB-CDC (baud ignored), required if it is a real UART.
PORT_BAUD = {"j": 250000}

# X axis motor is driven by three ASCII commands instead of a raw number:
#   "F\n" = forward, "B\n" = backward, "S\n" = stop.
FBS_AXES   = {"x"}
FBS_BUFFER = 250
# ==============================================================================

TOPIC_JOY    = "torus/joy"
TOPIC_STATUS = "torus/joy/status"
TOPIC_JNT    = "torus/joints"
TOPIC_RX     = "torus/rx"   # + "/<name>" per port -- so the master can show it too
WRITE_AXES   = {"x", "y", "z"}   # names that receive a value from the MQTT payload
JOINT_PORT   = "j"               # receives the joint-angle line
JNT_RE = re.compile(r"^\d{1,4}\$\d{1,4}\$\d{1,4}$")
STOP = 2048
RETRY_S = 2.0


class Port:
    """One serial port: writes commands to it (if it's a write axis) and
    continuously reads back whatever the board prints, for the dashboard."""

    def __init__(self, name, port):
        self.name, self.port = name, port
        self.ser = None
        self.next_try = 0.0
        self.warned = False
        self.last_rx = ""          # most recent raw line read back from the board
        self.rx_at = 0.0
        self.client = None         # set once the MQTT client exists, so RX can be relayed
        self.events = []           # recent connect/lost/etc. messages, newest last

    def start(self):
        threading.Thread(target=self._reader_loop, daemon=True, name=f"rx-{self.name}").start()

    def _note(self, msg):
        self.events.append(msg)
        del self.events[:-3]       # keep only the last few per port

    def _open(self):
        if time.time() < self.next_try:
            return
        self.next_try = time.time() + RETRY_S
        try:
            self.ser = serial.Serial(self.port, PORT_BAUD.get(self.name, BAUD), timeout=0.3, write_timeout=0.2)
            self._note("connected")
            self.warned = False
        except (serial.SerialException, OSError):
            self.ser = None
            if not self.warned:
                self._note("not available -- retrying")
                self.warned = True

    def _reader_loop(self):
        while True:
            if self.ser is None:
                self._open()
                time.sleep(0.1)
                continue
            try:
                line = self.ser.readline()
            except (serial.SerialException, OSError):
                self._note("lost")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                continue
            if line:
                self.last_rx = line.decode(errors="replace").strip()
                self.rx_at = time.time()
                if self.client is not None:
                    # best effort -- a dropped telemetry packet doesn't matter,
                    # so this never blocks or retries
                    self.client.publish(f"{TOPIC_RX}/{self.name}", self.last_rx, qos=0)

    def write_line(self, text):
        """Write a ready-made text line (used for the joint-angle port)."""
        if self.ser is None:
            return False
        try:
            self.ser.write(text.encode() + b"\n")
            return True
        except (serial.SerialException, OSError, serial.SerialTimeoutException):
            return False

    def write(self, value):
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
            return False


ports = {n: Port(n, p) for n, p in PORTS.items()}
count = {n: 0 for n in WRITE_AXES}
latest = {n: None for n in WRITE_AXES}
latest_joint = None
rate = 0
gap_ms = 0.0
max_gap_ms = 0.0
last_msg_ever = 0.0
mqtt_state = "connecting..."
master_state = "unknown"


def build_table():
    """One dashboard, rebuilt each refresh and handed to the Live display,
    which does the in-place redraw -- no manual clear-screen, no scrolling."""
    table = Table(title=f"TORUS SLAVE  |  mqtt: {mqtt_state}  |  master: {master_state}  |  "
                         f"{rate} msg/s  gap {gap_ms:.0f}ms (max {max_gap_ms:.0f}ms)")
    table.add_column("Axis")
    table.add_column("Port")
    table.add_column("Status")
    table.add_column("TX")
    table.add_column("RX (raw from board)")
    table.add_column("Age")
    table.add_column("Last events")

    for n, p in ports.items():
        status = "[green]ok[/]" if p.ser is not None else "[red]MISSING[/]"
        if n == JOINT_PORT:
            tx = latest_joint or "----"
        elif n not in WRITE_AXES:
            tx = "--"
        else:
            tx = "----" if latest[n] is None else str(latest[n])
        age = "--" if not p.rx_at else f"{time.time() - p.rx_at:.1f}s"
        table.add_row(n.upper(), p.port, status, tx, p.last_rx or "(nothing yet)", age,
                      " / ".join(p.events) or "--")
    return table


def stop_all():
    for n in WRITE_AXES:
        if n in ports:
            ports[n].write(STOP)


def on_message(client, userdata, msg):
    global last_msg_ever, gap_ms, max_gap_ms, master_state, latest_joint
    text = msg.payload.decode(errors="ignore").strip()

    if msg.topic == TOPIC_JNT:
        # Joint angles are absolute position targets, so there is deliberately
        # no "stop" for them -- on master loss the servos just hold position.
        if JNT_RE.match(text):
            latest_joint = text
            if JOINT_PORT in ports:
                ports[JOINT_PORT].write_line(text)
        return

    if msg.topic == TOPIC_STATUS:
        master_state = text
        if text == "offline":
            stop_all()
        return

    try:
        values = [int(v) for v in text.split(",")]
        assert len(values) == 3 and all(0 <= v <= 4095 for v in values)
    except Exception:
        return

    now = time.time()
    if last_msg_ever:
        gap_ms = (now - last_msg_ever) * 1000.0
        max_gap_ms = max(max_gap_ms, gap_ms)
    last_msg_ever = now

    for axis, v in zip(("x", "y", "z"), values):
        latest[axis] = v
        count[axis] += 1
        if axis in ports:
            ports[axis].write(v)


def on_connect(client, userdata, *args):
    global mqtt_state
    mqtt_state = "connected"
    client.subscribe([(TOPIC_JOY, 0), (TOPIC_STATUS, 1), (TOPIC_JNT, 0)])


def on_disconnect(client, userdata, *args):
    global mqtt_state
    mqtt_state = "disconnected -- retrying"
    stop_all()


def main():
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="torus-slave")
    except AttributeError:
        client = mqtt.Client(client_id="torus-slave")
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.reconnect_delay_set(1, 5)

    for p in ports.values():
        p.client = client   # now the reader threads can relay what they read
        p.start()

    client.connect_async(BROKER_HOST, BROKER_PORT, keepalive=10)
    client.loop_start()

    global rate, max_gap_ms
    t0 = time.time()
    try:
        with Live(build_table(), refresh_per_second=10, screen=False) as live:
            while True:
                time.sleep(0.1)
                if time.time() - t0 >= 1.0:
                    rate = count["x"]
                    for n in count:
                        count[n] = 0
                    t0 = time.time()
                    max_gap_ms = 0.0
                live.update(build_table())
    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()