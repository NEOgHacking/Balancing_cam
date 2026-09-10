#!/usr/bin/env python3
import json
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import paho.mqtt.client as mqtt

BROKER = os.environ.get("MQTT_BROKER", "localhost")
PORT = int(os.environ.get("MQTT_PORT", "1883"))
STATUS_TOPIC = "robots/+/status"
ALL_COMMAND_TOPIC = "robots/all/command"
TOPIC_TEMPLATE = "robots/{}/command"
ONLINE_BG = "#d0f8d0"
OFFLINE_TIMEOUT = 3

update_queue = queue.Queue()
robot_frames = {}

class RobotFrame(tk.Frame):
    def __init__(self, parent, robot_name):
        super().__init__(parent, bd=1, relief="ridge", padx=6, pady=6)
        self.robot_name = robot_name
        self.columnconfigure(1, weight=1)
        self.last_seen = 0
        self.default_bg = self.cget("bg")

        self.title = tk.Label(self, text=robot_name, font=(None, 12, "bold"), fg="black", bg=self.default_bg)
        self.title.grid(row=0, column=0, columnspan=3, sticky="w")

        tk.Label(self, text="Battery:", bg=self.default_bg).grid(row=1, column=0, sticky="w")
        self.battery = tk.Label(self, text="--", bg=self.default_bg)
        self.battery.grid(row=1, column=1, sticky="w")

        tk.Label(self, text="Angle:", bg=self.default_bg).grid(row=2, column=0, sticky="w")
        self.angle = tk.Label(self, text="--", bg=self.default_bg)
        self.angle.grid(row=2, column=1, sticky="w")

        tk.Label(self, text="State:", bg=self.default_bg).grid(row=3, column=0, sticky="w")
        self.state = tk.Label(self, text="--", bg=self.default_bg)
        self.state.grid(row=3, column=1, sticky="w")

        tk.Label(self, text="Mode:", bg=self.default_bg).grid(row=4, column=0, sticky="w")
        self.mode = tk.Label(self, text="--", bg=self.default_bg)
        self.mode.grid(row=4, column=1, sticky="w")

        self.stop_button = ttk.Button(self, text="Stop", command=self.stop_robot)
        self.stop_button.grid(row=5, column=0, sticky="ew", pady=(8, 0))

        self.restart_button = ttk.Button(self, text="Restart", command=self.restart_robot)
        self.restart_button.grid(row=5, column=1, sticky="ew", pady=(8, 0))

        self.store_button = ttk.Button(self, text="Store", command=self.store_pids)

        # PID angle controller sliders (use same ranges as web interface)
        # Put them in an internal frame so they can be hidden/shown as a group
        self.pid_frame = tk.Frame(self, bg=self.default_bg)
        self.pid_frame.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(8,0))
        self.pid_frame.columnconfigure(1, weight=1)

        # Proportional gain: c1p min=0.1 max=3 step=0.01
        tk.Label(self.pid_frame, text="P (angle):", bg=self.default_bg).grid(row=0, column=0, sticky="w")
        self.c1p = tk.Scale(self.pid_frame, from_=0.1, to=3.0, resolution=0.01, orient="horizontal", length=200,
                            command=lambda v: self._on_pid_change('c1p', float(v), fmt="{:.2f}"))
        self.c1p.set(0.65)
        self.c1p.grid(row=0, column=1, sticky="ew")

        tk.Label(self.pid_frame, text="I (angle):", bg=self.default_bg).grid(row=1, column=0, sticky="w")
        # Integral gain: c1i min=0 max=1 step=0.01
        self.c1i = tk.Scale(self.pid_frame, from_=0.0, to=1.0, resolution=0.01, orient="horizontal", length=200,
                            command=lambda v: self._on_pid_change('c1i', float(v), fmt="{:.2f}"))
        self.c1i.set(1.00)
        self.c1i.grid(row=1, column=1, sticky="ew")

        tk.Label(self.pid_frame, text="D (angle):", bg=self.default_bg).grid(row=2, column=0, sticky="w")
        # Derivative gain: c1d min=0 max=0.3 step=0.001
        self.c1d = tk.Scale(self.pid_frame, from_=0.0, to=0.3, resolution=0.001, orient="horizontal", length=200,
                            command=lambda v: self._on_pid_change('c1d', float(v), fmt="{:.3f}"))
        self.c1d.set(0.075)
        self.c1d.grid(row=2, column=1, sticky="ew")

        tk.Label(self.pid_frame, text="Max angle (deg):", bg=self.default_bg).grid(row=3, column=0, sticky="w")
        # Maximum angle: c1m min=0 max=50 step=0.1
        self.c1m = tk.Scale(self.pid_frame, from_=0.0, to=50.0, resolution=0.1, orient="horizontal", length=200,
                            command=lambda v: self._on_pid_change('c1m', float(v), fmt="{:.1f}"))
        self.c1m.set(15.0)
        self.c1m.grid(row=3, column=1, sticky="ew")

        # Toggle button to show/hide controller sliders
        self.pid_visible = True
        self.toggle_button = ttk.Button(self, text="Hide controller settings", command=self._toggle_pid_visibility)
        self.toggle_button.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(8,0))

        self.store_button.grid(row=7, column=2, sticky="ew", padx=(8,0), pady=(8,0))

    def _on_pid_change(self, cmd_id, value, fmt="{:.2f}"):
        # Publish command in the same format as the web UI: e.g. "c1p0.65x"
        try:
            # Suppress publishes when sliders are being programmatically updated from robot status
            if getattr(self, 'suppress_pid_callbacks', False):
                return
            if cmd_id.startswith('c1'):
                payload = f"{cmd_id}{fmt.format(value)}x"
            else:
                payload = f"{cmd_id}{value}x"
            client.publish(TOPIC_TEMPLATE.format(self.robot_name), payload)
        except Exception as exc:
            print(f"Failed publishing PID change {cmd_id} -> {value}: {exc}")

    def store_pids(self):
        # Publish a simple store command that the robot recognizes to write current PID params to EEPROM
        try:
            client.publish(TOPIC_TEMPLATE.format(self.robot_name), "store_pids")
        except Exception as exc:
            print(f"Failed publishing store command: {exc}")

    def _toggle_pid_visibility(self):
        if self.pid_visible:
            self.pid_frame.grid_remove()
            self.store_button.grid_remove()
            self.toggle_button.configure(text="Show controller settings")
            self.pid_visible = False
        else:
            self.pid_frame.grid()
            self.store_button.grid()
            self.toggle_button.configure(text="Hide controller settings")
            self.pid_visible = True

    def update_status(self, status):
        self.battery.configure(text=f"{status.get('battery', 0):.2f} V")
        self.angle.configure(text=f"{status.get('angle', 0):.2f}°")
        self.state.configure(text=status.get('state', 'unknown'))
        self.mode.configure(text=status.get('mode', 'unknown'))
        self.last_seen = time.time()
        self.set_online(True)

        # Update PID sliders from robot-reported values if present. Suppress publishing while updating.
        try:
            self.suppress_pid_callbacks = True
            if 'c1p' in status:
                try:
                    self.c1p.set(float(status['c1p']))
                except Exception:
                    pass
            if 'c1i' in status:
                try:
                    self.c1i.set(float(status['c1i']))
                except Exception:
                    pass
            if 'c1d' in status:
                try:
                    self.c1d.set(float(status['c1d']))
                except Exception:
                    pass
            if 'c1m' in status:
                try:
                    self.c1m.set(float(status['c1m']))
                except Exception:
                    pass
        finally:
            self.suppress_pid_callbacks = False

    def set_online(self, online):
        bg = ONLINE_BG if online else self.default_bg
        fg = "green" if online else "black"
        self.configure(bg=bg)
        for child in self.winfo_children():
            if isinstance(child, tk.Label):
                child.configure(bg=bg)
        self.title.configure(fg=fg)

    def check_online(self):
        if time.time() - self.last_seen > OFFLINE_TIMEOUT:
            self.set_online(False)

    def stop_robot(self):
        client.publish(TOPIC_TEMPLATE.format(self.robot_name), "stop")

    def restart_robot(self):
        client.publish(TOPIC_TEMPLATE.format(self.robot_name), "selfright")


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT broker {BROKER}:{PORT}")
        client.subscribe(STATUS_TOPIC)
    else:
        print(f"MQTT connection failed, rc={rc}")


def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode('utf-8')
        status = json.loads(payload)
        robot_name = status.get('name')
        if not robot_name:
            return
        update_queue.put((robot_name, status))
    except Exception as exc:
        print(f"Failed to parse message {msg.topic}: {exc}")


def maintain_frames():
    while not update_queue.empty():
        robot_name, status = update_queue.get()
        if robot_name not in robot_frames:
            frame = RobotFrame(robot_list_frame, robot_name)
            frame.grid(sticky="ew", padx=4, pady=4)
            robot_frames[robot_name] = frame
        robot_frames[robot_name].update_status(status)

    for frame in robot_frames.values():
        frame.check_online()

    root.after(100, maintain_frames)


def stop_all():
    client.publish(ALL_COMMAND_TOPIC, "stop")


def clear_robot_frames():
    global robot_frames
    for frame in robot_frames.values():
        frame.destroy()
    robot_frames = {}


def build_gui():
    global root, robot_list_frame
    root = tk.Tk()
    root.title("Robot MQTT Dashboard")
    root.geometry("520x420")

    toolbar = ttk.Frame(root, padding="10 10 10 10")
    toolbar.pack(fill="x")

    stop_all_button = ttk.Button(toolbar, text="Stop All Robots", command=stop_all)
    stop_all_button.pack(side="left")

    clear_robots_button = ttk.Button(toolbar, text="Clear Robots", command=clear_robot_frames)
    clear_robots_button.pack(side="left", padx=(8, 0))

    separator = ttk.Separator(root, orient="horizontal")
    separator.pack(fill="x", pady=8)

    robot_list_frame = ttk.Frame(root, padding="10 10 10 10")
    robot_list_frame.pack(fill="both", expand=True)
    robot_list_frame.columnconfigure(0, weight=1)

    root.after(100, maintain_frames)
    return root


def start_mqtt():
    global client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, 60)
    client.loop_start()


if __name__ == "__main__":
    build_gui()
    start_mqtt()
    root.mainloop()
