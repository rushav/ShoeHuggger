#!/usr/bin/env python3
"""
AlienBot Control UI
===================
Launches alongside RViz. Manages robot modes via subprocess.

Buttons:
  1. Teleop    — launches teleop_hold for manual control
  2. Annoy     — launches person_follower (chase + flap)
  3. Follow    — placeholder for calm follow mode
  4. STOP ALL  — kills all modes, zeros motors and wings

Usage:
    python3 alienbot_ui.py
    OR
    ros2 run alienbot_vision alienbot_ui
"""

import tkinter as tk
from tkinter import font as tkfont
import subprocess
import signal
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class AlienBotUI:
    def __init__(self):
        # ── ROS2 setup for emergency stop ─────────────────────────────────
        rclpy.init()
        self.ros_node = rclpy.create_node('alienbot_ui')
        self.cmd_pub = self.ros_node.create_publisher(Twist, '/cmd_vel', 10)
        self.flap_pub = self.ros_node.create_publisher(Float64, '/wing_flap_speed', 10)

        # Spin ROS in background thread
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()

        # ── Process management ────────────────────────────────────────────
        self.active_process = None
        self.active_mode = None

        # ── Build UI ──────────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title('AlienBot Control')
        self.root.geometry('400x600')
        self.root.configure(bg='#1a1a2e')
        self.root.resizable(False, False)

        # Fonts
        title_font = tkfont.Font(family='Helvetica', size=24, weight='bold')
        btn_font = tkfont.Font(family='Helvetica', size=16, weight='bold')
        status_font = tkfont.Font(family='Helvetica', size=12)
        info_font = tkfont.Font(family='Helvetica', size=10)

        # Title
        tk.Label(
            self.root, text='AlienBot', font=title_font,
            fg='#e94560', bg='#1a1a2e'
        ).pack(pady=(20, 5))

        tk.Label(
            self.root, text='Control Panel', font=status_font,
            fg='#888888', bg='#1a1a2e'
        ).pack(pady=(0, 20))

        # Status indicator
        self.status_frame = tk.Frame(self.root, bg='#1a1a2e')
        self.status_frame.pack(pady=(0, 20))

        self.status_dot = tk.Canvas(
            self.status_frame, width=16, height=16,
            bg='#1a1a2e', highlightthickness=0
        )
        self.status_dot.pack(side=tk.LEFT, padx=(0, 8))
        self.status_circle = self.status_dot.create_oval(2, 2, 14, 14, fill='#555555')

        self.status_label = tk.Label(
            self.status_frame, text='IDLE', font=status_font,
            fg='#888888', bg='#1a1a2e'
        )
        self.status_label.pack(side=tk.LEFT)

        # ── Buttons ───────────────────────────────────────────────────────
        btn_frame = tk.Frame(self.root, bg='#1a1a2e')
        btn_frame.pack(expand=True, fill=tk.BOTH, padx=30)

        # Button 1: Teleop
        self.btn_teleop = tk.Button(
            btn_frame, text='1  TELEOP', font=btn_font,
            fg='white', bg='#0f3460', activebackground='#16213e',
            activeforeground='white', relief=tk.FLAT,
            cursor='hand2', command=self.start_teleop,
            height=2
        )
        self.btn_teleop.pack(fill=tk.X, pady=5)

        tk.Label(
            btn_frame, text='Manual control with arrow keys',
            font=info_font, fg='#555555', bg='#1a1a2e'
        ).pack(pady=(0, 10))

        # Button 2: Annoy Mode
        self.btn_annoy = tk.Button(
            btn_frame, text='2  ANNOY MODE', font=btn_font,
            fg='white', bg='#e94560', activebackground='#c73a52',
            activeforeground='white', relief=tk.FLAT,
            cursor='hand2', command=self.start_annoy,
            height=2
        )
        self.btn_annoy.pack(fill=tk.X, pady=5)

        tk.Label(
            btn_frame, text='Chase person + flap wings aggressively',
            font=info_font, fg='#555555', bg='#1a1a2e'
        ).pack(pady=(0, 10))

        # Button 3: Follow Mode
        self.btn_follow = tk.Button(
            btn_frame, text='3  FOLLOW MODE', font=btn_font,
            fg='white', bg='#533483', activebackground='#442b6e',
            activeforeground='white', relief=tk.FLAT,
            cursor='hand2', command=self.start_follow,
            height=2
        )
        self.btn_follow.pack(fill=tk.X, pady=5)

        tk.Label(
            btn_frame, text='Calm follow from distance (coming soon)',
            font=info_font, fg='#555555', bg='#1a1a2e'
        ).pack(pady=(0, 10))

        # Button 4: STOP ALL
        self.btn_stop = tk.Button(
            btn_frame, text='4  STOP ALL', font=btn_font,
            fg='white', bg='#cc0000', activebackground='#990000',
            activeforeground='white', relief=tk.FLAT,
            cursor='hand2', command=self.stop_all,
            height=2
        )
        self.btn_stop.pack(fill=tk.X, pady=5)

        tk.Label(
            btn_frame, text='Emergency stop — kills all modes + motors',
            font=info_font, fg='#555555', bg='#1a1a2e'
        ).pack(pady=(0, 10))

        # ── Keyboard shortcuts ────────────────────────────────────────────
        self.root.bind('1', lambda e: self.start_teleop())
        self.root.bind('2', lambda e: self.start_annoy())
        self.root.bind('3', lambda e: self.start_follow())
        self.root.bind('4', lambda e: self.stop_all())
        self.root.bind('<Escape>', lambda e: self.stop_all())
        self.root.bind('<space>', lambda e: self.stop_all())

        # Handle Ctrl+C in terminal
        signal.signal(signal.SIGINT, lambda s, f: self.on_close())

        # Clean exit on window close
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

    # ══════════════════════════════════════════════════════════════════════
    # ROS2
    # ══════════════════════════════════════════════════════════════════════

    def _ros_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    def _publish_stop(self):
        """Publish zero velocity and stop wing flapping."""
        for _ in range(10):
            self.cmd_pub.publish(Twist())
            self.flap_pub.publish(Float64(data=0.0))
            time.sleep(0.02)

    # ══════════════════════════════════════════════════════════════════════
    # PROCESS MANAGEMENT
    # ══════════════════════════════════════════════════════════════════════

    def _kill_active(self):
        """Kill the currently running mode process."""
        if self.active_process is not None:
            try:
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGTERM)
                self.active_process.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self.active_process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            self.active_process = None

        # Also kill any straggler processes by name
        subprocess.run('pkill -f person_follower 2>/dev/null', shell=True)
        subprocess.run('pkill -f teleop_hold 2>/dev/null', shell=True)

        # Always publish stop after killing a mode
        self._publish_stop()

    def _launch_mode(self, name, cmd):
        """Kill any active mode, then launch a new one."""
        # Don't restart the same mode
        if self.active_mode == name:
            return

        self._kill_active()
        time.sleep(0.3)  # brief pause to let ports settle

        try:
            self.active_process = subprocess.Popen(
                cmd,
                shell=True,
                preexec_fn=os.setsid,  # create process group for clean kill
            )
            self.active_mode = name
            self._update_status(name)
        except Exception as e:
            self._update_status(f'ERROR: {e}')

    def _update_status(self, mode):
        """Update status label and dot color."""
        colors = {
            'TELEOP': '#0f3460',
            'ANNOY': '#e94560',
            'FOLLOW': '#533483',
            'IDLE': '#555555',
        }
        color = colors.get(mode, '#555555')
        self.status_dot.itemconfig(self.status_circle, fill=color)
        self.status_label.config(text=mode, fg=color)

        # Highlight active button, dim others
        self.btn_teleop.config(relief=tk.SUNKEN if mode == 'TELEOP' else tk.FLAT)
        self.btn_annoy.config(relief=tk.SUNKEN if mode == 'ANNOY' else tk.FLAT)
        self.btn_follow.config(relief=tk.SUNKEN if mode == 'FOLLOW' else tk.FLAT)

    # ══════════════════════════════════════════════════════════════════════
    # MODE LAUNCHERS
    # ══════════════════════════════════════════════════════════════════════

    def start_teleop(self):
        """Launch teleop_hold for manual arrow-key control."""
        # Teleop needs a real terminal for key input — launch in xterm
        self._launch_mode(
            'TELEOP',
            'xterm -e bash -c "source /opt/ros/humble/setup.bash && '
            'source ~/turtlebot4_ws/install/setup.bash && '
            'export ROS_DOMAIN_ID=117 && '
            'ros2 run alienbot_vision teleop_hold"'
        )

    def start_annoy(self):
        """Launch person_follower in annoy mode (chase + aggressive flap)."""
        self._launch_mode(
            'ANNOY',
            'bash -c "source /opt/ros/humble/setup.bash && '
            'source ~/turtlebot4_ws/install/setup.bash && '
            'export ROS_DOMAIN_ID=117 && '
            'ros2 run alienbot_vision person_follower"'
        )

    def start_follow(self):
        """Launch calm follow mode — follows from distance, flaps when stopped."""
        self._launch_mode(
            'FOLLOW',
            'bash -c "source /opt/ros/humble/setup.bash && '
            'source ~/turtlebot4_ws/install/setup.bash && '
            'export ROS_DOMAIN_ID=117 && '
            'ros2 run alienbot_vision person_follower_calm"'
        )

    def stop_all(self):
        """Emergency stop — kill all modes, zero motors and wings."""
        self._kill_active()
        self._publish_stop()
        self.active_mode = None
        self._update_status('IDLE')

    # ══════════════════════════════════════════════════════════════════════
    # LIFECYCLE
    # ══════════════════════════════════════════════════════════════════════

    def on_close(self):
        """Clean shutdown."""
        self.stop_all()
        try:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass
        os._exit(0)

    def run(self):
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.on_close()


def main():
    ui = AlienBotUI()
    ui.run()


if __name__ == '__main__':
    main()