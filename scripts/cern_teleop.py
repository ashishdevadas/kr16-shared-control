#!/usr/bin/env python3
"""
Keyboard teleoperation window for the human trials (cern_shared_control v0.7)

    ros2 run cern_shared_control cern_teleop.py

Hold a key to move, release to stop. Commands are in the fixed frame (base_link):
    W / S   forward / back  (towards / away from the equipment face, +x / -x)
    A / D   left / right    (+y / -y)
    R / F   up / down       (+z / -z)
    I / K   pitch           (rotation about y)
    J / L   yaw             (rotation about z)
    U / O   roll            (rotation about x)
    1 / 2   fine / normal speed
    SPACE   start the trial         ENTER   done (the tool is on the green target)

Publishes /operator/twist (geometry_msgs/Twist, 50 Hz) and /trial/cmd (std_msgs/String),
shows /trial/status (std_msgs/String "STATE|text[|depth_mm|side_mm|angle_deg]") from the shared_control
node (OPERATOR=human), with the readout of the tool position and angle w.r.t. the green target.
"""
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from python_qt_binding import QtCore, QtGui, QtWidgets

Qt = QtCore.Qt

SPEED = {"fine": (0.008, 0.06), "normal": (0.03, 0.20)}   # (m/s, rad/s) before the simulator gain
STATUS_TIMEOUT = 2.0                                      # [s] no status -> controller not running
TOL_POS, TOL_ROT = 5.0, 3.0                                # success tolerances [mm], [deg]
COLORS = {"WAIT": "#7f8c8d", "READY": "#e67e22", "GO": "#27ae60", "DONE": "#2980b9", "OFF": "#c0392b"}


def qt(name, group=None):
    """Qt enum compatible with Qt5 (Qt.Key_W) and Qt6 (Qt.Key.Key_W) bindings"""
    if hasattr(Qt, name):
        return getattr(Qt, name)
    return getattr(getattr(Qt, group), name)


def as_int(k):
    try:
        return int(k)
    except TypeError:
        return k.value


# key -> (twist component, sign); components 0-2 linear x y z, 3-5 angular x y z
MOTION = {
    "Key_W": (0, +1), "Key_S": (0, -1),
    "Key_A": (1, +1), "Key_D": (1, -1),
    "Key_R": (2, +1), "Key_F": (2, -1),
    "Key_U": (3, +1), "Key_O": (3, -1),
    "Key_I": (4, +1), "Key_K": (4, -1),
    "Key_J": (5, +1), "Key_L": (5, -1),
}
MOTION = {as_int(qt(k, "Key")): v for k, v in MOTION.items()}
K_SPACE = as_int(qt("Key_Space", "Key"))
K_ENTER = {as_int(qt("Key_Return", "Key")), as_int(qt("Key_Enter", "Key"))}
K_FINE, K_NORMAL = as_int(qt("Key_1", "Key")), as_int(qt("Key_2", "Key"))

LEGEND = """<table cellpadding="4" style="font-size:15px">
<tr><td><b>W / S</b></td><td>forward / back</td><td width="30"></td><td><b>I / K</b></td><td>pitch</td></tr>
<tr><td><b>A / D</b></td><td>left / right</td><td></td><td><b>J / L</b></td><td>yaw</td></tr>
<tr><td><b>R / F</b></td><td>up / down</td><td></td><td><b>U / O</b></td><td>roll</td></tr>
<tr><td><b>SPACE</b></td><td>start trial</td><td></td><td><b>ENTER</b></td><td>done</td></tr>
<tr><td><b>1 / 2</b></td><td>fine / normal speed</td><td></td><td></td><td></td></tr>
</table>"""


class TeleopNode(Node):
    def __init__(self):
        super().__init__("cern_teleop")
        self.twist_pub = self.create_publisher(Twist, "operator/twist", 10)
        self.cmd_pub = self.create_publisher(String, "trial/cmd", 10)
        self.status = "OFF|Waiting for the controller (shared_control with OPERATOR=human)..."
        self.status_time = 0.0
        self.create_subscription(String, "trial/status", self.on_status, 10)

    def on_status(self, msg):
        self.status = msg.data
        self.status_time = time.monotonic()

    def send_cmd(self, text):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    def send_twist(self, v):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = v[0], v[1], v[2]
        msg.angular.x, msg.angular.y, msg.angular.z = v[3], v[4], v[5]
        self.twist_pub.publish(msg)


class TeleopWindow(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.held = set()
        self.speed = "normal"
        self.setWindowTitle("CERN shared control - operator keyboard")
        self.setFocusPolicy(qt("StrongFocus", "FocusPolicy"))
        self.resize(780, 500)

        center = qt("AlignCenter", "AlignmentFlag")
        self.state = QtWidgets.QLabel()
        self.state.setAlignment(center)
        self.state.setStyleSheet("font-size: 34px; font-weight: bold; color: white; padding: 14px;")
        self.text = QtWidgets.QLabel()
        self.text.setAlignment(center)
        self.text.setWordWrap(True)
        self.text.setStyleSheet("font-size: 18px; padding: 8px;")
        self.hud = QtWidgets.QLabel()
        self.hud.setAlignment(center)
        self.hud.setTextFormat(qt("RichText", "TextFormat"))
        self.info = QtWidgets.QLabel()
        self.info.setAlignment(center)
        self.info.setStyleSheet("font-size: 14px; color: #555;")
        legend = QtWidgets.QLabel(LEGEND)
        legend.setAlignment(center)
        self.focus = QtWidgets.QLabel("Click here so the keys reach the robot")
        self.focus.setAlignment(center)
        self.focus.setStyleSheet("font-size: 16px; font-weight: bold; color: white; background: #c0392b; padding: 6px;")

        lay = QtWidgets.QVBoxLayout(self)
        for w in (self.state, self.text, self.hud, self.info, legend, self.focus):
            lay.addWidget(w)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)   # 50 Hz
        self.tick()

    # ---------------- keyboard ----------------
    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        k = as_int(e.key())
        if k in MOTION:
            self.held.add(k)
        elif k == K_SPACE:
            self.node.send_cmd("start")
        elif k in K_ENTER:
            self.held.clear()
            self.node.send_cmd("done")
        elif k == K_FINE:
            self.speed = "fine"
        elif k == K_NORMAL:
            self.speed = "normal"

    def keyReleaseEvent(self, e):
        if e.isAutoRepeat():
            return
        self.held.discard(as_int(e.key()))

    def focusOutEvent(self, e):
        self.held.clear()        # never keep moving when the window loses the keyboard
        super().focusOutEvent(e)

    def mousePressEvent(self, e):
        self.setFocus()
        super().mousePressEvent(e)

    def show_hud(self, vals):
        if vals is None:
            self.hud.setText("")
            return
        depth, side, angle = (float(v) for v in vals)
        ok = lambda good: "#27ae60" if good else "#c0392b"
        where = f"{depth:.0f} mm in front" if depth >= 0 else f"{-depth:.0f} mm TOO DEEP"
        cell = '<td style="padding:4px 18px; font-size:22px; font-weight:bold; color:{};">{}</td>'
        self.hud.setText("<table align='center'><tr>"
                         + cell.format(ok(abs(depth) < TOL_POS), "Depth: " + where)
                         + cell.format(ok(side < TOL_POS), f"Side: {side:.0f} mm")
                         + cell.format(ok(angle < TOL_ROT), f"Angle: {angle:.1f}&deg;")
                         + "</tr></table><div style='font-size:13px; color:#555;'>"
                         f"target reached when all three are green (&lt; {TOL_POS:.0f} mm, &lt; {TOL_ROT:.0f}&deg;)</div>")

    # ---------------- 50 Hz loop ----------------
    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        v_lin, v_ang = SPEED[self.speed]
        v = [0.0] * 6
        for k in self.held:
            i, sign = MOTION[k]
            v[i] += sign * (v_lin if i < 3 else v_ang)
        self.node.send_twist(v)

        status = self.node.status
        if time.monotonic() - self.node.status_time > STATUS_TIMEOUT:
            status = "OFF|Waiting for the controller (shared_control with OPERATOR=human)..."
        parts = status.split("|")
        state, text = parts[0], parts[1] if len(parts) > 1 else ""
        self.show_hud(parts[2:5] if len(parts) >= 5 and state in ("READY", "GO") else None)
        title = {"WAIT": "PLEASE WAIT", "READY": "READY", "GO": "GO", "DONE": "FINISHED", "OFF": "NOT CONNECTED"}
        self.state.setText(title.get(state, state))
        self.state.setStyleSheet("font-size: 34px; font-weight: bold; color: white; padding: 14px; "
                                 f"background: {COLORS.get(state, '#7f8c8d')};")
        self.text.setText(text)
        moving = "moving" if any(v) else "stopped"
        self.info.setText(f"Speed: {self.speed.upper()}   |   robot command: {moving}")
        self.focus.setVisible(not self.hasFocus())


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)   # Ctrl+C closes the window
    rclpy.init(args=sys.argv)
    node = TeleopNode()
    app = QtWidgets.QApplication(sys.argv)
    win = TeleopWindow(node)
    win.show()
    win.activateWindow()
    win.setFocus()
    run = getattr(app, "exec", None) or getattr(app, "exec_")
    code = run()
    node.send_twist([0.0] * 6)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
