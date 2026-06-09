#!/usr/bin/env python3
"""Live colour swatch for the sensor host colour sensor, in the terminal.

Renders the published RGB as a 24-bit truecolor block plus H/S/V gradient bars
(works over SSH, no GUI).

The ISL29125's raw channels are unbalanced (green is far more sensitive), so a
neutral white reads greenish. Point the sensor at something white/grey and press
'w' to white-balance: each channel is then divided by that reference so white
reads white and objects show their true hue. The published topic stays raw.

Usage:
    python3 color_sensor.py [topic]

`topic` defaults to /sensor_host/color_sensor_update.
Keys: w = capture white, r = reset balance, q = quit. Truecolor terminal needed.
"""
import colorsys
import select
import sys
import termios
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ghost_msgs.msg import ColorSensorState

DEFAULT_TOPIC = "/sensor_host/color_sensor_update"
BAR_W = 48

# Red/blue ball classification from the raw R/B ratio (green is ignored — it
# only carries the sensor's gain imbalance). Tune by holding up each ball and
# watching the printed R/B value. A red ball gives R/B well above 1, blue well
# below 1. MIN_LEVEL gates out "nothing/too dark in front of the sensor".
RED_RB = 1.4
BLUE_RB = 0.7
MIN_LEVEL = 1500


def classify(r16, g16, b16):
    if max(r16, g16, b16) < MIN_LEVEL:
        return "none", "\x1b[90m none  (dark / no object)\x1b[0m"
    rb = r16 / max(1, b16)
    if rb >= RED_RB:
        return "red", f"\x1b[1;97;41m RED  \x1b[0m  R/B={rb:4.2f}"
    if rb <= BLUE_RB:
        return "blue", f"\x1b[1;97;44m BLUE \x1b[0m  R/B={rb:4.2f}"
    return "unsure", f"\x1b[1;30;43m  ?   \x1b[0m  R/B={rb:4.2f}"


def hsv_px(h, s, v):
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, s, v)
    return int(r * 255), int(g * 255), int(b * 255)


def bar(color_fn, pos_frac):
    """A BAR_W-wide truecolor gradient with a marker at pos_frac (0..1)."""
    marker = max(0, min(BAR_W - 1, round(pos_frac * (BAR_W - 1))))
    cells = []
    for i in range(BAR_W):
        r, g, b = color_fn(i / (BAR_W - 1))
        if i == marker:
            fg = "0;0;0" if (r + g + b) > 384 else "255;255;255"
            cells.append(f"\x1b[48;2;{r};{g};{b}m\x1b[38;2;{fg}m|\x1b[0m")
        else:
            cells.append(f"\x1b[48;2;{r};{g};{b}m \x1b[0m")
    return "".join(cells)


def render(name, r16, g16, b16, white):
    # Per-channel divisor: the white reference if balanced, else full scale.
    dr, dg, db = white if white else (65535, 65535, 65535)
    nr = min(1.0, r16 / max(1, dr))
    ng = min(1.0, g16 / max(1, dg))
    nb = min(1.0, b16 / max(1, db))
    r, g, b = int(nr * 255), int(ng * 255), int(nb * 255)
    h, s, v = colorsys.rgb_to_hsv(nr, ng, nb)

    out = ["\x1b[H"]  # cursor home
    sw = f"\x1b[48;2;{r};{g};{b}m"
    for _ in range(6):
        out.append(sw + " " * BAR_W + "\x1b[0m\x1b[K\n")
    out.append(f"\x1b[K {name}\n")
    out.append(f"\x1b[K RGB  R:{r16:5d}  G:{g16:5d}  B:{b16:5d}   ->  {r:3d},{g:3d},{b:3d}\n")
    out.append(f"\x1b[K ball: {classify(r16, g16, b16)[1]}\x1b[K\n")
    out.append(f"\x1b[K H {bar(lambda t: hsv_px(t, s, v), h)} {h * 360:5.1f}deg\n")
    out.append(f"\x1b[K S {bar(lambda t: hsv_px(h, t, v), s)} {s * 100:5.1f}%\n")
    out.append(f"\x1b[K V {bar(lambda t: hsv_px(h, s, t), v)} {v * 100:5.1f}%\n")
    wb = f"{white[0]},{white[1]},{white[2]}" if white else "OFF"
    out.append(f"\x1b[K white-balance: {wb}\x1b[K\n")
    out.append("\x1b[K [w] capture white   [r] reset   [q] quit\n")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TOPIC
    state = {"latest": (0, 0, 0), "white": None, "name": ""}

    rclpy.init()
    node = Node("color_sensor_debug")

    def on_msg(msg):
        state["latest"] = (msg.r, msg.g, msg.b)
        state["name"] = msg.name
        render(msg.name, msg.r, msg.g, msg.b, state["white"])

    node.create_subscription(ColorSensorState, topic, on_msg, qos_profile_sensor_data)

    interactive = sys.stdin.isatty()
    old_termios = None
    if interactive:
        old_termios = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        def poll_keys():
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "w":
                    state["white"] = state["latest"]
                elif ch == "r":
                    state["white"] = None
                elif ch in ("q", "\x03"):
                    rclpy.shutdown()
        node.create_timer(0.05, poll_keys)

    sys.stdout.write("\x1b[2J\x1b[?25l")  # clear screen, hide cursor
    sys.stdout.flush()
    print(f"Listening on {topic} ...")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if interactive and old_termios is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_termios)
        sys.stdout.write("\x1b[?25h\n")  # restore cursor
        sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
