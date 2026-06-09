#!/usr/bin/env python3
"""Live colour swatch for the sensor host colour sensor, in the terminal.

Renders the published RGB as a 24-bit truecolor block (works over SSH, no GUI).

Usage:
    python3 color_sensor.py [topic]

`topic` defaults to /sensor_host/color_sensor_update.
Ctrl-C to quit. Requires a truecolor-capable terminal.
"""
import colorsys
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ghost_msgs.msg import ColorSensorState

DEFAULT_TOPIC = "/sensor_host/color_sensor_update"
BAR_W = 48


def hsv_px(h, s, v):
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, s, v)
    return int(r * 255), int(g * 255), int(b * 255)


def bar(color_fn, pos_frac):
    """A BAR_W-wide truecolor gradient with a marker at pos_frac (0..1).

    color_fn(t) -> (r, g, b) for t in [0, 1].
    """
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


def render(name, r16, g16, b16):
    # 16-bit sensor channels -> 8-bit display.
    r, g, b = r16 >> 8, g16 >> 8, b16 >> 8
    # HSV from the full 16-bit channels (hue is ratio-based; V shows saturation).
    h, s, v = colorsys.rgb_to_hsv(r16 / 65535.0, g16 / 65535.0, b16 / 65535.0)

    out = ["\x1b[H"]  # cursor home
    sw = f"\x1b[48;2;{r};{g};{b}m"
    for _ in range(6):
        out.append(sw + " " * BAR_W + "\x1b[0m\x1b[K\n")
    out.append(f"\x1b[K {name}\n")
    out.append(f"\x1b[K RGB  R:{r16:5d}  G:{g16:5d}  B:{b16:5d}   (8-bit {r:3d},{g:3d},{b:3d})\n")
    # Each bar sweeps one HSV component, holding the other two at current.
    out.append(f"\x1b[K H {bar(lambda t: hsv_px(t, s, v), h)} {h * 360:5.1f}deg\n")
    out.append(f"\x1b[K S {bar(lambda t: hsv_px(h, t, v), s)} {s * 100:5.1f}%\n")
    out.append(f"\x1b[K V {bar(lambda t: hsv_px(h, s, t), v)} {v * 100:5.1f}%\n")
    out.append("\x1b[K Ctrl-C to quit\n")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TOPIC

    rclpy.init()
    node = Node("color_sensor_debug")
    node.create_subscription(
        ColorSensorState, topic,
        lambda m: render(m.name, m.r, m.g, m.b),
        qos_profile_sensor_data)

    sys.stdout.write("\x1b[2J\x1b[?25l")  # clear screen, hide cursor
    sys.stdout.flush()
    print(f"Listening on {topic} ...")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        sys.stdout.write("\x1b[?25h\n")  # restore cursor
        sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
