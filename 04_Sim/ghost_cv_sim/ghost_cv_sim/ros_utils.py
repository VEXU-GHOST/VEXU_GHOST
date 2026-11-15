"""Small ROS helpers shared by the nodes (image <-> numpy without cv_bridge)."""
import numpy as np
from sensor_msgs.msg import Image


def image_to_bgr(msg):
    """sensor_msgs/Image (rgb8 / bgr8 / mono8) -> HxWx3 uint8 BGR array."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = data.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img.copy()
    if msg.encoding == "mono8":
        img = data.reshape(msg.height, msg.step)[:, :msg.width]
        return np.repeat(img[:, :, None], 3, axis=2)
    raise ValueError("unsupported image encoding '%s'" % msg.encoding)


def bgr_to_image(bgr, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(bgr).tobytes()
    return msg


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
