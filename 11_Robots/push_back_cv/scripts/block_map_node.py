#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from push_back_cv.msg import CvDetectionArray, FieldBlock, FieldBlockArray
import math
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformException, TransformListener

MERGE_DIST_M = 0.15
CAMERA_FRAME = "camera_link"
HFOV_RAD = math.radians(69)       # 69 degree HFOV from intel realsense
MAX_RANGE_M = 2.5    # max distance to repaint tracks

class BlockMapNode(Node):
    def __init__(self):
        super().__init__("block_map")
        self.tracks = {}
        self.next_id = 0
        self.map_frame = "map"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(CvDetectionArray, "/cv/detections", self._on_detections, 10)
        self.pub = self.create_publisher(FieldBlockArray, "/field/blocks", 10)
        self.get_logger().info("Starting block_map")

    def _yaw_from_transform(self, t) -> float:
        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _angle_diff(self, a: float, b: float) -> float:
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def _in_fov(self, x: float, y: float, cam_x: float, cam_y: float, cam_yaw: float) -> bool:
        dx = x - cam_x
        dy = y - cam_y
        dist = math.hypot(dx, dy)
        if dist > MAX_RANGE_M:
            return False
        bearing = math.atan2(dy, dx)
        return abs(self._angle_diff(bearing, cam_yaw)) <= HFOV_RAD / 2.0

    def _camera_view(self) -> tuple[float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                CAMERA_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warning(f"FOV TF failed: {exc}", throttle_duration_sec=2.0)
            return None
        cam_x = tf.transform.translation.x
        cam_y = tf.transform.translation.y
        cam_yaw = self._yaw_from_transform(tf)
        return cam_x, cam_y, cam_yaw

    def _publish(self):
        out = FieldBlockArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        out.blocks = list(self.tracks.values())
        self.pub.publish(out)

    def _on_detections(self, msg: CvDetectionArray):
        view = self._camera_view()
        if view is None:
            self._publish()
            return
        cam_x, cam_y, cam_yaw = view

        now = self.get_clock().now().to_msg()
        matched_track_ids = set()

        # Empty frame — clear tracks in FOV (the B2 fix)
        if not msg.detections:
            for tid in list(self.tracks.keys()):
                t = self.tracks[tid]
                if self._in_fov(t.x, t.y, cam_x, cam_y, cam_yaw):
                    del self.tracks[tid]
            self._publish()
            return

        # Match detections → tracks (same as B1, but _in_fov)
        for det in msg.detections:
            if not self._in_fov(det.x, det.y, cam_x, cam_y, cam_yaw):
                continue
            best_id = None
            best_dist = MERGE_DIST_M

            for tid, track in self.tracks.items():
                if tid in matched_track_ids:
                    continue
                if track.is_red != det.is_red:
                    continue
                if not self._in_fov(track.x, track.y, cam_x, cam_y, cam_yaw):
                    continue
                dist = math.hypot(track.x - det.x, track.y - det.y)
                if dist < best_dist:
                    best_dist, best_id = dist, tid

            if best_id is not None:
                matched_track_ids.add(best_id)
                t = self.tracks[best_id]
                t.x, t.y, t.z = det.x, det.y, det.z
                t.confidence = det.confidence
                t.last_seen = now
            else:
                self.tracks[self.next_id] = FieldBlock(
                    id=self.next_id,
                    x=det.x, y=det.y, z=det.z,
                    is_red=det.is_red,
                    confidence=det.confidence,
                    last_seen=now,
                )
                self.next_id += 1

        # Delete unmatched tracks inside FOV
        for tid in list(self.tracks.keys()):
            t = self.tracks[tid]
            if not self._in_fov(t.x, t.y, cam_x, cam_y, cam_yaw):
                continue
            if tid not in matched_track_ids:
                del self.tracks[tid]

        self._publish()


def main():
    rclpy.init()
    node = BlockMapNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()