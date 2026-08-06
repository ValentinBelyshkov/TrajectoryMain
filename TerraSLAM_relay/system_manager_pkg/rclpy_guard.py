"""
Guard the shared rclpy context.

Several modules (ros_pose_subscriber, camera_stream, camera_snapshot) spin up
rclpy nodes that all share ONE rclpy context for the lifetime of the process.
If any of them calls rclpy.shutdown(), every other subscriber silently breaks
— including the GPS/pose subscriber. We suppress rclpy.shutdown() so the context
stays alive for the whole manager process. Import this module early (before any
rclpy-using module) to install the guard.
"""
import sys

try:
    import rclpy

    _original_shutdown = rclpy.shutdown

    def _guarded_shutdown(*args, **kwargs):
        print(
            "[rclpy_guard] rclpy.shutdown() suppressed — shared context must stay "
            "alive for the manager process",
            file=sys.stderr,
            flush=True,
        )

    rclpy.shutdown = _guarded_shutdown
except Exception:
    # rclpy not available in this environment (e.g. outside the ROS container).
    pass
