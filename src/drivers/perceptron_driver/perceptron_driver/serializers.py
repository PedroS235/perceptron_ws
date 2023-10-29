from nav_msgs.msg import Odometry
import rclpy
from rclpy.time import Time


def serialize_odom_msg(
    frame_id: str,
    child_frame_id: str,
    now: Time,
    x: float,
    y: float,
    theta: float,
):
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id
    msg.header.stamp = now.to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = theta
    return msg
