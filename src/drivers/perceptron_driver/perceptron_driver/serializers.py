from nav_msgs.msg import Odometry
import rclpy
from rclpy.time import Time
from tf2_ros import TransformStamped


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


def create_tf_transform(
    parent: str, child: str, now: Time, x: float, y: float, theta: float
) -> TransformStamped:
    tf = TransformStamped()
    tf.header.stamp = now.to_msg()
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.rotation.z = theta
    return tf
