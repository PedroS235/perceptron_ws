from nav_msgs.msg import Odometry
from rclpy.time import Time
from tf2_ros import TransformStamped
from tf_transformations import quaternion_from_euler


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

    q = quaternion_from_euler(0, 0, theta)
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]

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

    q = quaternion_from_euler(0, 0, theta)
    tf.transform.rotation.x = q[0]
    tf.transform.rotation.y = q[1]
    tf.transform.rotation.z = q[2]
    tf.transform.rotation.w = q[3]

    return tf
