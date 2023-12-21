import rclpy
import time
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from perceptron_driver.serializers import (
    serialize_odom_msg,
    create_tf_transform,
)
import tf2_ros

# TODO: Add a timeout for cmd_vel.
# In case of no cmd_vel message for a long time, the robot should stop,
# to prevent the robot from running away if the connection is lost.


class DriverNode(Node):
    """Driver node class

    Args:
        Node (rclpy.node.Node): base class for all nodes in ROS2
    """

    cmd_vel_topic_name = "/cmd_vel"
    odom_topic_name = "/odom"
    base_link_frame_id = "base_link"
    odom_frame_id = "odom"

    cmd_vel_sub = None
    odom_pub = None
    odom_pub_timer = None
    odom_timer_frequency = 10  # Hz
    tf_boardcast_timer = None

    serial_port = "/dev/ttyUSB0"
    serial_baudrate = 9600

    reset_odom_flag = True

    def __init__(self, node_name) -> None:
        """Constructor of DriverNode class

        Args:
            node_name (str): name of the node
        """
        super().__init__(node_name)
        self.read_params()

        try:
            self.serial = serial.Serial(
                self.serial_port, self.serial_baudrate, timeout=0.5
            )
        except serial.SerialException:
            self.get_logger().error(
                f"Could not open serial port {self.serial_port}"
            )
            exit()

        # Give some time for serial port to open
        time.sleep(2)

        self.check_serial()

        if self.reset_odom_flag:
            self.reset_odom()

        self.init_subs()
        self.init_pubs()
        self.init_timers()

        self.get_logger().info("Driver node is ready")

    def reset_odom(self) -> None:
        """Reset odometry"""
        cmd = "r\n"
        self.serial.write(cmd.encode())

        ack = self.serial.readline().decode()
        if ack != "OK\r\n":
            self.get_logger().error("Error resetting odometry")

    def check_serial(self) -> None:
        """Check if serial port is open"""
        if not self.serial.is_open:
            self.get_logger().error(
                f"serial port {self.serial_port} is not open"
            )
            exit()

    def init_subs(self) -> None:
        """Initialize subscribers"""
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic_name,
            self.cmd_vel_callback,
            1,
        )

    def init_pubs(self) -> None:
        """Initialize publishers"""
        self.odom_pub = self.create_publisher(Odometry, "odom", 1)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def init_timers(self) -> None:
        """Initialize timers"""
        self.odom_pub_timer = self.create_timer(
            1 / self.odom_timer_frequency, self.odom_callback
        )

    def read_params(self) -> None:
        """Read parameters from parameter server"""
        self.declare_parameter("serial_port", self.serial_port)
        self.declare_parameter("serial_baudrate", self.serial_baudrate)
        self.declare_parameter("cmd_vel_topic_name", self.cmd_vel_topic_name)
        self.declare_parameter("base_link_frame_id", self.base_link_frame_id)
        self.declare_parameter("odom_frame_id", self.odom_frame_id)
        self.declare_parameter("reset_odom", self.reset_odom_flag)

        self.serial_port = (
            self.get_parameter("serial_port")
            .get_parameter_value()
            .string_value
        )
        self.serial_baudrate = (
            self.get_parameter("serial_baudrate")
            .get_parameter_value()
            .integer_value
        )
        self.cmd_vel_topic_name = (
            self.get_parameter("cmd_vel_topic_name")
            .get_parameter_value()
            .string_value
        )
        self.base_link_frame_id = (
            self.get_parameter("base_link_frame_id")
            .get_parameter_value()
            .string_value
        )
        self.odom_frame_id = (
            self.get_parameter("odom_frame_id")
            .get_parameter_value()
            .string_value
        )
        self.reset_odom_flag = (
            self.get_parameter("reset_odom").get_parameter_value().bool_value
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Callback function for cmd_vel topic

        Args:
            msg (Twist): message from cmd_vel topic
        """
        flag = "c"
        x = int(msg.linear.x * 1000)
        w = int(msg.angular.z * 1000)

        cmd = f"{flag} {x} {w}\n"

        self.serial.write(cmd.encode())

        ack = self.serial.readline().decode()
        if ack != "OK\r\n":
            self.get_logger().error("Error sending command to arduino")

    def odom_callback(self) -> None:
        """Callback function for odom timer"""
        cmd = "q\n"

        self.serial.write(cmd.encode())

        pose = self.serial.readline().decode()
        now = self.get_clock().now()

        try:
            pose = [float(q) for q in pose.split(" ")]
        except ValueError:
            self.get_logger().error("Error reading odometry data")
            return

        odom_msg = serialize_odom_msg(
            self.odom_frame_id,
            self.base_link_frame_id,
            now,
            pose[0],
            pose[1],
            pose[2],
        )

        tf_odom_base = create_tf_transform(
            self.odom_frame_id,
            self.base_link_frame_id,
            now,
            pose[0],
            pose[1],
            pose[2],
        )

        # publish odometry
        self.odom_pub.publish(odom_msg)

        # publish tf
        self.tf_broadcaster.sendTransform(tf_odom_base)

    def close_serial(self) -> None:
        self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    driver_node = DriverNode("perceptron_driver")
    rclpy.spin(driver_node)
    driver_node.close_serial()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
