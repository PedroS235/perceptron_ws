import rclpy
import time
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriverNode(Node):
    cmd_vel_topic_name = "/cmd_vel"
    odom_topic_name = "/odom"
    base_link_frame_id = "base_link"
    odom_frame_id = "odom"

    cmd_vel_sub = None
    odom_pub = None
    odom_pub_timer = None
    odom_timer_frequency = 10  # Hz

    serial_port = "/dev/ttyUSB0"
    serial_baudrate = 9600

    def __init__(self, node_name) -> None:
        super().__init__(node_name)
        self.read_params()

        self.serial = serial.Serial(
            self.serial_port, self.serial_baudrate, timeout=0.5
        )
        time.sleep(2)

        if self.serial.is_open:
            self.get_logger().info(
                f"succesfully connected to {self.serial_port}"
            )
        else:
            self.get_logger().error(
                f"failed to connect to {self.serial_port}. Exiting..."
            )
            exit()
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic_name,
            self.cmd_vel_callback,
            1,
        )
        self.odom_pub = self.create_publisher(Odometry, "odom", 1)
        self.odom_pub_timer = self.create_timer(
            1 / self.odom_timer_frequency, self.odom_callback
        )

        self.get_logger().info("Driver node started")

    def read_params(self) -> None:
        self.declare_parameter("serial_port", self.serial_port)
        self.declare_parameter("serial_baudrate", self.serial_baudrate)
        self.declare_parameter("cmd_vel_topic_name", self.cmd_vel_topic_name)
        self.declare_parameter("base_link_frame_id", self.base_link_frame_id)
        self.declare_parameter("odom_frame_id", self.odom_frame_id)

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

    def cmd_vel_callback(self, msg: Twist) -> None:
        flag = "c"
        x = int(msg.linear.x * 1000)
        w = int(msg.angular.z * 1000)

        cmd = f"{flag} {x} {w}\n"

        self.serial.write(cmd.encode())

        ack = self.serial.readline().decode()
        if ack != "OK\r\n":
            self.get_logger().error("Error sending command to arduino")

    def odom_callback(self) -> None:
        cmd = "q\n"

        self.serial.write(cmd.encode())

        pose = self.serial.readline().decode()
        try:
            pose = [float(q) for q in pose.split(" ")]
        except ValueError:
            self.get_logger().error("Error reading odometry data")
            return

        msg = Odometry()
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = pose[2]

        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_link_frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        self.odom_pub.publish(msg)

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
