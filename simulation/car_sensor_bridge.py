import nav_msgs.msg
import rclpy
import rclpy.node
import std_msgs.msg
from ament_index_python.packages import get_package_share_directory


class CarSensorBridge(rclpy.node.Node):
    """Bridges the car sensor data from Gazebo to ROS."""

    def __init__(self):
        """Initialize the CarSensorBridge."""
        super().__init__("car_sensor_bridge")

        # Get the package path
        self.package_share_path = get_package_share_directory("simulation")

        # Load the parameters from the ROS parameter server and initialize
        # the publishers and subscribers
        self.load_ros_params()
        self.init_publisher_and_subscriber()

        self.get_logger().info("CarSensorBridge initialized")

    def load_ros_params(self):
        """Gets the parameters from the ROS parameter server."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("velocity_topic", "/sensor/velocity"),
                ("odom_topic", "/smarty/odometry/ackermann"),
            ],
        )

        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value

    def init_publisher_and_subscriber(self):
        """Initializes the subscribers and publishers."""
        self.odom_subscriber = self.create_subscription(
            nav_msgs.msg.Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile=1,
        )
        self.velocity_publisher = self.create_publisher(
            std_msgs.msg.Float32,
            self.velocity_topic,
            qos_profile=1,
        )

    def odom_callback(self, msg: nav_msgs.msg.Odometry):
        """
        Callback function for the odometry subscriber.

        Arguments:
            msg -- The received message.
        """
        velocity = msg.twist.twist.linear.x

        msg = std_msgs.msg.Float32()
        msg.data = velocity
        self.velocity_publisher.publish(msg)


def main(args=None):
    """
    Main function to start the CarSensorBridge.

    Keyword Arguments:
        args -- Launch arguments (default: {None})
    """
    rclpy.init(args=args)
    node = CarSensorBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        # Shutdown if not already done by the ROS2 launch system
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
