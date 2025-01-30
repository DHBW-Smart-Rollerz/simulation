import geometry_msgs.msg
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.wait_for_message
import std_msgs.msg
from ament_index_python.packages import get_package_share_directory


class CarControlBridge(rclpy.node.Node):
    """Bridges the car control commands to the Gazebo simulation."""

    def __init__(self):
        """Initialize the CarControlBridge."""
        super().__init__("car_control_bridge")

        # Get the package path
        self.package_share_path = get_package_share_directory("simulation")

        # Load the parameters from the ROS parameter server and initialize
        # the publishers and subscribers
        self.load_ros_params()
        self.init_publisher_and_subscriber()

        self.steering_angle = 0.0
        self.velocity = 0.0

        # Create a timer to publish the control command
        self.create_timer(
            1.0 / self.publish_frequency,
            self.publish_control_command,
        )

        self.get_logger().info("CarControlBridge initialized")

    def load_ros_params(self):
        """Gets the parameters from the ROS parameter server."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("steering_topic", "/control/steering/target"),
                ("veloctiy_topic", "/control/velocity/target"),
                ("car_control_topic", "/smarty/cmd_vel"),
                ("publish_frequency", 100),
            ],
        )

        self.steering_topic = self.get_parameter("steering_topic").value
        self.veloctiy_topic = self.get_parameter("veloctiy_topic").value
        self.car_control_topic = self.get_parameter("car_control_topic").value
        self.publish_frequency = self.get_parameter("publish_frequency").value

    def init_publisher_and_subscriber(self):
        """Initializes the subscribers and publishers."""
        self.steering_angle_subscriber = self.create_subscription(
            std_msgs.msg.Int16,
            self.steering_topic,
            self.steering_angle_callback,
            qos_profile=1,
        )
        self.velocity_subscriber = self.create_subscription(
            std_msgs.msg.Float32,
            self.veloctiy_topic,
            self.velocity_callback,
            qos_profile=1,
        )

        self.control_command_publisher = self.create_publisher(
            geometry_msgs.msg.Twist,
            self.car_control_topic,
            qos_profile=10,
        )

    def steering_angle_callback(self, msg: std_msgs.msg.Int16):
        """
        Callback function for the steering angle subscriber.

        Arguments:
            msg -- The received message.
        """
        self.steering_angle = float(msg.data)

    def velocity_callback(self, msg: std_msgs.msg.Float32):
        """
        Callback function for the velocity subscriber.

        Arguments:
            msg -- The received message.
        """
        self.velocity = msg.data

    def publish_control_command(self):
        """Publishes the control command as a Twist to the Gazebo simulation."""
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = self.velocity
        msg.angular.z = self.steering_angle

        self.control_command_publisher.publish(msg)


def main(args=None):
    """
    Main function to start the CarControlBridge.

    Keyword Arguments:
        args -- Launch arguments (default: {None})
    """
    rclpy.init(args=args)
    node = CarControlBridge()

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
