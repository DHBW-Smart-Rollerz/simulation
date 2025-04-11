import rclpy
import rclpy.node
import rclpy.qos
import rclpy.wait_for_message
import std_msgs.msg
from ament_index_python.packages import get_package_share_directory


class KeyboardControl(rclpy.node.Node):
    """Keyboard control node for the steering angle and velocity."""

    def __init__(self):
        """Initialize the KeyboardControl."""
        super().__init__("keyboard_control_node")

        # Get the package path
        self.package_share_path = get_package_share_directory("simulation")

        # Load the parameters from the ROS parameter server and initialize
        # the publishers and subscribers
        self.load_ros_params()
        self.init_publisher_and_subscriber()

        self.steering_angle = 0.0
        self.velocity = 0.0
        self.steering_angle_increment = 15.0
        self.velocity_increment = 0.2

        self.key_pressed = None
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("KeyboardControl initialized")

    def load_ros_params(self):
        """Gets the parameters from the ROS parameter server."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("max_steering_angle", 45),
                ("max_velocity", 3.0),
                ("steering_topic", "/control/steering/target"),
                ("velocity_topic", "/control/velocity/target"),
                ("keypress_topic", "/gazebo/keypress"),
            ],
        )

        self.max_steering_angle = self.get_parameter("max_steering_angle").value
        self.max_velocity = self.get_parameter("max_velocity").value
        self.steering_topic = self.get_parameter("steering_topic").value
        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.keypress_topic = self.get_parameter("keypress_topic").value

    def init_publisher_and_subscriber(self):
        """Initializes the subscribers and publishers."""
        self.steering_angle_publisher = self.create_publisher(
            std_msgs.msg.Int16,
            self.steering_topic,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.velocity_publisher = self.create_publisher(
            std_msgs.msg.Float32,
            self.velocity_topic,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.keypress_subscriber = self.create_subscription(
            std_msgs.msg.Int32,
            self.keypress_topic,
            self.keypress_callback,
            qos_profile=1,
        )

    def update_control_commands(self):
        if self.key_pressed in (87, 16777235):  # w, arrow up
            self.velocity = min(
                self.max_velocity,
                self.velocity + self.velocity_increment,
            )
        elif self.key_pressed in (83, 16777237):  # s, arrow down
            self.velocity = max(
                -self.max_velocity,
                self.velocity - self.velocity_increment,
            )
        elif self.key_pressed in (65, 16777234):  # a, arrow left
            self.steering_angle = min(
                self.max_steering_angle,
                self.steering_angle + self.steering_angle_increment,
            )
        elif self.key_pressed in (68, 16777236):  # d, arrow right
            self.steering_angle = max(
                -self.max_steering_angle,
                self.steering_angle - self.steering_angle_increment,
            )

        if self.key_pressed not in (65, 16777234, 68, 16777236):  # a, left, d, right
            if self.steering_angle > 0.0:
                self.steering_angle = max(
                    0.0,
                    self.steering_angle - self.steering_angle_increment,
                )
            elif self.steering_angle < 0.0:
                self.steering_angle = min(
                    0.0,
                    self.steering_angle + self.steering_angle_increment,
                )

    def publish_control_commands(self):
        """Publishes the steering angle and velocity commands."""
        steering_angle_msg = std_msgs.msg.Int16()
        velocity_msg = std_msgs.msg.Float32()
        steering_angle_msg.data = int(self.steering_angle)
        velocity_msg.data = self.velocity

        self.steering_angle_publisher.publish(steering_angle_msg)
        self.velocity_publisher.publish(velocity_msg)

    def keypress_callback(self, msg: std_msgs.msg.Float32):
        self.key_pressed = msg.data

    def control_loop(self):
        """Control loop executed by the timer."""
        self.update_control_commands()
        self.publish_control_commands()
        self.key_pressed = None


def main(args=None):
    """
    Main function to start the KeyboardControl.

    Keyword Arguments:
        args -- Launch arguments (default: {None})
    """
    rclpy.init(args=args)
    node = KeyboardControl()

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
