import os

import rclpy
import rclpy.node
import rclpy.qos
import rclpy.wait_for_message
import std_msgs.msg
from ament_index_python.packages import get_package_share_directory


class RobotDescriptionPublisher(rclpy.node.Node):
    """Robot description publisher node."""

    def __init__(self):
        """Initialize the ROS2ExampleNode."""
        super().__init__("robot_description_publisher")

        # Get the package path
        self.package_share_path = get_package_share_directory("simulation")

        # Load the parameters from the ROS parameter server and initialize
        # the publishers and subscribers
        self.load_ros_params()
        self.init_publisher_and_subscriber()
        self.get_logger().info("RobotDescriptionPublisher initialized")

        # Publish the robot description once (!)
        self.publish_robot_description(self.model_path)

    def load_ros_params(self):
        """Gets the parameters from the ROS parameter server."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("model_path", "models/smarty"),
                ("robot_description_topic", "/robot_description"),
            ],
        )

        self.model_path = self.get_parameter("model_path").value
        if self.model_path.endswith(".sdf") or self.model_path.endswith(".urdf"):
            self.model_path = os.path.join(
                self.package_share_path,
                self.get_parameter("model_path").value,
            )
        else:
            self.model_path = os.path.join(
                self.package_share_path,
                self.get_parameter("model_path").value,
                "model.sdf",
            )

        self.robot_description_topic = self.get_parameter(
            "robot_description_topic"
        ).value

    def init_publisher_and_subscriber(self):
        """Initializes the subscribers and publishers."""
        self.robot_description_publisher = self.create_publisher(
            std_msgs.msg.String,
            self.robot_description_topic,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def publish_robot_description(self, model_path):
        """
        Publishes the model description to the robot_description topic.

        Arguments:
            model_path -- The path to the model file.
        """
        try:
            with open(model_path, "r") as file:
                model_xml = file.read()
        except FileNotFoundError:
            self.get_logger().error(f"Model file not found: {model_path}")
            return

        # Inlcude the package folder in the paths to files so that rviz can find them
        model_xml = model_xml.replace("package://", "package://simulation/")

        msg = std_msgs.msg.String(data=model_xml)
        self.robot_description_publisher.publish(msg)
        self.get_logger().info(
            f"Published robot description to {self.robot_description_topic}"
        )


def main(args=None):
    """
    Main function to start the ROS2ExampleNode.

    Keyword Arguments:
        args -- Launch arguments (default: {None})
    """
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()

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
