import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class NavigatorNode(LifecycleNode):
    """Main node for robot line follow."""

    def __init__(self) -> None:
        super().__init__('navigator_node')

        self.camera_sub: Subscription | None = None
        self.error_pub: Publisher | None = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.error_pub = self.create_publisher(Float64, 'line_error', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.camera_sub = self.create_subscription(
            Image, 'camera_down/image_raw', self.image_callback, 10
        )

        return TransitionCallbackReturn.SUCCESS

    def image_callback(self, msg: Image) -> None:
        pass


def main():
    rclpy.init()
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
