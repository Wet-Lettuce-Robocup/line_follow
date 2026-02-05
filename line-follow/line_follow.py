from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class LineFollowerNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__('line_follower')
        self.vel_publisher = None
