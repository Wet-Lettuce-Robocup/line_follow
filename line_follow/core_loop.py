from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn


class LineFollowerNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__('line_follower')

        self.navigation_client = self.create_client(
            ChangeState, '/line_follow/navigation/'
        )
        self.pid_manager_client = self.create_client(
            ChangeState, '/line_follow/pid_manager/'
        )

    def change_node_state(self, client, transition_id):
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.change_node_state(self.navigation_client, Transition.TRANSITION_ACTIVATE)
        self.change_node_state(self.pid_manager_client, Transition.TRANSITION_ACTIVATE)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.change_node_state(self.navigation_client, Transition.TRANSITION_DEACTIVATE)
        self.change_node_state(
            self.pid_manager_client, Transition.TRANSITION_DEACTIVATE
        )

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
