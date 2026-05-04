import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.time import Time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class PIDManager(LifecycleNode):
    def __init__(self):
        super().__init__('pid_manager')

        self.declare_parameter('kp', 0.1)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('default_speed', 0.05)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.default_speed = (
            self.get_parameter('default_speed').get_parameter_value().double_value
        )

        self.integral: float = 0.0
        self.last_error: float = 0.0
        self.last_time: Time | None = None

        self.error_sub: Subscription | None = None
        self.twist_pub: Publisher | None = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.error_sub = self.create_subscription(
            Float64, 'line_error', self.error_callback, 10
        )
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        return super().on_cleanup(state)

    def error_callback(self, msg: Float64):
        if self.twist_pub is None:
            return

        current_time: Time = self.get_clock().now()
        error: float = msg.data
        derivative: float = 0.0

        if self.last_time is not None:
            dt: float = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9

            if dt <= 0:
                return

            self.integral += dt * error
            derivative = (error - self.last_error) / dt

        self.last_error = error
        self.last_time = self.get_clock().now()

        twist_msg = Twist()
        twist_msg.linear.x = self.default_speed
        twist_msg.angular.z = (
            self.kp * error + self.ki * self.integral + self.kd * derivative
        )

        self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
