#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


WHEEL_NAMES = (
    'front_left',
    'front_right',
    'back_left',
    'back_right',
)


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def optimize_steering(
    target_angle: float,
    wheel_speed: float,
    current_angle: float,
) -> Tuple[float, float]:
    target_angle = current_angle + wrap_to_pi(target_angle - current_angle)
    if abs(wrap_to_pi(target_angle - current_angle)) > math.pi / 2.0:
        target_angle = current_angle + wrap_to_pi(
            target_angle + math.pi - current_angle
        )
        wheel_speed = -wheel_speed
    return target_angle, wheel_speed


class RbvoguiSwerveController(Node):
    def __init__(self) -> None:
        super().__init__('rbvogui_swerve_controller')

        self.declare_parameter(
            'cmd_vel_topic',
            '/robot/robotnik_base_control/cmd_vel_unstamped',
        )
        self.declare_parameter(
            'steering_command_topic',
            '/robot/steering_position_controller/commands',
        )
        self.declare_parameter(
            'wheel_command_topic',
            '/robot/wheel_velocity_controller/commands',
        )
        self.declare_parameter('joint_states_topic', '/robot/joint_states')
        self.declare_parameter('joint_prefix', 'robot_')
        self.declare_parameter('wheel_offset_x', 0.368)
        self.declare_parameter('wheel_offset_y', 0.235)
        self.declare_parameter('wheel_radius', 0.1165)
        self.declare_parameter('max_wheel_speed', 6.0)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('stationary_speed_threshold', 1.0e-4)

        self.joint_prefix = str(self.get_parameter('joint_prefix').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_wheel_speed = float(
            self.get_parameter('max_wheel_speed').value
        )
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        self.stationary_speed_threshold = float(
            self.get_parameter('stationary_speed_threshold').value
        )

        offset_x = float(self.get_parameter('wheel_offset_x').value)
        offset_y = float(self.get_parameter('wheel_offset_y').value)
        self.wheel_positions = (
            (offset_x, offset_y),
            (offset_x, -offset_y),
            (-offset_x, offset_y),
            (-offset_x, -offset_y),
        )

        self.current_steering: Dict[str, float] = {}
        self.last_steering = [0.0] * len(WHEEL_NAMES)
        self.command = Twist()
        self.last_command_time = None

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('steering_command_topic').value),
            10,
        )
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('wheel_command_topic').value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self._command_cb,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter('joint_states_topic').value),
            self._joint_state_cb,
            10,
        )

        control_rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / control_rate, self._update)
        self.get_logger().info(
            'Using standard joint controllers for RB-VOGUI swerve commands'
        )

    def _command_cb(self, msg: Twist) -> None:
        self.command = msg
        self.last_command_time = self.get_clock().now()

    def _joint_state_cb(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name.endswith('_steering_joint'):
                self.current_steering[name] = position

    def _command_is_fresh(self) -> bool:
        if self.last_command_time is None:
            return False
        age = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        return age <= self.command_timeout

    def _update(self) -> None:
        if self._command_is_fresh():
            vx = self.command.linear.x
            vy = self.command.linear.y
            wz = self.command.angular.z
        else:
            vx = 0.0
            vy = 0.0
            wz = 0.0

        steering_commands: List[float] = []
        wheel_commands: List[float] = []

        for index, (name, (wheel_x, wheel_y)) in enumerate(
            zip(WHEEL_NAMES, self.wheel_positions)
        ):
            wheel_vx = vx - wz * wheel_y
            wheel_vy = vy + wz * wheel_x
            linear_speed = math.hypot(wheel_vx, wheel_vy)

            joint_name = f'{self.joint_prefix}{name}_steering_joint'
            current_angle = self.current_steering.get(
                joint_name,
                self.last_steering[index],
            )

            if linear_speed <= self.stationary_speed_threshold:
                target_angle = self.last_steering[index]
                wheel_speed = 0.0
            else:
                target_angle = math.atan2(wheel_vy, wheel_vx)
                wheel_speed = linear_speed / self.wheel_radius
                target_angle, wheel_speed = optimize_steering(
                    target_angle,
                    wheel_speed,
                    current_angle,
                )

            wheel_speed = max(
                -self.max_wheel_speed,
                min(self.max_wheel_speed, wheel_speed),
            )
            self.last_steering[index] = target_angle
            steering_commands.append(target_angle)
            wheel_commands.append(wheel_speed)

        steering_msg = Float64MultiArray()
        steering_msg.data = steering_commands
        self.steering_pub.publish(steering_msg)

        wheel_msg = Float64MultiArray()
        wheel_msg.data = wheel_commands
        self.wheel_pub.publish(wheel_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[RbvoguiSwerveController] = None
    try:
        node = RbvoguiSwerveController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
