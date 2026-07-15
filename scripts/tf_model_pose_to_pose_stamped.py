#!/usr/bin/env python3

from copy import deepcopy
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage


class TfModelPoseToPoseStamped(Node):
    def __init__(self) -> None:
        super().__init__('tf_model_pose_to_pose_stamped')

        self.declare_parameter('input_topic', '/model/robot/pose')
        self.declare_parameter('output_topic', '/robot_pose')
        self.declare_parameter('model_frame', 'robot')
        self.declare_parameter('world_frame', 'robotnik_simple')
        self.declare_parameter('output_frame', '')
        self.declare_parameter('fallback_transform_index', 0)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('tf_child_frame', '')

        self.model_frame = str(self.get_parameter('model_frame').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.output_frame = str(self.get_parameter('output_frame').value).strip()
        self.fallback_transform_index = int(
            self.get_parameter('fallback_transform_index').value
        )
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.tf_child_frame = str(self.get_parameter('tf_child_frame').value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None
        )
        self.create_subscription(TFMessage, input_topic, self._tf_cb, 10)

        self.get_logger().info(
            f'Publishing {output_topic} from {input_topic} transform '
            f'{self.world_frame} -> {self.model_frame}'
        )

    def _tf_cb(self, msg: TFMessage) -> None:
        transform = self._find_model_transform(msg)
        if transform is None:
            return

        pose = PoseStamped()
        pose.header = transform.header
        # Gazebo's model-pose stream may carry a zero stamp.  /robot_pose is
        # an observed pose, so publish it with the current simulation time;
        # consumers can then reject stale samples deterministically.
        pose.header.stamp = self.get_clock().now().to_msg()
        if not pose.header.frame_id:
            pose.header.frame_id = self.world_frame
        if self.output_frame:
            pose.header.frame_id = self.output_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.publisher.publish(pose)

        if self.tf_broadcaster is not None:
            tf_transform = deepcopy(transform)
            tf_transform.header.stamp = self.get_clock().now().to_msg()
            if not tf_transform.header.frame_id:
                tf_transform.header.frame_id = self.world_frame
            if self.output_frame:
                tf_transform.header.frame_id = self.output_frame
            if self.tf_child_frame:
                tf_transform.child_frame_id = self.tf_child_frame
            self.tf_broadcaster.sendTransform(tf_transform)

    def _find_model_transform(self, msg: TFMessage):
        for transform in msg.transforms:
            child = transform.child_frame_id
            parent = transform.header.frame_id
            if child == self.model_frame and (
                not self.world_frame or parent == self.world_frame
            ):
                return transform

        # Some Gazebo conversions include scoped frame names. Keep this as a
        # fallback, but still prefer the exact model frame above.
        scoped_suffix = f'::{self.model_frame}'
        for transform in msg.transforms:
            child = transform.child_frame_id
            parent = transform.header.frame_id
            if child.endswith(scoped_suffix) and (
                not self.world_frame or parent == self.world_frame
            ):
                return transform
        if (
            self.fallback_transform_index >= 0
            and self.fallback_transform_index < len(msg.transforms)
        ):
            transform = msg.transforms[self.fallback_transform_index]
            if not transform.child_frame_id and not transform.header.frame_id:
                transform.header.frame_id = self.world_frame
                transform.child_frame_id = self.model_frame
                return transform
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[TfModelPoseToPoseStamped] = None
    try:
        node = TfModelPoseToPoseStamped()
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
