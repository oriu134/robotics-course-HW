"""Receive and synchronize HW1 camera inputs without processing or moving."""

from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image


class CameraListener(Node):
    """Report camera metadata and timestamp-matched RGB/depth pairs."""

    def __init__(self) -> None:
        """Create RGB, depth and calibration subscriptions."""
        super().__init__('camera_listener')
        self.received_images = 0
        self.received_depth_images = 0
        self.received_camera_info = 0
        self.received_pairs = 0
        self.image_subscription = Subscriber(
            self,
            Image,
            '/camera/image_raw',
            qos_profile=qos_profile_sensor_data,
        )
        self.image_subscription.registerCallback(self.on_image)
        self.depth_subscription = Subscriber(
            self,
            Image,
            '/camera/depth/image_raw',
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_subscription.registerCallback(self.on_depth_image)
        self.image_synchronizer = ApproximateTimeSynchronizer(
            [self.image_subscription, self.depth_subscription],
            queue_size=10,
            slop=0.05,
        )
        self.image_synchronizer.registerCallback(self.on_synchronized_images)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'Waiting for RGB, depth and camera calibration; start the HW1 simulation.'
        )

    def on_image(self, message: Image) -> None:
        """Count incoming RGB images and report their metadata."""
        self.received_images += 1
        self.log_image(message, 'image', self.received_images)

    def on_depth_image(self, message: Image) -> None:
        """Count incoming depth images and report their metadata."""
        self.received_depth_images += 1
        self.log_image(message, 'depth image', self.received_depth_images)

    def on_synchronized_images(self, rgb: Image, depth: Image) -> None:
        """Report pairs whose capture timestamps satisfy the timing tolerance."""
        self.received_pairs += 1
        if self.received_pairs == 1 or self.received_pairs % 30 == 0:
            rgb_ns = Time.from_msg(rgb.header.stamp).nanoseconds
            depth_ns = Time.from_msg(depth.header.stamp).nanoseconds
            gap_seconds = abs(rgb_ns - depth_ns) / 1_000_000_000
            self.get_logger().info(
                f'Synchronized pair #{self.received_pairs}: '
                f'RGB stamp={rgb.header.stamp.sec}.{rgb.header.stamp.nanosec:09d}, '
                f'depth stamp={depth.header.stamp.sec}.{depth.header.stamp.nanosec:09d}, '
                f'delta={gap_seconds:.9f}s'
            )

    def log_image(self, message: Image, label: str, count: int) -> None:
        """Log the first image and every 30th image from either stream."""
        if count == 1 or count % 30 == 0:
            stamp = message.header.stamp
            self.get_logger().info(
                f'Received {label} #{count}: '
                f'{message.width}x{message.height}, '
                f'encoding={message.encoding}, '
                f'frame={message.header.frame_id}, '
                f'stamp={stamp.sec}.{stamp.nanosec:09d}'
            )

    def on_camera_info(self, message: CameraInfo) -> None:
        """Report the calibration needed for later pixel-to-3D reconstruction."""
        self.received_camera_info += 1
        if self.received_camera_info == 1 or self.received_camera_info % 30 == 0:
            stamp = message.header.stamp
            self.get_logger().info(
                f'Received camera info #{self.received_camera_info}: '
                f'{message.width}x{message.height}, '
                f'fx={message.k[0]:.2f}, fy={message.k[4]:.2f}, '
                f'cx={message.k[2]:.2f}, cy={message.k[5]:.2f}, '
                f'frame={message.header.frame_id}, '
                f'stamp={stamp.sec}.{stamp.nanosec:09d}'
            )


def main(args=None) -> None:
    """Run the camera listener until interrupted."""
    rclpy.init(args=args)
    node = CameraListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
