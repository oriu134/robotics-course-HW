"""Receive camera inputs and highlight candidate red pixels without moving."""

import copy

from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image


def make_red_mask(rgb_pixels: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Highlight pixels passing a simple RGB red-color rule."""
    if (
        rgb_pixels.ndim != 3
        or rgb_pixels.shape[2] != 3
        or rgb_pixels.size == 0
        or rgb_pixels.dtype != np.uint8
    ):
        raise ValueError('Expected a nonempty height-by-width RGB uint8 array.')
    red = rgb_pixels[:, :, 0]
    green = rgb_pixels[:, :, 1]
    blue = rgb_pixels[:, :, 2]
    is_red = (red >= 150) & (green <= 100) & (blue <= 100)
    return is_red.astype(np.uint8) * np.uint8(255)


class CameraListener(Node):
    """Report synchronized camera inputs and candidate red-pixel information."""

    def __init__(self) -> None:
        """Create camera subscriptions, a synchronizer and an RGB decoder."""
        super().__init__('camera_listener')
        self.bridge = CvBridge()
        self.latest_rgb_pixels: NDArray[np.uint8] | None = None
        self.latest_red_mask: NDArray[np.uint8] | None = None
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
        """Decode matched RGB images and highlight their candidate red pixels."""
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
        self.latest_red_mask = None
        self.update_rgb_pixels(rgb)
        if self.latest_rgb_pixels is not None:
            mask = make_red_mask(self.latest_rgb_pixels)
            self.latest_red_mask = mask
            if self.received_pairs == 1 or self.received_pairs % 30 == 0:
                red_count = int(np.count_nonzero(mask))
                message = f'Red mask: shape={mask.shape}, red_pixels={red_count}'
                if red_count == 0:
                    self.get_logger().warning(message + '; no red region in current image.')
                else:
                    self.get_logger().info(message)

    def update_rgb_pixels(self, rgb: Image) -> None:
        """Decode the current RGB image and report one sample pixel."""
        self.latest_rgb_pixels = None
        try:
            if rgb.width == 0 or rgb.height == 0:
                raise ValueError('Image dimensions must be positive.')
            source_dtype, channels = self.bridge.encoding_to_dtype_with_channels(
                rgb.encoding
            )
            row_bytes = rgb.width * np.dtype(source_dtype).itemsize * channels
            if rgb.step < row_bytes:
                raise ValueError('Row stride is smaller than the encoded pixel row.')
            if len(rgb.data) != rgb.height * rgb.step:
                raise ValueError('Image data length does not match height and row stride.')
            image = rgb
            if rgb.step > row_bytes:
                # This CvBridge version misreads padding that is not a whole pixel.
                image = copy.copy(rgb)
                rows = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.step)
                image.data = rows[:, :row_bytes].tobytes()
                image.step = row_bytes
            pixels: NDArray[np.uint8] = self.bridge.imgmsg_to_cv2(
                image, desired_encoding='rgb8'
            )
            if pixels.shape != (rgb.height, rgb.width, 3) or pixels.dtype != np.uint8:
                raise ValueError('Decoded image is not a height-by-width RGB uint8 array.')
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(
                f'Cannot decode RGB image ({rgb.encoding}, '
                f'{rgb.width}x{rgb.height}): {error}'
            )
            return
        self.latest_rgb_pixels = pixels
        if self.received_pairs == 1 or self.received_pairs % 30 == 0:
            row, column = rgb.height // 2, rgb.width // 2
            self.get_logger().info(
                f'RGB array: shape={pixels.shape}, dtype={pixels.dtype}, '
                f'center[row={row}, col={column}]={pixels[row, column].tolist()}'
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
