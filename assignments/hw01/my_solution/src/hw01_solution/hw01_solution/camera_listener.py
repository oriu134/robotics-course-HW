"""Find the red marker in synchronized RGB/depth images and measure its depth."""

import copy
from dataclasses import dataclass
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


# Camera clipping range: depth outside it is a bad measurement, not a position.
MIN_CAMERA_DEPTH_METRES = 0.05
MAX_CAMERA_DEPTH_METRES = 3.0
MIN_REGION_PIXELS = 20  # smaller red patches are treated as noise
SYNC_SLOP_SECONDS = 0.05  # RGB/depth stamps must differ by less than this
LOG_EVERY = 30  # repeated messages are logged on the first and every 30th time


def _should_log(count: int) -> bool:
    return count == 1 or count % LOG_EVERY == 0


# ----- detection (pure functions, no ROS) -----------------------------------

def make_red_mask(rgb_pixels: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Return 255 where a pixel is clearly red (R >= 150, G <= 100, B <= 100)."""
    if (
        rgb_pixels.ndim != 3
        or rgb_pixels.shape[2] != 3
        or rgb_pixels.size == 0
        or rgb_pixels.dtype != np.uint8
    ):
        raise ValueError('Expected a nonempty height-by-width RGB uint8 array.')
    red, green, blue = (rgb_pixels[:, :, i] for i in range(3))
    is_red = (red >= 150) & (green <= 100) & (blue <= 100)
    return is_red.astype(np.uint8) * np.uint8(255)


@dataclass(frozen=True)
class MarkerDetection:
    """Subpixel center (u=column, v=row) and the mask of the chosen region."""

    pixel: tuple[float, float]
    mask: NDArray[np.uint8]


def _bounding_box_rejection(
    stats_row: NDArray[np.int32], image_height: int, image_width: int
) -> str | None:
    """Return why the region's bounding box looks wrong, or None if it is fine."""
    x, y, width, height, area = (int(value) for value in stats_row[:5])
    # A region cut by the image edge would have a biased center.
    if x == 0 or y == 0 or x + width == image_width or y + height == image_height:
        return 'Red region touches the image border.'
    # A round marker has a roughly square box; a covered one gets elongated.
    if min(width, height) / max(width, height) < 0.75:
        return 'Red region is elongated (possible occlusion).'
    # A disk fills about pi/4 = 0.79 of its box.
    if not 0.55 <= area / (width * height) <= 0.90:
        return 'Red region does not have a filled round shape.'
    return None


def _outline_rejection(region_mask: NDArray[np.uint8]) -> str | None:
    """Return why the region's outline is not round, or None if it is fine."""
    contours, _ = cv2.findContours(region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) != 1:
        return 'Red region outline is unreliable.'
    perimeter = cv2.arcLength(contours[0], True)
    if perimeter <= 0:
        return 'Red region outline is degenerate.'
    # Circularity 4*pi*A/P^2 is 1 for a perfect circle; 0.50 allows pixelation.
    if 4.0 * math.pi * cv2.contourArea(contours[0]) / perimeter**2 < 0.50:
        return 'Red region is not round enough (possible occlusion).'
    return None


def detect_marker(mask: NDArray[np.uint8]) -> tuple[MarkerDetection | None, str]:
    """
    Pick the single round red region in a 0/255 mask.

    Returns (detection, reason). detection is None when there is no red region,
    only tiny specks, more than one sizeable region (ambiguous, we never guess by
    size), or the one region fails the shape checks (border, elongated, not
    filled, not round). Raises ValueError for a malformed mask.
    """
    if mask.ndim != 2 or mask.size == 0 or mask.dtype != np.uint8:
        raise ValueError('Expected a nonempty two-dimensional uint8 mask.')
    if np.any((mask != 0) & (mask != 255)):
        raise ValueError('Mask values must be 0 or 255.')
    # Label connected red patches (like "Number of Islands"); label 0 is background.
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count == 1:
        return None, 'No red region.'
    regions = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= MIN_REGION_PIXELS]
    if not regions:
        return None, 'Red regions are too small.'
    if len(regions) != 1:
        return None, 'Multiple sizeable red regions; marker is ambiguous.'
    region = regions[0]
    reason = _bounding_box_rejection(stats[region], *mask.shape)
    if reason is not None:
        return None, reason
    # Keep only this region so depth sampling never includes red noise.
    region_mask = (labels == region).astype(np.uint8) * np.uint8(255)
    reason = _outline_rejection(region_mask)
    if reason is not None:
        return None, reason
    u, v = centers[region]
    return MarkerDetection((float(u), float(v)), region_mask), 'Marker region accepted.'


def measure_marker_depth(
    depth_pixels: NDArray[np.float32], marker_mask: NDArray[np.uint8]
) -> tuple[float | None, str]:
    """
    Return the median valid optical-Z depth inside the marker mask.

    Valid means finite and inside the camera clip range. At least half of the
    marker pixels must be valid; otherwise (None, reason) is returned. The median
    ignores a few bad edge pixels. Raises ValueError for malformed arrays.
    """
    if depth_pixels.ndim != 2 or depth_pixels.size == 0 or depth_pixels.dtype != np.float32:
        raise ValueError('Expected a nonempty two-dimensional float32 depth array.')
    if marker_mask.shape != depth_pixels.shape or marker_mask.dtype != np.uint8:
        raise ValueError('Marker mask must be a same-sized uint8 array.')
    if np.any((marker_mask != 0) & (marker_mask != 255)):
        raise ValueError('Marker mask values must be 0 or 255.')
    samples = depth_pixels[marker_mask != 0]
    if samples.size == 0:
        raise ValueError('Selected marker mask must contain pixels.')
    valid = samples[
        np.isfinite(samples)
        & (samples >= MIN_CAMERA_DEPTH_METRES)
        & (samples <= MAX_CAMERA_DEPTH_METRES)
    ]
    if valid.size == 0:
        return None, 'No valid marker depth samples.'
    if valid.size * 2 < samples.size:
        return None, 'Less than half of the marker has valid depth.'
    return float(np.median(valid)), 'Marker depth accepted.'


# ----- image decoding -------------------------------------------------------

def _packed_image(image: Image, bridge: CvBridge) -> Image:
    """Validate the image size fields and drop row padding (on a copy) if any."""
    if image.width == 0 or image.height == 0:
        raise ValueError('Image dimensions must be positive.')
    dtype, channels = bridge.encoding_to_dtype_with_channels(image.encoding)
    row_bytes = image.width * np.dtype(dtype).itemsize * channels
    if image.step < row_bytes:
        raise ValueError('Row stride is smaller than the encoded pixel row.')
    if len(image.data) != image.height * image.step:
        raise ValueError('Image data length does not match height and row stride.')
    if image.step == row_bytes:
        return image
    # This CvBridge version misreads padding that is not a whole pixel.
    packed = copy.copy(image)
    rows = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
    packed.data = rows[:, :row_bytes].tobytes()
    packed.step = row_bytes
    return packed


def _check_depth_matches(depth: Image, rgb: Image) -> None:
    """Raise ValueError unless depth is 32FC1 and aligned with the RGB image."""
    if depth.encoding != '32FC1':
        raise ValueError('Expected 32FC1 depth values in metres.')
    if (depth.width, depth.height) != (rgb.width, rgb.height):
        raise ValueError('Depth and RGB dimensions differ.')
    if not rgb.header.frame_id or depth.header.frame_id != rgb.header.frame_id:
        raise ValueError('Depth must share the nonempty RGB optical frame.')


# ----- ROS node -------------------------------------------------------------

class CameraListener(Node):
    """Synchronize RGB/depth, then store the marker pixel center and depth."""

    def __init__(self, node_name: str = 'camera_listener', **node_options) -> None:
        """Subscribe to RGB, depth (synchronized) and camera info."""
        super().__init__(node_name, **node_options)
        self.bridge = CvBridge()
        # Results of the latest pair; all reset to None at the start of each pair.
        self.latest_rgb_pixels: NDArray[np.uint8] | None = None
        self.latest_depth_pixels: NDArray[np.float32] | None = None
        self.latest_red_mask: NDArray[np.uint8] | None = None
        self.latest_marker: MarkerDetection | None = None
        self.latest_marker_depth: float | None = None
        self._marker_reason: str | None = None
        self._depth_reason: str | None = None
        self.received_images = 0
        self.received_depth_images = 0
        self.received_camera_info = 0
        self.received_pairs = 0

        qos = qos_profile_sensor_data  # best effort, as the camera publishes
        self.image_subscription = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos)
        self.depth_subscription = Subscriber(
            self, Image, '/camera/depth/image_raw', qos_profile=qos
        )
        self.image_subscription.registerCallback(self.on_image)
        self.depth_subscription.registerCallback(self.on_depth_image)
        # Pairs RGB and depth by header stamp (capture time), not arrival time.
        self.image_synchronizer = ApproximateTimeSynchronizer(
            [self.image_subscription, self.depth_subscription],
            queue_size=10,
            slop=SYNC_SLOP_SECONDS,
        )
        self.image_synchronizer.registerCallback(self.on_synchronized_images)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera_info, qos)
        self.get_logger().info('Waiting for RGB, depth and camera calibration.')

    # ----- raw stream callbacks (logging only) -----

    def on_image(self, message: Image) -> None:
        """Count RGB images."""
        self.received_images += 1
        self.log_image(message, 'image', self.received_images)

    def on_depth_image(self, message: Image) -> None:
        """Count depth images."""
        self.received_depth_images += 1
        self.log_image(message, 'depth image', self.received_depth_images)

    def on_camera_info(self, message: CameraInfo) -> None:
        """Log calibration occasionally."""
        self.received_camera_info += 1
        if _should_log(self.received_camera_info):
            k = message.k
            self.get_logger().info(
                f'Camera info #{self.received_camera_info}: {message.width}x{message.height}, '
                f'fx={k[0]:.2f}, fy={k[4]:.2f}, cx={k[2]:.2f}, cy={k[5]:.2f}, '
                f'frame={message.header.frame_id}'
            )

    def log_image(self, message: Image, label: str, count: int) -> None:
        """Log image metadata on the first and every 30th message."""
        if _should_log(count):
            stamp = message.header.stamp
            self.get_logger().info(
                f'Received {label} #{count}: {message.width}x{message.height}, '
                f'encoding={message.encoding}, frame={message.header.frame_id}, '
                f'stamp={stamp.sec}.{stamp.nanosec:09d}'
            )

    # ----- synchronized pipeline -----

    def on_synchronized_images(self, rgb: Image, depth: Image) -> None:
        """Decode a matched pair, find the marker and measure its depth."""
        self.received_pairs += 1
        self.latest_red_mask = None
        self.latest_marker = None
        self.latest_depth_pixels = None
        self.latest_marker_depth = None
        self.update_rgb_pixels(rgb)
        if self.latest_rgb_pixels is None:
            self._marker_reason = self._depth_reason = 'RGB decoding failed.'
            return
        self.update_depth_pixels(depth, rgb)
        self.latest_red_mask = make_red_mask(self.latest_rgb_pixels)
        self.update_marker(self.latest_red_mask)
        if self.latest_marker is None:
            self._depth_reason = 'Marker unavailable.'
        else:
            self.update_marker_depth(self.latest_marker.mask)

    def update_rgb_pixels(self, rgb: Image) -> None:
        """Decode RGB into a (height, width, 3) uint8 array, or None on error."""
        self.latest_rgb_pixels = None
        try:
            pixels = self.bridge.imgmsg_to_cv2(
                _packed_image(rgb, self.bridge), desired_encoding='rgb8'
            )
            if pixels.shape != (rgb.height, rgb.width, 3) or pixels.dtype != np.uint8:
                raise ValueError('Decoded image is not a height-by-width RGB uint8 array.')
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(f'Cannot decode RGB image ({rgb.encoding}): {error}')
            return
        self.latest_rgb_pixels = pixels

    def update_depth_pixels(self, depth: Image, rgb: Image) -> None:
        """Decode aligned depth into a (height, width) float32 array, or None."""
        self.latest_depth_pixels = None
        try:
            _check_depth_matches(depth, rgb)
            pixels = self.bridge.imgmsg_to_cv2(
                _packed_image(depth, self.bridge), desired_encoding='passthrough'
            )
            if pixels.shape != (rgb.height, rgb.width) or pixels.dtype != np.float32:
                raise ValueError('Decoded depth is not a height-by-width float32 array.')
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(
                f'Cannot decode aligned depth image ({depth.encoding}): {error}'
            )
            return
        self.latest_depth_pixels = pixels

    def update_marker(self, mask: NDArray[np.uint8]) -> None:
        """Detect the marker and log when the result changes."""
        detection, reason = detect_marker(mask)
        self.latest_marker = detection
        if reason != self._marker_reason or _should_log(self.received_pairs):
            if detection is None:
                self.get_logger().warning('Marker not detected: ' + reason)
            else:
                u, v = detection.pixel
                area = int(np.count_nonzero(detection.mask))
                self.get_logger().info(f'Marker pixel: u={u:.2f}, v={v:.2f}, pixels={area}')
        self._marker_reason = reason

    def update_marker_depth(self, marker_mask: NDArray[np.uint8]) -> None:
        """Measure marker depth (never reusing an old value) and log changes."""
        self.latest_marker_depth = None
        if self.latest_depth_pixels is None:
            self._depth_reason = 'Depth image unavailable.'  # decoder already logged why
            return
        distance, reason = measure_marker_depth(self.latest_depth_pixels, marker_mask)
        self.latest_marker_depth = distance
        if reason != self._depth_reason or _should_log(self.received_pairs):
            if distance is None:
                self.get_logger().warning('Marker depth unavailable: ' + reason)
            else:
                self.get_logger().info(f'Marker depth: {distance:.6f}m')
        self._depth_reason = reason


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
