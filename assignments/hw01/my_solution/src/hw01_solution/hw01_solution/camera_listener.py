"""Locate a red marker and measure its camera-axis depth without moving."""

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
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image


# Sensor clipping limits filter measurements; they are not a marker position.
MIN_CAMERA_DEPTH_METRES = 0.05
MAX_CAMERA_DEPTH_METRES = 3.0


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


@dataclass(frozen=True)
class MarkerDetection:
    """Keep the image-space center and mask of the selected marker region."""

    pixel: tuple[float, float]
    mask: NDArray[np.uint8]


def _bounding_box_rejection(
    stats_row: NDArray[np.int32], image_height: int, image_width: int
) -> str | None:
    """Return a bounding-box rejection reason, or None when its checks pass."""
    x = int(stats_row[cv2.CC_STAT_LEFT])
    y = int(stats_row[cv2.CC_STAT_TOP])
    width = int(stats_row[cv2.CC_STAT_WIDTH])
    height = int(stats_row[cv2.CC_STAT_HEIGHT])
    area = int(stats_row[cv2.CC_STAT_AREA])
    # A border-touching patch may extend outside the image and bias its center.
    if x == 0 or y == 0 or x + width == image_width or y + height == image_height:
        return 'Red region touches the image border.'
    # A round marker should have a roughly equally wide and tall bounding box.
    if min(width, height) / max(width, height) < 0.75:
        return 'Red region is elongated (possible occlusion).'
    # A disk leaves empty box corners; very sparse or fully filled boxes differ.
    fill = area / (width * height)
    if not 0.55 <= fill <= 0.90:
        return 'Red region does not have a filled round shape.'
    return None


def _outline_rejection(region_mask: NDArray[np.uint8]) -> str | None:
    """Return an outline rejection reason, or None when roundness checks pass."""
    contours, _ = cv2.findContours(
        region_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if len(contours) != 1:
        return 'Red region outline is unreliable.'
    perimeter = cv2.arcLength(contours[0], True)
    if perimeter <= 0:
        return 'Red region outline is degenerate.'
    # Circularity uses contour polygon area, not the white-pixel count above.
    # An ideal circle scores 1; the lower bound tolerates pixelated outlines.
    circularity = 4.0 * math.pi * cv2.contourArea(contours[0]) / perimeter**2
    if circularity < 0.70:
        return 'Red region is not round enough (possible occlusion).'
    return None


def detect_marker(mask: NDArray[np.uint8]) -> tuple[MarkerDetection | None, str]:
    """
    Select one plausible round marker from an already computed red mask.

    Color filtering has already happened in make_red_mask. This function groups
    its white pixels, discards small specks, and checks the remaining region's
    geometry. It does not use depth, preset coordinates, or robot coordinates.

    Parameters
    ----------
    mask : numpy.ndarray
        A nonempty (height, width) uint8 array. Values are 255 for candidate
        red pixels and 0 for background. The input array is not modified.

    Returns
    -------
    detection : MarkerDetection or None
        On acceptance, contains the floating-point pixel center (u, v),
        meaning (column, row), and a mask of only the selected component.
        None means a valid mask contains no unique, reliable-looking marker.
    reason : str
        Explains acceptance or what was missing or unreliable. On detection
        failure, the caller reports this reason and clears its previous marker.

    Raises
    ------
    ValueError
        If the input is not a nonempty two-dimensional uint8 mask containing
        only 0 and 255. Malformed input is different from an ordinary missing
        marker in a valid mask.

    Notes
    -----
    The course camera sees one approximately round marker from above. The
    size and shape thresholds are conservative heuristics for that setup,
    not proof that every accepted red object is the intended marker.
    Multiple sizeable components are rejected BEFORE shape checking; this
    function does not test all their shapes and then choose a winner.

    For example, foreground labels 1 and 2 with areas 91 and 3 pixels leave
    regions=[1] after the size filter. Label 1 must still pass the shape
    checks. Areas 91 and 80 leave regions=[1, 2], so the observation is
    rejected as ambiguous rather than selecting the larger component.

    """
    if mask.ndim != 2 or mask.size == 0 or mask.dtype != np.uint8:
        raise ValueError('Expected a nonempty two-dimensional uint8 mask.')
    if np.any((mask != 0) & (mask != 255)):
        raise ValueError('Mask values must be 0 or 255.')
    # Label 0 is background; other labels identify separate connected patches.
    # Eight-connectivity also groups pixels that touch diagonally.
    # Btw its like finding islands leetcode 200. Number of Islands(very similiar)
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count == 1:
        return None, 'No red region.'
    # Keep component LABEL IDs, not pixels or coordinates. Areas count pixels.
    regions = [
        index for index in range(1, count)
        if stats[index, cv2.CC_STAT_AREA] >= 20
    ]
    if not regions:
        return None, 'Red regions are too small.'
    # A larger patch can be a distractor, so size alone cannot resolve ambiguity.
    if len(regions) != 1:
        return None, 'Multiple sizeable red regions; marker is ambiguous.'
    region = regions[0]
    image_height, image_width = mask.shape
    reason = _bounding_box_rejection(stats[region], image_height, image_width)
    if reason is not None:
        return None, reason
    # Exclude every other component so later depth sampling cannot include noise.
    region_mask: NDArray[np.uint8] = (
        (labels == region).astype(np.uint8) * np.uint8(255)
    )
    reason = _outline_rejection(region_mask)
    if reason is not None:
        return None, reason
    # OpenCV already computed each label's mean (x, y): column first, row second.
    # Keep subpixel precision for later reconstruction rather than rounding here.
    u, v = centers[region]
    return MarkerDetection((float(u), float(v)), region_mask), 'Marker region accepted.'


def _packed_image(image: Image, bridge: CvBridge) -> Image:
    """Validate image storage and remove row padding on a copy when needed."""
    if image.width == 0 or image.height == 0:
        raise ValueError('Image dimensions must be positive.')
    source_dtype, channels = bridge.encoding_to_dtype_with_channels(image.encoding)
    row_bytes = image.width * np.dtype(source_dtype).itemsize * channels
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


def measure_marker_depth(
    depth_pixels: NDArray[np.float32], marker_mask: NDArray[np.uint8]
) -> tuple[float | None, str]:
    """
    Estimate marker optical-Z depth from valid pixels in its selected region.

    Background and small red-noise regions are excluded by marker_mask.
    Nonfinite samples and values outside the camera's clipping range are ignored.
    At least half of the region must have valid depth; the median then limits the
    influence of isolated outliers. This is a depth-selection heuristic, not a
    reconstruction of a three-dimensional position.

    Parameters
    ----------
    depth_pixels : numpy.ndarray
        Nonempty, two-dimensional float32 depth values in metres.
    marker_mask : numpy.ndarray
        Same-sized uint8 mask with 255 for selected marker pixels and 0 elsewhere.

    Returns
    -------
    distance : float or None
        Median usable depth along the camera optical Z axis, or None when there
        are no valid samples or less than half the selected region is usable.
    reason : str
        Explanation of acceptance or missing/unreliable depth.

    Raises
    ------
    ValueError
        If the arrays have invalid types, shapes or an empty marker selection.

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
    valid = (
        np.isfinite(samples)
        & (samples >= MIN_CAMERA_DEPTH_METRES)
        & (samples <= MAX_CAMERA_DEPTH_METRES)
    )
    valid_count = int(np.count_nonzero(valid))
    if valid_count == 0:
        return None, 'No valid marker depth samples.'
    if valid_count * 2 < samples.size:
        return None, 'Less than half of the marker has valid depth.'
    return float(np.median(samples[valid])), 'Marker depth accepted.'


class CameraListener(Node):
    """Report synchronized marker pixel centers and camera-axis depth."""

    def __init__(self) -> None:
        """Create camera subscriptions, a synchronizer and an RGB decoder."""
        super().__init__('camera_listener')
        self.bridge = CvBridge()
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
        """Locate a marker in matched images and estimate its optical-Z depth."""
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
        self.latest_marker = None
        self.latest_depth_pixels = None
        self.latest_marker_depth = None
        self.update_rgb_pixels(rgb)
        if self.latest_rgb_pixels is not None:
            self.update_depth_pixels(depth, rgb)
            mask = make_red_mask(self.latest_rgb_pixels)
            self.latest_red_mask = mask
            if self.received_pairs == 1 or self.received_pairs % 30 == 0:
                red_count = int(np.count_nonzero(mask))
                message = f'Red mask: shape={mask.shape}, red_pixels={red_count}'
                if red_count == 0:
                    self.get_logger().warning(message + '; no red region in current image.')
                else:
                    self.get_logger().info(message)
            detection, reason = detect_marker(mask)
            self.latest_marker = detection
            if (
                self.received_pairs == 1
                or self.received_pairs % 30 == 0
                or reason != self._marker_reason
            ):
                if detection is None:
                    self.get_logger().warning('Marker not detected: ' + reason)
                else:
                    u, v = detection.pixel
                    area = int(np.count_nonzero(detection.mask))
                    self.get_logger().info(
                        f'Marker pixel: u={u:.2f}, v={v:.2f}, region_pixels={area}'
                    )
            self._marker_reason = reason
            if detection is not None:
                self.update_marker_depth(detection.mask)
            else:
                self._depth_reason = 'Marker unavailable.'
        else:
            self._marker_reason = 'RGB decoding failed.'
            self._depth_reason = 'RGB decoding failed.'

    def update_rgb_pixels(self, rgb: Image) -> None:
        """Decode the current RGB image and report one sample pixel."""
        self.latest_rgb_pixels = None
        try:
            image = _packed_image(rgb, self.bridge)
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

    def update_depth_pixels(self, depth: Image, rgb: Image) -> None:
        """Decode depth only when its format, dimensions and frame match RGB."""
        self.latest_depth_pixels = None
        try:
            if depth.encoding != '32FC1':
                raise ValueError('Expected 32FC1 depth values in metres.')
            if (depth.width, depth.height) != (rgb.width, rgb.height):
                raise ValueError('Depth and RGB dimensions differ.')
            if not rgb.header.frame_id or depth.header.frame_id != rgb.header.frame_id:
                raise ValueError('Depth must share the nonempty RGB optical frame.')
            image = _packed_image(depth, self.bridge)
            pixels: NDArray[np.float32] = self.bridge.imgmsg_to_cv2(
                image, desired_encoding='passthrough'
            )
            if pixels.shape != (rgb.height, rgb.width) or pixels.dtype != np.float32:
                raise ValueError('Decoded depth is not a height-by-width float32 array.')
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(
                f'Cannot decode aligned depth image ({depth.encoding}, '
                f'{depth.width}x{depth.height}): {error}'
            )
            return
        self.latest_depth_pixels = pixels
        if self.received_pairs == 1 or self.received_pairs % 30 == 0:
            self.get_logger().info(
                f'Depth array: shape={pixels.shape}, dtype={pixels.dtype}, unit=metres'
            )

    def update_marker_depth(self, marker_mask: NDArray[np.uint8]) -> None:
        """Report a current marker-depth estimate without retaining old values."""
        self.latest_marker_depth = None
        pixels = self.latest_depth_pixels
        if pixels is None:
            # The depth decoder already reported why this image is unavailable.
            self._depth_reason = 'Depth image unavailable.'
            return
        distance, reason = measure_marker_depth(pixels, marker_mask)
        self.latest_marker_depth = distance
        if (
            self.received_pairs == 1
            or self.received_pairs % 30 == 0
            or reason != self._depth_reason
        ):
            if distance is None:
                self.get_logger().warning('Marker depth unavailable: ' + reason)
            else:
                self.get_logger().info(f'Marker depth: {distance:.6f}m')
        self._depth_reason = reason

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
