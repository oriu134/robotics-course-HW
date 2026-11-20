"""ROS planning messages and headless raw-map/configuration-space figures."""

from copy import deepcopy
from pathlib import Path as FilePath

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
import numpy as np
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from .cspace import PLANNING_RADIUS
from .grid import GridMap
from .roadmap import Roadmap
from .search import Route


def _segments(graph: Roadmap | None):
    if graph is None:
        return []
    return [(graph.points[index], graph.points[other])
            for index, edges in enumerate(graph.adjacency)
            for other, _ in edges if index < other]


def make_messages(
    info: MapMetaData | None, cspace: GridMap | None, graph: Roadmap | None,
    route: Route | None, stamp: Time,
) -> tuple[OccupancyGrid, MarkerArray, Path]:
    """Build a complete latched snapshot, including explicit stale-result clears."""
    header = Header(frame_id='map', stamp=stamp)
    occupancy = OccupancyGrid(header=header)
    if cspace is not None:
        received = None if info is None else (
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y)
        expected = (cspace.width, cspace.height, cspace.resolution,
                    cspace.origin_x, cspace.origin_y)
        if received != expected:
            raise ValueError('C-space metadata does not match the received map')
    if info is not None:
        occupancy.info = deepcopy(info)
        # An invalid map must not leave the last free-space image visible.
        occupancy.data = list(cspace.data) if cspace else [100] * (info.width * info.height)
    markers = [Marker(header=header, action=Marker.DELETEALL)]
    if graph is not None:
        for marker_id, kind, color in (
            (0, Marker.POINTS, ColorRGBA(r=0.1, g=0.35, b=0.9, a=0.8)),
            (1, Marker.LINE_LIST, ColorRGBA(r=0.1, g=0.35, b=0.9, a=0.15)),
        ):
            marker = Marker(
                header=header, ns='hw02_prm', id=marker_id, type=kind, action=Marker.ADD)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03 if kind == Marker.POINTS else 0.006
            marker.scale.y = 0.03
            marker.color = color
            points = graph.points if kind == Marker.POINTS else (
                point for segment in _segments(graph) for point in segment)
            marker.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in points]
            markers.append(marker)
        for index, (x, y) in enumerate(graph.points[:2]):
            marker = Marker(
                header=header, ns='hw02_targets', id=index, type=Marker.SPHERE,
                action=Marker.ADD)
            marker.pose.position = Point(x=float(x), y=float(y), z=0.04)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.12
            marker.color = (ColorRGBA(r=0.1, g=0.8, b=0.2, a=1.0) if index == 0
                            else ColorRGBA(r=0.9, g=0.15, b=0.1, a=1.0))
            markers.append(marker)
    path = Path(header=header)
    if route is not None:
        for x, y in route.points:
            pose = PoseStamped(header=header)
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
    return occupancy, MarkerArray(markers=markers), path


def _png_path(filename: str) -> FilePath:
    output = FilePath(filename)
    if output.suffix.lower() != '.png':
        raise ValueError('Figure path must end in .png')
    return output


def remove_figure(filename: str) -> None:
    """Remove a previous snapshot when its plan becomes invalid."""
    if filename:
        _png_path(filename).unlink(missing_ok=True)


def save_figure(
    filename: str, raw: GridMap, cspace: GridMap, graph: Roadmap,
    route: Route | None, seed: int, status: str,
) -> FilePath:
    """Save a correctly oriented PNG with no invented route for no-path results."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    output = _png_path(filename)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(1, 2, figsize=(12, 5), sharex=True, sharey=True)
    try:
        for axis, grid, title in zip(
            axes, (raw, cspace),
            ('Physical map', f'Configuration space - radius {PLANNING_RADIUS:.2f} m'),
        ):
            cells = np.asarray(grid.data).reshape(grid.height, grid.width) != 0
            extent = (grid.origin_x, grid.origin_x + grid.width * grid.resolution,
                      grid.origin_y, grid.origin_y + grid.height * grid.resolution)
            axis.imshow(cells, origin='lower', extent=extent, cmap='Greys', vmin=0, vmax=1)
            axis.set(title=title, xlabel='map X (m)', ylabel='map Y (m)')
            axis.set_aspect('equal')
        points = np.asarray(graph.points)
        axes[1].add_collection(LineCollection(
            _segments(graph), colors='royalblue', linewidths=0.35, alpha=0.12,
            label='Roadmap edges'))
        axes[1].scatter(points[:, 0], points[:, 1], s=3, c='royalblue', alpha=0.6,
                        label='Samples')
        if route is not None:
            selected = np.asarray(route.points)
            axes[1].plot(selected[:, 0], selected[:, 1], color='darkorange', linewidth=2.5,
                         label='Selected path', zorder=4)
        for axis in axes:
            axis.scatter(*graph.points[0], s=70, c='limegreen', edgecolors='black',
                         label='Start', zorder=5)
            axis.scatter(*graph.points[1], s=120, c='red', marker='*', edgecolors='black',
                         label='Goal', zorder=5)
        handles, labels = axes[1].get_legend_handles_labels()
        figure.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, 0.01),
                      ncol=5, fontsize=8)
        figure.suptitle(
            f'PRM/A* - seed {seed} - {status} - '
            f'{len(graph.points)} vertices / {graph.edge_count} edges')
        figure.tight_layout(rect=(0, 0.10, 1, 0.94))
        figure.savefig(output, dpi=150)
    finally:
        plt.close(figure)
    return output.resolve()
