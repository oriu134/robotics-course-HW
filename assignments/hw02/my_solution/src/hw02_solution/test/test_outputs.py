"""Focused tests of visualization messages and saved figure semantics."""

from dataclasses import replace
import math

from builtin_interfaces.msg import Time
from hw02_solution import outputs
from hw02_solution.cspace import inflate
from hw02_solution.grid import GridMap
from hw02_solution.roadmap import Roadmap
from hw02_solution.search import Route
from nav_msgs.msg import MapMetaData
import pytest
from visualization_msgs.msg import Marker


@pytest.fixture
def example():
    raw = GridMap(20, 20, 0.1, 1.0, -2.0, (0,) * 400)
    space = inflate(raw)
    points = ((1.65, -1.35), (2.35, -0.65))
    length = math.dist(*points)
    graph = Roadmap(points, (((1, length),), ((0, length),)), 0, 1)
    info = MapMetaData(width=20, height=20, resolution=0.1)
    info.origin.position.x, info.origin.position.y = 1.0, -2.0
    info.origin.orientation.w = -1.0
    info.map_load_time.sec = 17
    return raw, space, graph, Route(points, length), info


def test_complete_message_frames_metadata_and_graph_shape(example):
    _, space, graph, route, info = example
    occupancy, markers, path = outputs.make_messages(info, space, graph, route, Time(sec=23))
    assert occupancy.info == info and occupancy.info is not info
    assert tuple(occupancy.data) == space.data
    assert occupancy.header.frame_id == path.header.frame_id == 'map'
    assert occupancy.header.stamp.sec == path.header.stamp.sec == 23
    assert markers.markers[0].action == Marker.DELETEALL
    samples, edges = markers.markers[1:3]
    assert samples.type == Marker.POINTS and len(samples.points) == len(graph.points)
    assert edges.type == Marker.LINE_LIST and len(edges.points) == 2 * graph.edge_count
    assert all(marker.header.frame_id == 'map' for marker in markers.markers)
    positions = tuple((pose.pose.position.x, pose.pose.position.y) for pose in path.poses)
    assert positions == route.points
    assert all(pose.header == path.header and pose.pose.orientation.w == 1.0
               for pose in path.poses)


def test_no_path_or_invalid_map_cannot_leave_old_visuals(example, tmp_path):
    _, space, graph, _, info = example
    _, markers, path = outputs.make_messages(info, space, graph, None, Time())
    assert not path.poses and len(markers.markers) > 1
    occupancy, markers, path = outputs.make_messages(info, None, None, None, Time())
    assert occupancy.info == info and set(occupancy.data) == {100}
    assert len(markers.markers) == 1 and markers.markers[0].action == Marker.DELETEALL
    assert not path.poses
    with pytest.raises(ValueError, match='metadata'):
        outputs.make_messages(info, replace(space, origin_x=3.0), graph, None, Time())
    protected = tmp_path / 'notes.txt'
    protected.write_text('keep this file')
    with pytest.raises(ValueError, match='.png'):
        outputs.remove_figure(str(protected))
    assert protected.read_text() == 'keep this file'


def test_saved_figures_use_world_axes_and_never_invent_a_route(example, tmp_path, monkeypatch):
    from matplotlib.axes import Axes
    from matplotlib.figure import Figure
    import matplotlib.image as image

    raw, space, graph, route, _ = example
    images, plotted_paths = [], []
    imshow, plot, savefig = Axes.imshow, Axes.plot, Figure.savefig

    def record_image(axis, data, *args, **kwargs):
        images.append(kwargs)
        return imshow(axis, data, *args, **kwargs)

    def record_plot(axis, *args, **kwargs):
        if kwargs.get('label') == 'Selected path':
            plotted_paths.append(args)
        return plot(axis, *args, **kwargs)

    def check_legend(figure, *args, **kwargs):
        figure.canvas.draw()
        renderer = figure.canvas.get_renderer()
        for legend in figure.legends:
            bounds = legend.get_window_extent(renderer)
            assert figure.bbox.contains(bounds.x0, bounds.y0)
            assert figure.bbox.contains(bounds.x1, bounds.y1)
        return savefig(figure, *args, **kwargs)

    monkeypatch.setattr(Axes, 'imshow', record_image)
    monkeypatch.setattr(Axes, 'plot', record_plot)
    monkeypatch.setattr(Figure, 'savefig', check_legend)
    filename = outputs.save_figure(
        str(tmp_path / 'path.png'), raw, space, graph, route, 0, 'path_found')
    pixels = image.imread(filename)
    assert pixels.shape[0] > 400 and pixels.shape[1] > 1000
    assert len(plotted_paths) == 1
    assert all(item['origin'] == 'lower' and item['extent'] == (1.0, 3.0, -2.0, 0.0)
               for item in images)
    plotted_paths.clear()
    outputs.save_figure(
        str(tmp_path / 'no_path.png'), raw, space, graph, None, 0, 'no_path_found')
    assert not plotted_paths
    outputs.remove_figure(str(filename))
    assert not filename.exists()
