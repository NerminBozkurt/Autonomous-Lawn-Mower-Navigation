#!/usr/bin/env python3
"""
Track live mowing coverage and draw the mowed area in RViz.

Listens to /coverage_path (which defines the field unless field_polygon is
set) and to the true robot pose on /ground_truth/pose, and keeps a
CoverageGrid of the area swept so far. It publishes:
- /coverage_percent (std_msgs/Float32): share of the field mowed, in %;
- /coverage_markers (visualization_msgs/MarkerArray): the mowed area and a
  coverage label.

Coverage accumulates for as long as the node runs, i.e. one simulation
launch; ~/clear (std_srvs/Empty) starts it over. A coverage path with a
different shape rebuilds the field and starts over too.

field_polygon takes the field's corners as a flat list [x1, y1, x2, y2, ...]
in the map frame, for fields that are not the rectangle the swaths tile.
"""

from geometry_msgs.msg import Point, PoseStamped
from mower_sim.coverage_grid import CoverageGrid
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float32
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


def _color(r, g, b, a):
    return ColorRGBA(r=r, g=g, b=b, a=a)


class CoverageMonitor(Node):

    def __init__(self):
        super().__init__('coverage_monitor')

        self.declare_parameter('cutting_width', 0.75)
        self.declare_parameter('field_polygon', [0.0])
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('path_topic', '/coverage_path')
        self.declare_parameter('pose_topic', '/ground_truth/pose')
        self.declare_parameter('publish_rate', 2.0)
        # Edge of the squares the mowed area is drawn with in RViz.
        self.declare_parameter('draw_cell', 0.05)

        p = self.get_parameter
        self.cutting_width = p('cutting_width').value
        flat = list(p('field_polygon').value)
        self.polygon = (list(zip(flat[0::2], flat[1::2]))
                        if len(flat) >= 6 else None)
        self.map_frame = p('map_frame').value
        self.draw_cell = p('draw_cell').value

        self.grid = None
        self.path_key = None
        if self.polygon:
            self.grid = CoverageGrid(cutting_width=self.cutting_width,
                                     polygon=self.polygon)

        self.percent_pub = self.create_publisher(Float32, '/coverage_percent', 10)
        self.marker_pub = self.create_publisher(MarkerArray,
                                                '/coverage_markers', 10)
        self.create_subscription(Path, p('path_topic').value,
                                 self.path_callback, 10)
        self.create_subscription(PoseStamped, p('pose_topic').value,
                                 self.pose_callback, 50)
        self.create_service(Empty, '~/clear', self.clear_callback)
        self.create_timer(1.0 / p('publish_rate').value, self.publish)

        source = 'field_polygon' if self.polygon else p('path_topic').value
        self.get_logger().info(f'Coverage monitor: field from {source}')

    def path_callback(self, msg):
        if self.polygon or len(msg.poses) < 2:
            return
        xy = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        key = (len(xy), xy[0], xy[-1])
        if key == self.path_key:
            return  # the same path, republished
        try:
            self.grid = CoverageGrid(xy, cutting_width=self.cutting_width)
        except ValueError as exc:
            self.get_logger().warn(f'Cannot derive a field: {exc}')
            return
        self.path_key = key
        self.get_logger().info('New coverage path, field rebuilt')

    def pose_callback(self, msg):
        if self.grid is not None:
            self.grid.add_pose(msg.pose.position.x, msg.pose.position.y)

    def clear_callback(self, request, response):
        if self.grid is not None:
            self.grid.clear()
        self.get_logger().info('Coverage cleared')
        return response

    def publish(self):
        if self.grid is None:
            return
        percent = self.grid.percent
        self.percent_pub.publish(Float32(data=float(percent)))

        stamp = self.get_clock().now().to_msg()

        def marker(ns, kind):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = ns
            m.id = 0
            m.type = kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        mowed = marker('mowed_area', Marker.CUBE_LIST)
        xs, ys, size = self.grid.swept_cells(self.draw_cell)
        mowed.scale.x = mowed.scale.y = size
        mowed.scale.z = 0.005
        mowed.color = _color(0.1, 0.75, 0.2, 0.55)
        mowed.points = [Point(x=float(x), y=float(y), z=0.0)
                        for x, y in zip(xs, ys)]

        label = marker('coverage_label', Marker.TEXT_VIEW_FACING)
        bx = [x for x, _ in self.grid.boundary]
        by = [y for _, y in self.grid.boundary]
        label.pose.position.x = (min(bx) + max(bx)) / 2.0
        label.pose.position.y = max(by) + 0.6
        label.pose.position.z = 0.5
        label.scale.z = 0.35
        label.color = _color(1.0, 1.0, 1.0, 1.0)
        label.text = f'Coverage {percent:.1f} %'

        self.marker_pub.publish(MarkerArray(markers=[mowed, label]))


def main():
    rclpy.init()
    node = CoverageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
