#!/usr/bin/python3

import os
from collections import defaultdict

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped


class RowPointsLogger(Node):

    def __init__(self):
        super().__init__('row_points_logger')

        self.declare_parameter('row_points_file_path', os.path.expanduser('~/tmp/row_points.yaml'))
        self.row_points_file_path = self.get_parameter('row_points_file_path').get_parameter_value().string_value

        self.declare_parameter('task_plan_points_file_path', os.path.expanduser('~/tmp/task_plan_points.yaml'))
        self.task_plan_points_file_path = self.get_parameter('task_plan_points_file_path').get_parameter_value().string_value

        self.declare_parameter('frame_id', 'field')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.declare_parameter('row_axis', 'x')
        self.row_axis = self.get_parameter('row_axis').get_parameter_value().string_value
        if self.row_axis not in ['x', 'y']:
            self.get_logger().fatal(f"parameter row_axis should be 'x' or 'y'")
            return

        self.declare_parameter('row_naming_ascending', False)  # if row_naming_ascending is True, the row index increases the same way as the x/y coordinate
        self.row_naming_ascending = self.get_parameter('row_naming_ascending').get_parameter_value().bool_value

        self.declare_parameter('approximate_row_distance', 4.0)
        self.row_dist = self.get_parameter('approximate_row_distance').get_parameter_value().double_value

        if not os.path.exists(os.path.dirname(self.task_plan_points_file_path)):
            os.makedirs(os.path.dirname(self.task_plan_points_file_path))

        self.write_task_plan_rows()

    def write_task_plan_rows(self):

        if os.path.exists(self.row_points_file_path) and not os.path.isfile(self.row_points_file_path):
            self.get_logger().error(f"path exists but is not a file [{self.row_points_file_path}], can't save row points")
            return

        clicked_points = list()
        if os.path.exists(self.row_points_file_path):
            with open(self.row_points_file_path, 'r') as f:
                d = yaml.safe_load(f)
                if not isinstance(d, dict) or 'row_points' not in d:
                    self.get_logger().error(f"there are no points in the row_points file [{self.row_points_file_path}]")

                clicked_points = list(map(lambda d_p: Point(x=d_p['x'], y=d_p['y']), d['row_points']))

        def row_index(point) -> int:
            r_coord = point.x if self.row_axis == 'y' else point.y
            return int(np.floor(r_coord/self.row_dist))

        row_indices: list[int] = list(map(row_index, clicked_points))

        points: list[dict[str, int | float | str]] = list()
        for p in clicked_points:
            x: float = round(p.x, 2)
            y: float = round(p.y, 2)
            r_index: int = row_index(p)
            r_name_index: int = r_index - min(row_indices) + 1 if self.row_naming_ascending else max(row_indices) - r_index + 1

            points.append({
                'row_id': f"row_{r_name_index}",
                'row_index': r_name_index,
                'x': x,
                'y': y,
            })

        points = sorted(points, key=lambda r: (r['row_index'], r[self.row_axis]))

        self.get_logger().info(f"row_points:")
        for row_point in points:
            self.get_logger().info(f"n: {row_point['row_id']}  x: {row_point['x']:.2f}  y: {row_point['y']:.2f}")

        # write the points in the task plan format, only picking the ones at the far ends of each row (the points should already be sorted, so pick the first and last one)
        points_by_row_id: defaultdict[str, list[dict]] = defaultdict(list)
        for p in points:
            points_by_row_id[p['row_id']].append(p)

        plan_rows: dict = {'rows': list()}
        for row_id, points in points_by_row_id.items():
            plan_rows['rows'].append({
                'row_id': row_id,
                'frame_id': self.frame_id,
                'start_point': {
                    'x': points[0]['x'],
                    'y': points[0]['y'],
                },
                'end_point': {
                    'x': points[-1]['x'],
                    'y': points[-1]['y'],
                },
            })

        with open(self.task_plan_points_file_path, 'w') as f:
            yaml.dump(plan_rows, f, sort_keys=False)


def main(args=None):
    rclpy.init(args=args)
    node = RowPointsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
