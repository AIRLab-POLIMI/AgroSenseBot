import os


from typing_extensions import Self
from enum import Enum
import yaml
import numpy as np

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PointStamped, Point, Pose, Quaternion
from rosidl_runtime_py import message_to_yaml
from std_msgs.msg import Header


def dict_to_pose_stamped(d: dict) -> PoseStamped:
    p = PoseStamped()

    # rotation can be specified with a quaternion, euler angles, or not specified.
    # quaterion takes the precedence if both are specified.
    if 'qx' in d or 'qy' in d or 'qz' in d or 'qw' in d:
        q = list()
        q.append(d['qx'] if 'qx' in d else 0.0)
        q.append(d['qy'] if 'qy' in d else 0.0)
        q.append(d['qz'] if 'qz' in d else 0.0)
        q.append(d['qw'] if 'qw' in d else 0.0)
    elif 'roll' in d or 'pitch' in d or 'yaw' in d:
        q = quaternion_from_euler(d['roll'] if 'roll' in d else 0.0, d['pitch'] if 'pitch' in d else 0.0, d['yaw'] if 'yaw' in d else 0.0)
    else:
        q = [0.0, 0.0, 0.0, 1.0]  # no rotation

    p.header.frame_id = d['frame_id']
    p.pose.position.x = float(d['x'])
    p.pose.position.y = float(d['y'])
    p.pose.position.z = float(d['z']) if 'z' in d else 0.0
    p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
    return p


def pose_stamped_to_dict(p: PoseStamped) -> dict:
    d = dict()
    q = p.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    d['frame_id'] = p.header.frame_id
    d['x'] = p.pose.position.x
    d['y'] = p.pose.position.y
    d['yaw'] = yaw
    return d


def yaw_to_quaternion(theta: float) -> Quaternion:
    q = quaternion_from_euler(0.0, 0.0, theta)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class TaskPlanItemResult:
    def __init__(self):
        self.item_started: bool = False
        self.navigation_started: bool = False
        self.navigation_result = None


class TaskPlanItemType(Enum):
    APPROACH = 0
    ROW = 1


class TaskPlanItem:
    def __init__(self, item_id):
        self._item_id: str = item_id
        self._type: TaskPlanItemType | None = None
        self._approach_pose: PoseStamped | None = None
        self._row_waypoints: list[PoseStamped] | None = list()
        self._left_row_id: str | None = None
        self._right_row_id: str | None = None
        self._result: TaskPlanItemResult | None = None

    @classmethod
    def from_dict(cls, d) -> Self:
        item = TaskPlanItem(item_id=d['item_id'])
        item.set_type_from_str(d['type'])
        if 'result' in d:
            item.set_result(d['result'])
        if item.get_type() == TaskPlanItemType.APPROACH:
            item.set_approach_pose(dict_to_pose_stamped(d['approach_pose']))
        elif item.get_type() == TaskPlanItemType.ROW:
            item.set_row_waypoints(list(map(dict_to_pose_stamped, d['row_waypoints'])))
            if 'left_row_id' in d:
                item.set_left_row_id(d['left_row_id'])
            if 'right_row_id' in d:
                item.set_right_row_id(d['right_row_id'])
        else:
            raise ValueError(f"unknown item type [{item.get_type()}]")
        return item

    def to_dict(self):
        d = {
            'item_id': self._item_id,
            'type': self._type.name,
        }
        if self._type == TaskPlanItemType.APPROACH:
            d['approach_pose'] = pose_stamped_to_dict(self._approach_pose)
        elif self._type == TaskPlanItemType.ROW:
            d['row_waypoints'] = list(map(lambda w: pose_stamped_to_dict(w), self._row_waypoints))
            if self._left_row_id is not None:
                d['left_row_id'] = self._left_row_id
            if self._right_row_id is not None:
                d['right_row_id'] = self._right_row_id

        if self._result is not None:
            d['result'] = self._result

        return d

    def set_item_id(self, item_id: str) -> None:
        self._item_id = item_id

    def get_item_id(self) -> str:
        return self._item_id

    def get_type(self) -> TaskPlanItemType | None:
        return self._type

    def set_type(self, type_: TaskPlanItemType) -> None:
        self._type = type_

    def set_type_from_str(self, type_: str) -> None:
        if type_ == "APPROACH":
            self._type = TaskPlanItemType.APPROACH
        elif type_ == "ROW":
            self._type = TaskPlanItemType.ROW
        else:
            raise ValueError(f"trying to set unknown type [{type_}] from string in item {self._item_id}")

    def get_approach_pose(self) -> PoseStamped | None:
        if self._type != TaskPlanItemType.APPROACH:
            raise ValueError(f"trying to get approach pose from an item [{self._item_id}] with type different than APPROACH")
        return self._approach_pose

    def set_approach_pose(self, pose_stamped: PoseStamped) -> None:
        if self._type != TaskPlanItemType.APPROACH:
            raise ValueError(f"trying to set approach pose to an item [{self._item_id}] with type different than APPROACH")
        if not isinstance(pose_stamped, PoseStamped):
            raise TypeError(f"trying to set approach pose of type different than PoseStamped in item {self._item_id}")
        self._approach_pose = pose_stamped

    def get_row_waypoints(self) -> list[PoseStamped]:
        if self._type != TaskPlanItemType.ROW:
            raise ValueError(f"trying to get row waypoints from an item [{self._item_id}] with type different than ROW")
        return self._row_waypoints

    def set_row_waypoints(self, row_waypoints: list[PoseStamped]) -> None:
        if self._type != TaskPlanItemType.ROW:
            raise ValueError(f"trying to set row waypoints to an item [{self._item_id}] with type different than ROW")
        if not isinstance(row_waypoints, list):
            raise TypeError(f"trying to set row waypoint list of type different than list in item {self._item_id}")
        for waypoint in row_waypoints:
            if not isinstance(waypoint, PoseStamped):
                raise TypeError(f"trying to set a row waypoint of type different than PoseStamped in item {self._item_id}")
        self._row_waypoints = row_waypoints

    def set_left_row_id(self, row_id):
        if not isinstance(row_id, str):
            raise TypeError(f"trying to set left row id of type different than str in item {self._item_id}")
        self._left_row_id = row_id

    def get_left_row_id(self):
        if self._type != TaskPlanItemType.ROW:
            raise ValueError(f"trying to get left row id from an item [{self._item_id}] with type different than ROW")
        return self._left_row_id

    def set_right_row_id(self, row_id):
        if not isinstance(row_id, str):
            raise TypeError(f"trying to set right row id of type different than str in item {self._item_id}")
        self._right_row_id = row_id

    def get_right_row_id(self):
        if self._type != TaskPlanItemType.ROW:
            raise ValueError(f"trying to get right row id from an item [{self._item_id}] with type different than ROW")
        return self._right_row_id

    def get_result(self) -> TaskPlanItemResult | None:
        return self._result

    def set_result(self, result: TaskPlanItemResult):
        if not isinstance(result, TaskPlanItemResult):
            raise TypeError(f"trying to set result of type {type(result)} in item [{self._item_id}], result type should be {type(TaskPlanItemResult)}")
        self._result = result


class TaskPlanRow:
    def __init__(self, row_id: str, start_point: PointStamped, end_point: PointStamped):
        self._row_id: str = row_id
        self._start_point: PointStamped = start_point
        self._end_point: PointStamped = end_point

    @classmethod
    def from_dict(cls, d) -> Self:
        start_point: PointStamped = PointStamped(header=Header(frame_id=d['frame_id']), point=Point(x=d['start_point']['x'], y=d['start_point']['y']))
        end_point: PointStamped = PointStamped(header=Header(frame_id=d['frame_id']), point=Point(x=d['end_point']['x'], y=d['end_point']['y']))
        row = TaskPlanRow(d['row_id'], start_point, end_point)
        return row

    def to_dict(self):
        d = {
            'row_id': self._row_id,
            'frame_id': self._start_point.header.frame_id,
            'start_point': {
                'x': self._start_point.point.x,
                'y': self._start_point.point.y,
            },
            'end_point': {
                'x': self._start_point.point.x,
                'y': self._start_point.point.y,
            },
        }
        return d

    def get_row_id(self):
        return self._row_id

    def set_start_end_points(self, start_point: PointStamped,  end_point: PointStamped):
        if not isinstance(start_point, PointStamped):
            raise TypeError("trying to set a start point of type different than PointStamped")
        if not isinstance(end_point, PointStamped):
            raise TypeError("trying to set an end point of type different than PointStamped")
        if start_point.header.frame_id != end_point.header.frame_id:
            raise ValueError("trying to set a start point and an end point with different frame_id")

        self._start_point = start_point
        self._end_point = end_point

    def get_start_point(self) -> PointStamped:
        return self._start_point

    def get_end_point(self) -> PointStamped:
        return self._end_point

    @property
    def s(self) -> Point:
        """start point."""
        return self.get_start_point().point

    @property
    def e(self) -> Point:
        """end point."""
        return self.get_end_point().point


class SprayingTaskPlan:
    def __init__(self):
        self.items: list[TaskPlanItem] = list()
        self.rows: list[TaskPlanRow] = list()
        self.map_frame: str = "map"
        self.row_path_controller_id: str = "FollowPath"
        self.row_path_goal_checker_id: str = "asb_goal_checker"
        self.row_path_progress_checker_id: str = "simple_progress_checker"
        self.row_path_pose_distance: float = 0.1

        # parameters for auto items generation
        self.alternate_rows: bool = False
        self.switch_direction: bool = False
        self.row_path_external_dist: float = 2.5/2
        self.row_path_margin: float = 1.0
        self.row_approach_margin: float = 1.0

    @classmethod
    def from_dict(cls, d) -> Self:
        t = SprayingTaskPlan()
        t.rows = list(map(TaskPlanRow.from_dict, d['rows']))
        t.map_frame = d['map_frame']
        t.row_path_controller_id = d['row_path_controller_id']
        t.row_path_goal_checker_id = d['row_path_goal_checker_id']
        t.row_path_progress_checker_id = d['row_path_progress_checker_id']
        t.row_path_pose_distance = d['row_path_pose_distance']

        t.alternate_rows = bool(d['alternate_rows'])
        t.switch_direction = bool(d['switch_direction'])
        t.row_path_external_dist = d['row_path_external_dist']
        t.row_path_margin = d['row_path_margin']
        t.row_approach_margin = d['row_approach_margin']

        if 'items' in d:
            t.items = list(map(TaskPlanItem.from_dict, d['items']))
        return t

    def to_dict(self) -> dict:
        return {
            'map_frame': self.map_frame,
            'row_path_controller_id': self.row_path_controller_id,
            'row_path_goal_checker_id': self.row_path_goal_checker_id,
            'row_path_progress_checker_id': self.row_path_progress_checker_id,
            'row_path_pose_distance': self.row_path_pose_distance,
            'alternate_rows': self.alternate_rows,
            'switch_direction': self.switch_direction,
            'row_path_external_dist': self.row_path_external_dist,
            'row_path_margin': self.row_path_margin,
            'row_approach_margin': self.row_approach_margin,
            'items': list(map(lambda i: i.to_dict(), self.items)),
            'rows': list(map(lambda i: i.to_dict(), self.rows)),
        }

    @classmethod
    def load(cls, file_path: str) -> Self:
        with open(file_path, 'r') as f:
            return cls.from_dict(yaml.safe_load(f))

    def write(self, file_path: str) -> None:
        with open(file_path, 'w') as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False, sort_keys=False)

    @classmethod
    def get_template(cls) -> Self:
        t = SprayingTaskPlan()
        i_0 = TaskPlanItem(item_id='approach_1')
        i_0.set_type(TaskPlanItemType.APPROACH)
        i_0.set_approach_pose(PoseStamped(header=Header(frame_id='map')))
        t.items.append(i_0)

        i_1 = TaskPlanItem(item_id='row_1')
        i_1.set_type(TaskPlanItemType.ROW)
        i_1.set_row_waypoints([PoseStamped(header=Header(frame_id='map')), PoseStamped(header=Header(frame_id='map'))])
        i_1.set_left_row_id('row_1')
        i_1.set_right_row_id('row_2')
        t.items.append(i_1)

        r_1 = TaskPlanRow(row_id='row_1', start_point=PointStamped(header=Header(frame_id='map')), end_point=PointStamped(header=Header(frame_id='map')))
        t.rows.append(r_1)

        r_2 = TaskPlanRow(row_id='row_2', start_point=PointStamped(header=Header(frame_id='map')), end_point=PointStamped(header=Header(frame_id='map')))
        t.rows.append(r_2)

        return t

    def get_preceding_item(self, item: TaskPlanItem) -> TaskPlanItem | None:
        try:
            i = self.items.index(item) - 1
            if i >= 0:
                return self.items[i]
            else:
                return None
        except ValueError:
            return None

    def get_item_ids(self) -> list[str]:
        return list(map(lambda x: x.get_item_id(), self.items))

    def get_row(self, row_id: str):
        if not isinstance(row_id, str):
            raise TypeError(f"get_row: trying to get a row by row_id, but the type of the row_id requested is different than string [{row_id}]")
        found_rows = [x for x in self.rows if x.get_row_id() == row_id]
        if len(found_rows) == 0:
            raise IndexError(f"row {row_id} not in rows list")
        elif len(found_rows) > 1:
            raise IndexError(f"multiple rows with row_id [{row_id}] in rows list")
        else:
            return found_rows[0]

    def generate_items(self) -> None:
        self.items = list()
        rows = [None] + self.rows + [None]
        row_pairs: list[(TaskPlanRow | None, TaskPlanRow | None)] = list(zip(rows[0:-1], rows[1:]))

        if self.alternate_rows:
            row_pairs = row_pairs[::2] + row_pairs[1::2][::-1]
        for r_1, r_2 in row_pairs:
            print(f"{str(r_1):<50} | {r_2}")

        r_1: TaskPlanRow | None
        r_2: TaskPlanRow | None
        for i, (r_1, r_2) in enumerate(row_pairs):
            inter_row = TaskPlanItem(item_id=None)
            inter_row.set_type(TaskPlanItemType.ROW)
            approach = TaskPlanItem(item_id=None)
            approach.set_type(TaskPlanItemType.APPROACH)

            theta, r_1_t, r_2_t = transform_rows(r_1, r_2)
            path_start: Point | None = None
            path_end: Point | None = None

            discordant_direction = (i + self.switch_direction) % 2
            if discordant_direction:  # path goes from row's end to start

                if r_1 is not None and r_2 is not None:
                    path_start = un_transform(theta, Point(x=max(r_1_t.e.x, r_2_t.e.x) + self.row_path_margin, y=(r_1_t.e.y + r_2_t.e.y)/2))
                    path_end = un_transform(theta, Point(x=min(r_1_t.s.x, r_2_t.s.x) - self.row_path_margin, y=(r_1_t.s.y + r_2_t.s.y)/2))
                    inter_row.set_item_id(f"inter_{r_1.get_row_id()}_{r_2.get_row_id()}")
                    inter_row.set_left_row_id(r_2.get_row_id())
                    inter_row.set_right_row_id(r_1.get_row_id())

                elif r_1 is None:  # first row pair (discordant direction)
                    theta, _, r_2_t = transform_rows(r_1, r_2)
                    path_start = un_transform(theta, Point(x=r_2_t.e.x + self.row_path_margin, y=r_2_t.e.y + self.row_path_external_dist))
                    path_end = un_transform(theta, Point(x=r_2_t.s.x - self.row_path_margin, y=r_2_t.s.y + self.row_path_external_dist))
                    inter_row.set_item_id(f"inter_{r_2.get_row_id()}")
                    inter_row.set_left_row_id(r_2.get_row_id())

                elif r_2 is None:  # last row pair (discordant direction)
                    theta, r_1_t, _ = transform_rows(r_1, r_2)
                    path_start = un_transform(theta, Point(x=r_1_t.e.x + self.row_path_margin, y=r_1_t.e.y - self.row_path_external_dist))
                    path_end = un_transform(theta, Point(x=r_1_t.s.x - self.row_path_margin, y=r_1_t.s.y - self.row_path_external_dist))
                    inter_row.set_item_id(f"inter_{r_1.get_row_id()}")
                    inter_row.set_right_row_id(r_1.get_row_id())

                inter_row.set_row_waypoints([
                    PoseStamped(header=Header(frame_id=self.map_frame), pose=Pose(position=path_start, orientation=yaw_to_quaternion(theta + np.pi))),
                    PoseStamped(header=Header(frame_id=self.map_frame), pose=Pose(position=path_end, orientation=yaw_to_quaternion(theta + np.pi))),
                ])

                theta_approach, p_s_t, _ = transform_points(inter_row.get_row_waypoints()[0].pose.position, inter_row.get_row_waypoints()[1].pose.position)
                approach.set_approach_pose(PoseStamped(
                    header=Header(frame_id=self.map_frame),
                    pose=Pose(position=un_transform(theta_approach, Point(x=p_s_t.x - self.row_approach_margin, y=p_s_t.y)), orientation=yaw_to_quaternion(theta_approach)),
                ))

            else:  # path goes from row's start to end

                if r_1 is not None and r_2 is not None:
                    path_start = un_transform(theta, Point(x=min(r_1_t.s.x, r_2_t.s.x) - self.row_path_margin, y=(r_1_t.s.y + r_2_t.s.y)/2))
                    path_end = un_transform(theta, Point(x=max(r_1_t.e.x, r_2_t.e.x) + self.row_path_margin, y=(r_1_t.e.y + r_2_t.e.y)/2))
                    inter_row.set_item_id(f"inter_{r_1.get_row_id()}_{r_2.get_row_id()}")
                    inter_row.set_left_row_id(r_1.get_row_id())
                    inter_row.set_right_row_id(r_2.get_row_id())

                elif r_1 is None:  # first row pair (concordant direction)
                    path_start = un_transform(theta, Point(x=r_2_t.s.x - self.row_path_margin, y=r_2_t.s.y + self.row_path_external_dist))
                    path_end = un_transform(theta, Point(x=r_2_t.e.x + self.row_path_margin, y=r_2_t.e.y + self.row_path_external_dist))
                    inter_row.set_item_id(f"inter_{r_2.get_row_id()}")
                    inter_row.set_right_row_id(r_2.get_row_id())

                elif r_2 is None:  # last row pair (concordant direction)
                    path_start = un_transform(theta, Point(x=r_1_t.s.x - self.row_path_margin, y=r_1_t.s.y - self.row_path_external_dist))
                    path_end = un_transform(theta, Point(x=r_1_t.e.x + self.row_path_margin, y=r_1_t.e.y - self.row_path_external_dist))
                    inter_row.set_item_id(f"inter_{r_1.get_row_id()}")
                    inter_row.set_left_row_id(r_1.get_row_id())

                inter_row.set_row_waypoints([
                    PoseStamped(header=Header(frame_id=self.map_frame), pose=Pose(position=path_start, orientation=yaw_to_quaternion(theta))),
                    PoseStamped(header=Header(frame_id=self.map_frame), pose=Pose(position=path_end, orientation=yaw_to_quaternion(theta))),
                ])

                theta_approach, p_s_t, _ = transform_points(inter_row.get_row_waypoints()[0].pose.position, inter_row.get_row_waypoints()[1].pose.position)
                approach.set_approach_pose(PoseStamped(
                    header=Header(frame_id=self.map_frame),
                    pose=Pose(position=un_transform(theta_approach, Point(x=p_s_t.x - self.row_approach_margin, y=p_s_t.y)), orientation=yaw_to_quaternion(theta_approach)),
                ))

            approach.set_item_id(f"approach_{inter_row.get_item_id()}")

            self.items.append(approach)
            self.items.append(inter_row)


def transform_rows(r_1: TaskPlanRow | None, r_2: TaskPlanRow | None) -> (float, TaskPlanRow | None, TaskPlanRow | None):
    if r_1 is not None:
        theta = np.arctan2(
            r_1.e.y - r_1.s.y,
            r_1.e.x - r_1.s.x
        )
    else:
        theta = np.arctan2(
            r_2.e.y - r_2.s.y,
            r_2.e.x - r_2.s.x
        )

    return (
        theta,
        TaskPlanRow(
            row_id=r_1.get_row_id(),
            start_point=PointStamped(
                header=r_1.get_start_point().header,
                point=Point(
                    x=r_1.s.x * np.cos(-theta) - r_1.s.y * np.sin(-theta),
                    y=r_1.s.x * np.sin(-theta) + r_1.s.y * np.cos(-theta),
                )
            ),
            end_point=PointStamped(
                header=r_1.get_end_point().header,
                point=Point(
                    x=r_1.e.x * np.cos(-theta) - r_1.e.y * np.sin(-theta),
                    y=r_1.e.x * np.sin(-theta) + r_1.e.y * np.cos(-theta),
                )
            ),
        ) if r_1 is not None else None,
        TaskPlanRow(
            row_id=r_2.get_row_id(),
            start_point=PointStamped(
                header=r_2.get_start_point().header,
                point=Point(
                    x=r_2.s.x * np.cos(-theta) - r_2.s.y * np.sin(-theta),
                    y=r_2.s.x * np.sin(-theta) + r_2.s.y * np.cos(-theta),
                )
            ),
            end_point=PointStamped(
                header=r_2.get_end_point().header,
                point=Point(
                    x=r_2.e.x * np.cos(-theta) - r_2.e.y * np.sin(-theta),
                    y=r_2.e.x * np.sin(-theta) + r_2.e.y * np.cos(-theta),
                )
            ),
        ) if r_2 is not None else None,
    )


def transform_points(p_1: Point, p_2: Point) -> (float, Point, Point):
    theta = np.arctan2(
        p_2.y - p_1.y,
        p_2.x - p_1.x
    )

    return (
        theta,
        Point(
            x=p_1.x * np.cos(-theta) - p_1.y * np.sin(-theta),
            y=p_1.x * np.sin(-theta) + p_1.y * np.cos(-theta),
        ),
        Point(
            x=p_2.x * np.cos(-theta) - p_2.y * np.sin(-theta),
            y=p_2.x * np.sin(-theta) + p_2.y * np.cos(-theta),
        ),
    )


def un_transform(theta: float, p: Point) -> Point:
    return Point(
        x=p.x * np.cos(theta) - p.y * np.sin(theta),
        y=p.x * np.sin(theta) + p.y * np.cos(theta),
    )


if __name__ == '__main__':
    test_file_path = "/tmp/spraying_task_plan_template.yaml"
    if os.path.exists(test_file_path):
        os.remove(test_file_path)

    t1 = SprayingTaskPlan.get_template()
    print("t1:")
    print(t1.to_dict())
    t1.write(test_file_path)

    t2 = SprayingTaskPlan.load(test_file_path)
    print("t2:")
    print(t2.to_dict())
    assert t1.to_dict() == t2.to_dict()
    print("t1 == t2 -> OK")

    t3 = SprayingTaskPlan()
    print("t3:")
    print(t3.to_dict())
    assert t1.to_dict() != t3.to_dict()
    print("t1 != t3 -> OK")
