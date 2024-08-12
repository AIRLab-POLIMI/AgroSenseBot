import os

from typing_extensions import Self
from enum import Enum
import yaml

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from rosidl_runtime_py import set_message_fields, message_to_yaml
from std_msgs.msg import Header


def dict_to_pose_stamped(d: dict) -> PoseStamped:
    pose_stamped = PoseStamped()
    set_message_fields(pose_stamped, d)
    return pose_stamped


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
        self._timeout: float | None = None
        self._result: TaskPlanItemResult | None = None

    @classmethod
    def from_dict(cls, d) -> Self:
        item = TaskPlanItem(item_id=d['item_id'])
        item.set_type_from_str(d['type'])
        item.set_timeout(d['timeout'])
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
            'timeout': self._timeout,
        }
        if self._type == TaskPlanItemType.APPROACH:
            d['approach_pose'] = yaml.safe_load(message_to_yaml(self._approach_pose))
        elif self._type == TaskPlanItemType.ROW:
            d['row_waypoints'] = list(map(lambda w: yaml.safe_load(message_to_yaml(w)), self._row_waypoints))
            if self._left_row_id is not None:
                d['left_row_id'] = self._left_row_id
            if self._right_row_id is not None:
                d['right_row_id'] = self._right_row_id

        if self._result is not None:
            d['result'] = self._result

        return d

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

    def get_timeout(self) -> float:
        return self._timeout

    def set_timeout(self, timeout: float) -> None:
        if not isinstance(timeout, float):
            raise TypeError(f"trying to set timeout of type {type(timeout)} in item [{self._item_id}], timeout type should be {type(float)}")
        self._timeout = timeout

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

    def get_start_point(self):
        return self._start_point

    def get_end_point(self):
        return self._end_point


class SprayingTaskPlan:
    def __init__(self):
        self.items: list[TaskPlanItem] = list()
        self.rows: list[TaskPlanRow] = list()
        self.map_frame: str = "map"
        self.row_path_controller_id: str = "FollowPath"
        self.row_path_goal_checker_id: str = "asb_goal_checker"
        self.row_path_progress_checker_id: str = "simple_progress_checker"
        self.row_path_pose_distance: float = 0.1

    @classmethod
    def from_dict(cls, d) -> Self:
        t = SprayingTaskPlan()
        t.items = list(map(TaskPlanItem.from_dict, d['items']))
        t.rows = list(map(TaskPlanRow.from_dict, d['rows']))
        t.map_frame = d['map_frame']
        t.row_path_controller_id = d['row_path_controller_id']
        t.row_path_goal_checker_id = d['row_path_goal_checker_id']
        t.row_path_progress_checker_id = d['row_path_progress_checker_id']
        t.row_path_pose_distance = d['row_path_pose_distance']
        return t

    def to_dict(self) -> dict:
        return {
            'map_frame': self.map_frame,
            'row_path_controller_id': self.row_path_controller_id,
            'row_path_goal_checker_id': self.row_path_goal_checker_id,
            'row_path_progress_checker_id': self.row_path_progress_checker_id,
            'row_path_pose_distance': self.row_path_pose_distance,
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
        i_0.set_timeout(10.0)
        t.items.append(i_0)

        i_1 = TaskPlanItem(item_id='row_1')
        i_1.set_type(TaskPlanItemType.ROW)
        i_1.set_row_waypoints([PoseStamped(header=Header(frame_id='map')), PoseStamped(header=Header(frame_id='map'))])
        i_1.set_left_row_id('row_1')
        i_1.set_right_row_id('row_2')
        i_1.set_timeout(10.0)
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
            raise TypeError(f"trying to get a row with row_id [{row_id}] type different than string")
        found_rows = [x for x in self.rows if x.get_row_id() == row_id]
        if len(found_rows) == 0:
            raise IndexError(f"row {row_id} not in rows list")
        elif len(found_rows) > 1:
            raise IndexError(f"multiple rows with row_id [{row_id}] in rows list")
        else:
            return found_rows[0]


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
