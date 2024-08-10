import os

from typing_extensions import Self
from enum import Enum
import yaml

from geometry_msgs.msg import PoseStamped
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


class SprayingTaskPlan:
    def __init__(self):
        self.task_plan_items: list[TaskPlanItem] = list()
        self.map_frame: str = "map"
        self.row_path_controller_id: str = "FollowPath"
        self.row_path_goal_checker_id: str = "asb_goal_checker"
        self.row_path_progress_checker_id: str = "simple_progress_checker"
        self.row_path_pose_distance: float = 0.1

    @classmethod
    def from_dict(cls, d) -> Self:
        t = SprayingTaskPlan()
        t.task_plan_items = list(map(TaskPlanItem.from_dict, d['task_plan_items']))
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
            'task_plan_items': list(map(lambda i: i.to_dict(), self.task_plan_items)),
        }

    @classmethod
    def load(cls, file_path: str) -> Self:
        with open(file_path, 'r') as f:
            return cls.from_dict(yaml.unsafe_load(f))

    def write(self, file_path: str) -> None:
        with open(file_path, 'w') as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False, sort_keys=False)

    @classmethod
    def get_template(cls) -> Self:
        t = SprayingTaskPlan()
        i_0 = TaskPlanItem(item_id='approach_1')
        i_0.set_type(TaskPlanItemType.APPROACH)
        i_0.set_approach_pose(PoseStamped(header=Header(frame_id="map")))
        i_0._timeout = 0.0
        t.task_plan_items.append(i_0)
        i_0 = TaskPlanItem(item_id='row_1')
        i_0.set_type(TaskPlanItemType.ROW)
        i_0.set_row_waypoints([PoseStamped(header=Header(frame_id="map")), PoseStamped(header=Header(frame_id="map"))])
        i_0._timeout = 0.0
        t.task_plan_items.append(i_0)
        return t

    def get_preceding_item(self, item: TaskPlanItem) -> TaskPlanItem | None:
        try:
            i = self.task_plan_items.index(item) - 1
            if i >= 0:
                return self.task_plan_items[i]
            else:
                return None
        except ValueError:
            return None

    def get_item_ids(self) -> list[str]:
        return list(map(lambda x: x.get_item_id(), self.task_plan_items))


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
