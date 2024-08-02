from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import yaml
from rosidl_runtime_py import *


class TaskPlanItemResult:
    def __init__(self):
        self.item_started: bool = False
        self.navigation_started: bool = False
        self.navigation_result = None


class TaskPlanItemType:
    def __init__(self):
        self.started = False
        self.nav_result = None


class TaskPlanItem:
    def __init__(self, item_id=''):
        self.item_id: str = item_id
        # self.behavior_tree_name: str = ""  # TODO remove
        self.type: str | None = None
        # self.nav_path: list[dict] | None = list()
        self.row_waypoints: list[dict] | None = list()
        self.approach_pose: dict | None = None
        self.nav_timeout: float | None = None
        self.result: TaskPlanItemResult | None = None

    # def add_pose(self, pose_stamped) -> None:
    #     p = yaml.safe_load(message_to_yaml(pose_stamped))
    #     self.nav_path.append(p)
    #
    # def replace_last_pose(self, pose_stamped) -> None:
    #     if len(self.nav_path):
    #         p = yaml.safe_load(message_to_yaml(pose_stamped))
    #         self.nav_path[-1] = p
    #     else:
    #         self.add_pose(pose_stamped)
    #
    # def get_pose_stamped_list(self) -> list[PoseStamped]:
    #     pose_stamped_list = list()
    #     for goal_pose in self.nav_path:
    #         pose_stamped = PoseStamped()
    #         set_message_fields(pose_stamped, goal_pose)
    #         pose_stamped_list.append(pose_stamped)
    #
    #     return pose_stamped_list

    def get_row_waypoints(self) -> list[PoseStamped]:
        if self.row_waypoints is None:
            raise ValueError(f"row_waypoints was None in item {self.item_id}")
        pose_stamped_list = list()
        for goal_pose in self.row_waypoints:
            pose_stamped = PoseStamped()
            set_message_fields(pose_stamped, goal_pose)
            pose_stamped_list.append(pose_stamped)

        return pose_stamped_list

    def get_approach_pose(self) -> PoseStamped | None:
        if self.approach_pose is None:
            raise ValueError(f"approach_pose was None in item {self.item_id}")
        if not isinstance(self.approach_pose, dict):
            raise TypeError(f"approach_pose is not a dict in item {self.item_id}")
        pose_stamped = PoseStamped()
        set_message_fields(pose_stamped, self.approach_pose)
        return pose_stamped


class SprayingTaskPlan:
    def __init__(self):
        self.task_plan_items: list[TaskPlanItem] = list()
        self.map_frame: str = "map"
        self.row_path_controller_id: str = "FollowPath"
        self.row_path_pose_distance: float = 0.1

    @staticmethod
    def load(file_path):  # TODO class method and type hint
        try:
            # read existing poses
            with open(file_path, 'r') as f:
                return yaml.unsafe_load(f)
        except FileNotFoundError:
            # if the file does not exist, create a new task plan
            return SprayingTaskPlan()

    def get_item(self, item_id: str) -> TaskPlanItem | None:
        found_items = [x for x in self.task_plan_items if x.item_id == item_id]
        if len(found_items):
            return found_items[0]
        else:
            return None

    def get_item_ids(self) -> list[str]:
        return list(map(lambda x: x.item_id, self.task_plan_items))
