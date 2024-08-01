from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import yaml
from rosidl_runtime_py import *


class TaskPlanItemResult:
    def __init__(self):
        self.started = False
        self.nav_result = None


class TaskPlanItemType:
    def __init__(self):
        self.started = False
        self.nav_result = None


class TaskPlanItem:
    def __init__(self, item_id=''):
        self.item_id: str = item_id
        self.behavior_tree_name: str = ""
        self.type: str | None = None
        self.nav_path: list[dict] | None = list()
        self.nav_pose: PoseStamped | None = None
        self.nav_timeout: float | None = None
        self.result: TaskPlanItemResult | None = None

    def add_pose(self, pose_stamped) -> None:
        p = yaml.safe_load(message_to_yaml(pose_stamped))
        self.nav_path.append(p)

    def replace_last_pose(self, pose_stamped) -> None:
        if len(self.nav_path):
            p = yaml.safe_load(message_to_yaml(pose_stamped))
            self.nav_path[-1] = p
        else:
            self.add_pose(pose_stamped)

    def get_pose_stamped_list(self) -> list[PoseStamped]:
        pose_stamped_list = list()
        for goal_pose in self.nav_path:
            pose_stamped = PoseStamped()
            set_message_fields(pose_stamped, goal_pose)
            pose_stamped_list.append(pose_stamped)

        return pose_stamped_list

    def get_path(self) -> Path:
        return Path(poses=self.get_pose_stamped_list())

    def get_pose(self) -> PoseStamped:
        return self.nav_pose


class SprayingTaskPlan:
    def __init__(self):
        self.task_plan_items: list[TaskPlanItem] = list()

    @staticmethod
    def load(file_path):
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
