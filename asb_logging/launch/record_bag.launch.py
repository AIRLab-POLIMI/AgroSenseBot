
import os

import launch
import launch.actions
import launch.events


def generate_launch_description():

    record_node = launch.actions.ExecuteProcess(
        cmd="ros2 bag record -a --compression-mode file --compression-format zstd --max-bag-duration 60".split(),
        cwd=os.path.expanduser("~/asb_logs/"),
        output='screen'
    )

    ld = launch.LaunchDescription()

    ld.add_action(record_node)

    return ld
