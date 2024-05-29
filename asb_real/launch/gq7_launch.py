import os

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python import get_package_share_directory as pkg


def generate_launch_description():
    ntrip_caster_password_file_path = os.path.expanduser("~/NTRIP_caster_password")
    with open(ntrip_caster_password_file_path, mode='r') as ntrip_caster_password_file:
        ntrip_caster_password = ntrip_caster_password_file.read()

    return LaunchDescription([

        # Microstrain driver node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg('asb_real'), 'launch', 'microstrain_launch.py')),
            launch_arguments={
                'configure': 'true',
                'activate': 'true',
                'params_file': os.path.join(pkg('asb_real'), 'config', 'gq7.yml'),
                'namespace': '/',
            }.items()
        ),

        # NTRIP client node  TODO fix same name as microstrain_driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg('asb_real'), 'launch', 'ntrip_client_launch.py')),
            launch_arguments={
                'host': '158.102.7.10',  # SPIN3 GNSS
                'mountpoint': 'RTK_NRT_RTCM3',  # nearest base station
                'username': 'agrosensebot',
                'password': ntrip_caster_password,
            }.items()
        ),

    ])
