from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():
    launch_dir = get_package_share_directory('robobus_sensor_kit_launch')

    ouster_fl_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "launch", "ouster_fl_launch.py"))
        )
      
    ouster_fr_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "launch", "ouster_fr_launch.py"))
        )
        
    ouster_rl_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "launch", "ouster_rl_launch.py"))
        )

    ouster_rr_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "launch", "ouster_rr_launch.py"))
        )

    # Combine both launch files
    return LaunchDescription([
      ouster_fl_launch,
      ouster_fr_launch,
      ouster_rl_launch,
      ouster_rr_launch
    ])
