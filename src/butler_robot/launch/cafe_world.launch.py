import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess

def generate_launch_description():

    pkg_butler = get_package_share_directory('butler_robot')
    world_file = os.path.join(pkg_butler, 'worlds', 'cafe.world')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen'
        ),
    ])