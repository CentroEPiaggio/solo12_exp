import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
        
    terrain = LaunchConfiguration('terrain', default='rigid')
    
    bag_filename = LaunchConfiguration('bag_filename', default='010_move_base')
    
    # ======================================================================== #
    
    launch_arguments = {
        'bag_filename': bag_filename,
        'robot_name': 'solo12',
        'package_name': 'solo_description',
        'robot_file_path': os.path.join('xacro', 'solo12.urdf.xacro'),
        'world_file_path': os.path.join('worlds', 'solo.world'),
        'height': '0.35',
        'terrain': terrain,
    }.items()


    return LaunchDescription([
        
        DeclareLaunchArgument('terrain', default_value='rigid'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'robot.launch.py')
            ),
            launch_arguments = launch_arguments,
        ),
    ])
