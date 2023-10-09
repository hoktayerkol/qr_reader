
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    launch_dir = get_package_share_directory('qr_reader')
    
    print('ffffffff', launch_dir)
    usb_cam_node_cmd = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        remappings=[('/image_raw', 'image')]
    )

    zbar_node_cmd = Node(
        package='zbar_ros',
        executable='barcode_reader'
    )


    rviz2_cmd = Node(
        package='rviz2',
        executable='rviz2',
        namespace='',
        name='rviz2',
        arguments=['-d', os.path.join(launch_dir, 'config', 'test.rviz')]
    )

    ld = LaunchDescription()

    ld.add_action(usb_cam_node_cmd)
    ld.add_action(zbar_node_cmd)
    #ld.add_action(rviz2_cmd)

    return ld
