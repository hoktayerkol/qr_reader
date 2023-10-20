import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('qr_reader'),
        'params',
        'params.yaml'
        )
    
    qr_reader_params_file = LaunchConfiguration('qr_reader_params_file')
    declare_qr_reader_params_file_cmd = DeclareLaunchArgument(
        'qr_reader_params_file',
        default_value=config,
        description='full path to params.yaml of qrcode_reader'
    )
    
        
    print (config)
    node=Node(
        package = 'qr_reader',
        name = 'qr_reader_node',
        executable = 'qr_reader_node',
        #parameters = [qr_reader_params_file]
        parameters = [config]
        #parameters = [{"debug": False}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_qr_reader_params_file_cmd)
    ld.add_action(node)

    return ld