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
    
    qrcode_pcl_node=Node(
        package = 'qr_reader',
        name = 'qrcode_pcl_node',
        executable = 'qrcode_pcl',
        #parameters = [qr_reader_params_file]
        parameters = [config]
        #parameters = [{"debug": False}]
    )

    qrcode_image_node=Node(
        package = 'qr_reader',
        name = 'qrcode_image_node',
        executable = 'qrcode_image',
        parameters = [qr_reader_params_file]
        #parameters = [config]
        #parameters = [{"debug": False}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_qr_reader_params_file_cmd)
    #ld.add_action(qrcode_pcl_node)
    ld.add_action(qrcode_image_node)


    return ld