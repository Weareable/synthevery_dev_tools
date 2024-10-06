from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import json

def generate_launch_description():
    config_file = os.path.join(get_package_share_directory('synthevery_dev_tools'), 'config', 'device_list.json')

    device_list = {}
    with open(config_file, 'r') as f:
        device_list = json.load(f)

    ld = LaunchDescription()

    for device in device_list.values():
        ld.add_action(Node(
            package='synthevery_dev_tools',
            executable='state_machine',
            name=f'state_machine_{device["device_name"]}',
            remappings=[
                ('~/accel', f'/synthevery_serial_bridge/{device["device_name"]}/accel'),
                ('~/gyro', f'/synthevery_serial_bridge/{device["device_name"]}/gyro'),
                ('~/orientation_rpy', f'/akp_mahony_{device["device_name"]}/orientation_rpy'),
                ('~/global_accel', f'/global_accel_publisher_{device["device_name"]}/global_accel'),
            ]
        ))

    return ld

