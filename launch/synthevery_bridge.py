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

    ld.add_action(Node(
        package='synthevery_dev_tools',
        executable='synthevery_serial_bridge',
        name='synthevery_serial_bridge',
        parameters=[{'config_file': config_file}]
    ))
    
    for device in device_list.values():
        ld.add_action(Node(
            package='synthevery_dev_tools',
            executable='adaptive_kp_mahony_orientation_filter',
            name=f'akp_mahony_{device["device_name"]}',
            remappings=[
                ('~/accel', f'/synthevery_serial_bridge/{device["device_name"]}/accel'),
                ('~/gyro', f'/synthevery_serial_bridge/{device["device_name"]}/gyro'),
            ],
            parameters=[
                {'frame_id': device["frame_id"]},
                {'world_frame_id': device["world_frame"]["frame_id"]},
            ]
        ))
        ld.add_action(Node(
            package='synthevery_dev_tools',
            executable='global_accel_publisher',
            name=f'global_accel_publisher_{device["device_name"]}',
            remappings=[
                ('~/accel', f'/synthevery_serial_bridge/{device["device_name"]}/accel'),
                ('~/orientation', f'/akp_mahony_{device["device_name"]}/orientation'),
            ],
            parameters=[
                {'frame_id': device["world_frame"]["frame_id"]},
            ]
        ))
        ld.add_action(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_transform_publisher_{device["device_name"]}',
            arguments=[
                '--x', str(device["world_frame"]["position"]["x"]),
                '--y', str(device["world_frame"]["position"]["y"]),
                '--z', str(device["world_frame"]["position"]["z"]),
                '--qx', str(device["world_frame"]["orientation"]["x"]),
                '--qy', str(device["world_frame"]["orientation"]["y"]),
                '--qz', str(device["world_frame"]["orientation"]["z"]),
                '--qw', str(device["world_frame"]["orientation"]["w"]),
                '--frame-id', device["world_frame"]["parent_frame_id"],
                '--child-frame-id', device["world_frame"]["frame_id"],
            ]
        ))
        ld.add_action(Node(
            package='synthevery_dev_tools',
            executable='mesh_publisher',
            name=f'mesh_publisher_{device["device_name"]}',
            parameters=[
                {'frame_id': device["frame_id"]},
            ]
        ))

    return ld

