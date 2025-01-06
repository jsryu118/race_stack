import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare a launch argument for the configuration folder
    config_folder_arg = DeclareLaunchArgument(
        'config_folder',
        default_value=os.path.join(get_package_share_directory('vesc_ackermann'), 'config'),
        description='Path to the folder containing configuration files'
    )

    # Get the configuration folder from the launch argument
    config_folder = LaunchConfiguration('config_folder')

    def launch_setup(context, *args, **kwargs):
        # Resolve the configuration folder path
        config_folder_path = config_folder.perform(context)

        # Resolve individual configuration files
        ackermann_manage_config_path = os.path.join(config_folder_path, 'ackermann_manage.yaml')
        shared_config_path = os.path.join(config_folder_path, 'shared.yaml')
        vesc_odom_config_path = os.path.join(config_folder_path, 'vesc_odom.yaml')
        vesc_config_path = os.path.join(config_folder_path, 'vesc_config.yaml')

        # Load configurations using yaml.safe_load
        with open(vesc_config_path, 'r') as vesc_config_file:
            vesc_config = yaml.safe_load(vesc_config_file)

        with open(shared_config_path, 'r') as shared_config_file:
            shared_config = yaml.safe_load(shared_config_file)

        # Namespace from environment variable
        car_name = os.getenv('F1TENTH_CAR_NAME_MA', '')
        ns = car_name if car_name else 'default_namespace'

        # VESC Driver Node
        vesc_driver_node = Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            namespace=ns,
            parameters=[vesc_config],
            output='screen'
        )

        # Ackermann to VESC Node
        ackermann_to_vesc_node = Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            namespace=ns,
            parameters=[shared_config],
            output='screen'
        )

        # VESC to Odom Node
        vesc_to_odom_node = Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            namespace=ns,
            parameters=[shared_config, vesc_odom_config_path],
            output='screen'
        )

        # Ackermann Manager Node
        ackermann_manager_node = Node(
            package='vesc_ackermann',
            executable='ackermann_manager_node',
            name='ackermann_manager_node',
            namespace=ns,
            parameters=[ackermann_manage_config_path],
            output='screen'
        )

        return [
            vesc_driver_node,
            ackermann_to_vesc_node,
            vesc_to_odom_node,
            ackermann_manager_node
        ]

    return LaunchDescription([
        config_folder_arg,
        OpaqueFunction(function=launch_setup)
    ])
