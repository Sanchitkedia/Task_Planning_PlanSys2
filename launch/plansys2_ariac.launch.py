import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_ariac')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/durative_kitting_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    pickup_part_cmd = Node(
        package='plansys2_ariac',
        executable='pickup_part_action_node',
        name='pickup_part',
        namespace=namespace,
        output='screen',
        parameters=[])

    pickup_tray_cmd = Node(
        package='plansys2_ariac',
        executable='pickup_tray_action_node',
        name='pickup_tray',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    putdown_part_cmd = Node(
        package='plansys2_ariac',
        executable='putdown_part_action_node',
        name='putdown_part',
        namespace=namespace,
        output='screen',
        parameters=[])

    putdown_tray_cmd = Node(
        package='plansys2_ariac',
        executable='putdown_tray_action_node',
        name='putdown_tray',
        namespace=namespace,
        output='screen',
        parameters=[])
    
    change_to_part_gripper_cmd = Node(
        package='plansys2_ariac',
        executable='change_to_part_gripper_action_node',
        name='change_to_part_gripper',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    
    change_to_tray_gripper_cmd = Node(
        package='plansys2_ariac',
        executable='change_to_tray_gripper_action_node',
        name='change_to_tray_gripper',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    
    start_competition_cmd = Node(
        package='plansys2_ariac',
        executable='start_competition_action_node',
        name='start_competition',
        namespace=namespace,
        output='screen',
        parameters=[])   # Create the launch description and populate
    
    launch_description = LaunchDescription()

    launch_description.add_action(declare_namespace_cmd)

    # Declare the launch options
    launch_description.add_action(plansys2_cmd)
    launch_description.add_action(pickup_part_cmd)
    launch_description.add_action(pickup_tray_cmd)
    launch_description.add_action(putdown_part_cmd)
    launch_description.add_action(putdown_tray_cmd)
    launch_description.add_action(change_to_part_gripper_cmd)
    launch_description.add_action(change_to_tray_gripper_cmd)
    launch_description.add_action(start_competition_cmd)

    return launch_description
