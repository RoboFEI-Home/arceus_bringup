import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess,\
                           IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    description_share = launch_ros.substitutions.FindPackageShare(package='arceus_description').find('arceus_description')
    default_model_path = os.path.join(description_share, 'src/urdf/arceus_description.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': launch_ros.parameter_descriptions.ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    control_share = launch_ros.substitutions.FindPackageShare(package='arceus_control').find('arceus_control')
    controller_params_file = os.path.join(control_share, 'config/omnidirectional_controller.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])
    
    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omnidirectional_controller"],
    )

    omni_base_controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[omni_base_controller_spawner]
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_state_broadcaster_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(control_share, 'config/ekf.yaml')]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),                         
        robot_state_publisher_node,    
        delayed_controller_manager,                                  
        joint_state_broadcaster_event_handler,
        robot_localization_node,
        omni_base_controller_event_handler
    ])