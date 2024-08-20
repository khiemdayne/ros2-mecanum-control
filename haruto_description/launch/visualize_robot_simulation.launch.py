import os

from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessStart, OnProcessExit

def launch_setup(context, *args, **kwargs):

  # General arguments
  robot_type = LaunchConfiguration("robot_type")

  # rviz config file path
  rviz_config_file = PathJoinSubstitution(
      [FindPackageShare("haruto_description"), "rviz",
        "visualize_robot.rviz"]
  )

  # xacro command for model generation
  robot_description_content = Command(
    [
      PathJoinSubstitution([FindExecutable(name="xacro")]),
      " ",
      PathJoinSubstitution(
        [FindPackageShare("haruto_description"), "urdf", "robot.xacro"]
      ),
      " ",
      "robot_type:=",
      robot_type,
    ]
  )
  robot_description = {"robot_description": robot_description_content}

  # robot state publisher for publishing robot model
  robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[{"use_sim_time": True}, robot_description]
  )

  # gazebo node with empty world
  gazebo_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'])
  )

  # node to spawn robot model in gazebo
  spawn_entity_node = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-topic', 'robot_description',
							 '-entity', 'robot'],
    output='screen')
 
  # rviz node for visualizing robot model
  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rviz_config_file],
  )

  # delay start of gazebo until start of robot state publisher
  delay_rviz_after_spawn_entity = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=spawn_entity_node,
      on_exit=[
        LogInfo(msg='Spawned robot model in gazebo. Starting rviz'),
        rviz_node],
    )
  )

  nodes_to_start = [
    robot_state_publisher_node,
    gazebo_node,
    spawn_entity_node,
    delay_rviz_after_spawn_entity,
  ]

  return nodes_to_start


def generate_launch_description():
  declared_arguments = []

  declared_arguments.append(
    DeclareLaunchArgument(
      "robot_type",
      default_value="omni",
      description="Type of robot to visualize (Available options: omni, diff)",
    )
  )

  return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
