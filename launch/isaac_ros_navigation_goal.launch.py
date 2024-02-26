import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# nvblox_ros_navigation_goal = get_package_share_directory("nvblox_ros_navigation_goal")

def generate_launch_description():

    nav_goal_dir = get_package_share_directory('nvblox_ros_navigation_goal')

    global_frame = LaunchConfiguration(
        "frame_id",
        default='odom'
    )

    map_yaml_file = LaunchConfiguration(
        "map_yaml_path",
        default=os.path.join(
            nav_goal_dir, "maps", "isaac_test_map.yaml"
        ),
    )

    goal_text_file = LaunchConfiguration(
        "goal_text_file_path",
        default=os.path.join(
            nav_goal_dir, "maps", "goals.txt"),
    )


    navigation_goal_node = Node(
        name="set_navigation_goal",
        package="nvblox_ros_navigation_goal",
        executable="SetNavigationGoal",
        parameters=[
            {
                "initial_pose": [0.5, -0.87, 0.0, 0.0, 0.0, 0.70, 0.70],
                "map_yaml_path": map_yaml_file,
                # "goal_text_file_path": goal_text_file,
                "frame_id": global_frame,
                "action_server_name": "navigate_to_pose",
                "obstacle_search_distance_in_meters": 0.2,
                
            }
        ],
        output="screen",
    )

    return LaunchDescription([navigation_goal_node])
