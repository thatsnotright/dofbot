from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # MTC node
    pick_place = Node(
        # package="mtc",
        # executable="mtc_node",
        package="mtc",
        executable="mtc_node",
        output="screen",
        parameters=[
        ],
    )

    return LaunchDescription([pick_place])
