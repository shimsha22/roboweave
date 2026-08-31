from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # AMR_1: High Priority (Traverses full row 3 from left to right)
        Node(
            package='amr_coordination2',
            executable='amr_node',
            name='amr_1_node',
            output='screen',
            additional_env={'AMR_ID': 'AMR_1', 'AMR_PRIORITY': '3', 'AMR_START': '3,0', 'AMR_GOAL': '3,14'}
        ),
        # AMR_2: Medium Priority (Head-on opposition across row 3 from right to left)
        Node(
            package='amr_coordination2',
            executable='amr_node',
            name='amr_2_node',
            output='screen',
            additional_env={'AMR_ID': 'AMR_2', 'AMR_PRIORITY': '2', 'AMR_START': '3,14', 'AMR_GOAL': '3,0'}
        ),
        # AMR_3: Low Priority (Perpendicular cross cutting right through the middle)
        Node(
            package='amr_coordination2',
            executable='amr_node',
            name='amr_3_node',
            output='screen',
            additional_env={'AMR_ID': 'AMR_3', 'AMR_PRIORITY': '1', 'AMR_START': '0,7', 'AMR_GOAL': '6,7'}
        )
    ])