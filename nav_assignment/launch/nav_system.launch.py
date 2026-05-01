from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt', # Using MultiThreaded container for actions
        composable_node_descriptions=[
            ComposableNode(
                package='nav_assignment',
                plugin='nav_assignment::NavServer',
                name='nav_server'
            ),
            ComposableNode(
                package='nav_assignment',
                plugin='nav_assignment::NavClient',
                name='nav_client'
            )
        ],
        output='screen',
    )
    
    return LaunchDescription([container])
