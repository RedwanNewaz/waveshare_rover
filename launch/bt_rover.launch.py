import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_rover_controller = get_package_share_directory('rover_controller')
    
    tree_path = os.path.join(
        pkg_rover_controller, 'config', 'behavior_tree.xml') 
    

    # Path to the default behavior tree file
   
    return LaunchDescription([
        Node(
            package='rover_controller',
            executable='rover_controller_node',  # Change if your executable has different name
            name='rover_controller',
            output='screen',
            parameters=[
                {'tree_file': tree_path},
                {'wheel_base': 0.075}  # 0.15 / 2.0
            ]
        )
    ])
