from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
      pub_sub_node = Node(
        package='ros2-multithreading',
        executable='pub_sub2',
        name='pub_sub2_node',
      )

      # Define launch description
      ld = LaunchDescription()
      ld.add_action(pub_sub_node)

      # Return launch description
      return ld