from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    artslam_wrapper_node = Node(
        package='artslam_wrapper',
        executable='artslam_wrapper_node',
        name='wrapper_controller',
        output='screen',
        parameters=[{
            'configuration_file': "/home/gianluca/development/humble/SLAM/artslam_ws/src/ARTSLAM_wrapper/config/KITTI.json",
            'results_path': "/home/gianluca/development/humble/SLAM/artslam_ws/src/ARTSLAM_wrapper/results"
        }],
        # prefix=['xterm -e gdb -q -ex run --args']
    )

    ld.add_action(artslam_wrapper_node)

    return ld
