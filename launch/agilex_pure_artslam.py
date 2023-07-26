from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    config_file = os.path.join(
      get_package_share_directory('artslam_wrapper'),
      'config',
      'KITTI.json'
      )

    artslam_wrapper_node = Node(
        package='artslam_wrapper',
        executable='artslam_wrapper_node',
        name='wrapper_controller',
        output='screen',
        parameters=[{
            'configuration_file': config_file,
            'results_path': '/LOTS/results/',
            # 'imu_topic': '/ouster/imu'
        }],
        # prefix=['xterm -e gdb -q -ex run --args']
    )

    ld.add_action(artslam_wrapper_node)

    return ld
