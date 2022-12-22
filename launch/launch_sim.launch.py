import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():



    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='nsbot' 
    pkg_share = FindPackageShare(package='nsbot').find('navbot')    
    world_file_name = 'empty.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
      name='world',
      default_value=world_path,
      description='Full path to the world model file to load')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )



    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')]),
      #condition=IfCondition(use_simulator),
      launch_arguments={'world': world}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')]),
      #condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))
    )


    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'nsbot'],
                        output='screen')


    # Launch them all!
    return LaunchDescription([
        declare_world_cmd,
        rsp,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,

    ])

