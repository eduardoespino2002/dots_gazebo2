
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from scripts import GazeboRosPaths
import sys
import xacro

import subprocess

'''
Start the simulator and provide the robot model on a topic so controller
containers can spawn an instance into the simulator
'''

def generate_launch_description():

    # Package directories
    pkg_share       = get_package_share_directory('dots_sim')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')

    # Launch configuration variables
    world_file      = LaunchConfiguration('world_file')
    use_sim_time    = LaunchConfiguration('use_sim_time')
    use_rviz        = LaunchConfiguration('use_rviz')
    use_gdb         = LaunchConfiguration('use_gdb')

    # Declare launch arguments
    declare_world_file      = DeclareLaunchArgument('world_file', 
                                    default_value=os.path.join(pkg_share, 'worlds', 'arena.world'))
    declare_use_sim_time    = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_use_rviz        = DeclareLaunchArgument('use_rviz', default_value='False')
    declare_use_gdb         = DeclareLaunchArgument('use_gdb', default_value='False')

    # Put the robot description (local to the simulation container) on a topic so that
    # controller containers can spawn it. 
    xacro_file          = os.path.join(pkg_share, 'urdf', 'dots.xacro')
    doc                 = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description   = doc.toxml()

    pub_robot_description_cmd = ExecuteProcess(
            cmd = ['ros2', 'topic', 'pub', '/robot_description', 
                    'std_msgs/String', '{data: \'%s\' }' % robot_description],
    )

    # Launch the Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments    = { 'world'     : world_file, 
                                'verbose'   : 'true',
                                'lockstep'  : 'true',
                                'gdb'       : use_gdb,
                                'pause'     : 'true'}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( os.path.join(pkg_share, 'launch', 'gzclient_web.launch.py')),
        launch_arguments={'verbose' : 'true'}.items()
    )

    # Launch rviz as well if requested
    rviz_cmd = ExecuteProcess(
        condition   = IfCondition(use_rviz),
        cmd         = ['rviz2', '-d', pkg_share + '/rviz/arena.rviz'],
        output      = 'screen'
    )


    ld = LaunchDescription()

    # Configs
    ld.add_action(declare_world_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_gdb)

    # Commands
    ld.add_action(pub_robot_description_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(rviz_cmd)


    return ld
    

