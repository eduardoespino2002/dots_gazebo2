
import os
from os import environ
from os import pathsep

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from scripts import GazeboRosPaths


def generate_launch_description():


    pkg_share           = get_package_share_directory('dots_sim')
    pkg_gazebo_ros      = get_package_share_directory('gazebo_ros')

    use_gzweb           = LaunchConfiguration('use_gzweb')
    use_gzclient        = LaunchConfiguration('use_gzclient')
    declare_use_gzweb   = DeclareLaunchArgument('use_gzweb', default_value='False')
    declare_use_gzclient= DeclareLaunchArgument('use_gzclient', default_value='False')


    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep + environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep + environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep + environ['GAZEBO_RESOURCE_PATH']

    print(model, plugin, media)
    env = {'GAZEBO_MODEL_PATH':model, 
            'GAZEBO_PLUGIN_PATH':plugin, 
            'GAZEBO_RESOURCE_PATH':media,
            'PATH':'/usr/bin'}


    gzweb_cmd = ExecuteProcess(
        condition   = IfCondition(use_gzweb),
        #cwd         = pkg_share + '/../../../../../gzweb',
        env         = env,
        #additional_env     = env,
        cmd         = [os.path.join(pkg_share, 'launch', 'start_gzweb.sh')],
        shell       = True,
        #output      = 'screen'
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition           = IfCondition(use_gzclient),
        launch_arguments    = {'verbose'    : 'true'}.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_gzweb)
    ld.add_action(declare_use_gzclient)
    ld.add_action(gzweb_cmd)
    ld.add_action(gzclient_cmd)



    return ld
    

