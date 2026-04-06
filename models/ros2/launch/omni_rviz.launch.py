import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    rate = LaunchConfiguration('rate', default=50.0)  # Hz, default is 10 so we're increasing that a bit.  Funny enough joint and robot state publishers don't have the same name for that parameter :-(
    sensable_config = LaunchConfiguration('sensable_config')
    fake_hardware = LaunchConfiguration('fake_hardware')

    # urdf config
    urdf_file_name = 'omni.urdf'
    urdf = os.path.join(
        get_package_share_directory('sensable_phantom_models'),
        'urdf',
        urdf_file_name)

    # rviz config
    rviz_file_name = 'omni.rviz'
    rviz = os.path.join(
        get_package_share_directory('sensable_phantom_models'),
        'rviz',
        rviz_file_name)

    # sensable config
    sensable_phantom_node = Node(
        package='sensable_phantom',
        executable='sensable_phantom',
        name='sensable_phantom',
        output='screen',
        arguments=['-j', sensable_config],
        condition=UnlessCondition(fake_hardware)
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['arm/measured_js'],
                     'rate': rate
        }],
        condition=UnlessCondition(fake_hardware)
    )

    # Let's add a conditional for fake hardware that also uses joint_state_publisher
    # but without source_list.  Wait, the user wants the GUI version!
    fake_joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(fake_hardware)
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(
                         Command(['xacro ', str(urdf)]), value_type=str),
                     'publish_frequency': rate
        }],
        arguments=[urdf]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sensable_config',
            default_value='',
            description='JSON configuration file for sawSensablePhantom'),
        DeclareLaunchArgument(
            'fake_hardware',
            default_value='false',
            choices=['true', 'false'],
            description='If true, don\'t start the nodes for sensable_phantom and joint_state_publisher'),
        sensable_phantom_node,
        joint_state_publisher_node,
        fake_joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
