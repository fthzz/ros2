import launch   
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_package_path = get_package_share_directory('robot_description')
    default_xacro_file_path = os.path.join(urdf_package_path, 'urdf', 'fishbot/fishbot.urdf.xacro')
    # default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    default_gazebo_config_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')

    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_xacro_file_path,
        description='加载的urdf文件路径参数'
    ) 
    
    #robot_state_publisher节点需要读取urdf文件内容，而不是文件路径，因此需要使用LaunchConfiguration来获取参数值内容
    substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type = str)

    action_robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_value}]
    )
    
    #启动一个launch文件
    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments=[('world',default_gazebo_config_path)]
    )

    action_spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fishbot']
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_joint_state_broadcaster', '--set-state', 'active'],
        output='screen'
    )

    # 力控制器和差速控制器最好不要同时使用，不然轮子前进会乱
    # action_load_effort_controller = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', 'fishbot_effort_controller', '--set-state', 'active'],
    #     output='screen'
    # )

    action_load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'fishbot_diff_drive_controller', '--set-state', 'active'],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher_node,
        action_spawn_entity_node,
        action_launch_gazebo,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_spawn_entity_node,
                on_exit=[action_load_joint_state_controller],
            )
        ),

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=action_load_joint_state_controller,
                on_exit=[action_load_diff_drive_controller],
            )
        )
    ])        