import launch   
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_package_path = get_package_share_directory('robot_description')
    default_urdf_file_path = os.path.join(urdf_package_path, 'urdf', 'first_robot.urdf')
    default_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')

    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=default_urdf_file_path,
        description='加载的urdf文件路径参数'
    ) 
    

    #robot_state_publisher节点需要读取urdf文件内容，而不是文件路径，因此需要使用LaunchConfiguration来获取参数值内容
    substitutions_command_result = launch.substitutions.Command(['cat ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type = str)

    action_robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_value}]
    )
    
    action_joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    action_rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_config_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher_node,
        action_joint_state_publisher_node,
        action_rviz2_node,
    ])        