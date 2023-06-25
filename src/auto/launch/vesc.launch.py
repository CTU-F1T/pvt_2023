import launch
import launch.launch_description_sources
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='config',
            description='vehicle config',
            # default_value='tx2-auto-3.yaml',
        ),

        launch.actions.SetLaunchConfiguration(
            name='config_file',
            value=launch.substitutions.PathJoinSubstitution([
                launch_ros.substitutions.FindPackageShare(package='auto'),
                'config',
                launch.substitutions.LaunchConfiguration(variable_name='config'),
            ]),
        ),

        launch.actions.LogInfo(
            msg=['using config file = ', launch.substitutions.LaunchConfiguration(variable_name='config_file')],
        ),

        # vesc_driver
        launch_ros.actions.Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            output='screen',
            name='vesc',
            parameters=[
                launch.substitutions.LaunchConfiguration(variable_name='config_file')
            ],
        ),
    ])
