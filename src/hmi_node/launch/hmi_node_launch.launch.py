import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
		launch_ros.actions.Node(
                package='hmi_node', node_executable='hmi_node_server', output='screen', parameters=["/home/riccardochiaretti/alba_v2_hmi/param/config_params.yaml"],
                node_name='hmi_node_server', emulate_tty=True)
    ])
