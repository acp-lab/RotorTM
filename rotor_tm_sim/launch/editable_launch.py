from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
#from launch.frontend import Component
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    controller_launches = []
    pkg_path = get_package_share_directory("rotor_tm_config") 
    sim_pkg_path = get_package_share_directory("rotor_tm_sim")
    ##NMPC controller settings
    group_ns = "nmpc_control"   #namespace
    c_node = ComposableNode(package="rotor_tm_plcontrol",
                                     namespace=group_ns,
                                     name="nmpc_control_nodelet",
                                     remappings=[                                       
                                     ],
                                     plugin="nmpc_control_nodelet::NMPCControlNodelet",
                                     parameters=[])

    return LaunchDescription([
        DeclareLaunchArgument(
            'payload_params_path',
            # default_value=os.path.join(pkg_path , 'config', 'load_params/pointmass_payload.yaml'),
            default_value=os.path.join(pkg_path , 'config', 'load_params/triangular_payload.yaml'),
            description='Path to the payload parameters file'
        ),
        DeclareLaunchArgument(
            'uav_params_path',
            default_value=os.path.join(pkg_path , 'config', 'uav_params/'),
            description='Path to the UAV parameters file'
        ),
        DeclareLaunchArgument(
            'mechanism_params_path',
            # default_value=os.path.join(pkg_path , 'config', 'attach_mechanism/cable/1_robot_point_mass_0-5m.yaml'),
            default_value=os.path.join(pkg_path , 'config', 'attach_mechanism/cable/3_robots_triangular_payload_0-5m.yaml'),
            description='Path to the mechanism parameters file'
        ),
        DeclareLaunchArgument(
            'payload_control_gain_path',
            # default_value=os.path.join(pkg_path , 'config', 'control_params/pointmass_cable_gains.yaml'), 
            default_value=os.path.join(pkg_path , 'config', 'control_params/triangular_payload_cooperative_cable_gains.yaml'),
            #description='Path to the payload control gains file'
        ),
        DeclareLaunchArgument(
            'uav_control_gain_path',
            default_value=os.path.join(pkg_path , 'config', 'control_params/'),
            description='Path to the UAV control gains file'
        ),
        DeclareLaunchArgument(
            'number_of_robots',
            default_value='3',
            description='Number of UAVs'
        ),
        
        #Start simulation and trajectory nodes 
        Node(
            package='rotor_tm_sim',
            executable='sim_node',
            name='sim_node',
            output='screen',
            arguments=[
                LaunchConfiguration('payload_params_path'),
                LaunchConfiguration('uav_params_path'),
                LaunchConfiguration('mechanism_params_path'),
                LaunchConfiguration('payload_control_gain_path'),
                LaunchConfiguration('uav_control_gain_path')
            ],
        ),
        
        Node(
            package='rotor_tm_traj',
            executable='traj_node',
            name='traj',
            output='screen',
        ),
        
        # # # Recursively launch controller         
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_pkg_path , 'launch', 'controller_launcher.py')),
            condition=IfCondition(PythonExpression([ LaunchConfiguration('number_of_robots'), ' > 0'])),
            launch_arguments={
                'uav_params_path': LaunchConfiguration('uav_params_path'),
                'payload_params_path': LaunchConfiguration('payload_params_path'),
                'mechanism_params_path': LaunchConfiguration('mechanism_params_path'),
                'payload_control_gain_path': LaunchConfiguration('payload_control_gain_path'),
                'uav_control_gain_path': LaunchConfiguration('uav_control_gain_path'),
                'number_of_robots': LaunchConfiguration('number_of_robots'),
                'nr': LaunchConfiguration('number_of_robots')
            }.items()
        ),

        ##NMPC controller
        ComposableNodeContainer(name='container_1',
                                namespace=group_ns,
                                package='rclcpp_components',
                                executable='component_container',
                                composable_node_descriptions=[
                                  c_node
                                ],
                                output='screen',
                                emulate_tty=True),
           
                
        # Include RViz launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_pkg_path , 'launch', 'rviz.py')),

            ),
    ])


# === Other scenarios (for reference) ===

  # 1. 1 dragonfly with pointmass payload using cable mechanisms (0.5 m)
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/pointmass_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/1_robot_point_mass_0-5m.yaml"
  # payload_control_gain_path: "control_params/pointmass_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 1
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 2. 3 snapdragon with bullnose propeller flights with triangular payload using cable mechanisms (1 m)
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/triangular_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/3_robots_triangular_payload_1m_bullnose_dragonfly.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 3
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 3. 3 dragonfly with triangular payload using cable mechanisms (0.5 m)
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/triangular_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/3_robots_triangular_payload_0-5m.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 3
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 4. 2 dragonfly with long bar payload using cable mechanisms (0.5 m)
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/long_bar_wood_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/2_robots_long_bar_payload_0-5m.yaml"
  # payload_control_gain_path: "control_params/long_bar_payload_cooperative_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 2
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 5. 3 dragonfly with triangular payload using cable mechanisms with different cable lengths (2 robots 1m, 1 robot 0.5m)
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/triangular_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/3_robots_triangular_payload_1m_0-5m_different_cable_length.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 3
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 6. 3 dragonfly with triangular payload using cable mechanisms with different cable lengths (1.2m, 1m, 0.8m)
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/triangular_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/3_robots_triangular_payload_1-2m_1m_0-8m_different_cable_length.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 3
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 7. race, snapdragon, hummingbird with triangular payload using cable mechanisms
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/triangular_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/3_robots_triangular_payload_1m_different_robots_dynamics.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 3
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 8. 4 dragonfly with fedex box payload using cable mechanisms
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/fedex_box_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/4_robots_fedex_box_0_5m.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 4
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 9. 6 dragonfly with fedex box payload using cable mechanisms
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/fedex_box_payload.yaml"
  # mechanism_params_path: "attach_mechanism/cable/6_robots_fedex_box_0_5m.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_cable_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 6
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 10. race, snapdragon, hummingbird with triangular payload using rigid link mechanisms
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/triangular_payload.yaml"
  # mechanism_params_path: "attach_mechanism/rigid_link/3_hetereogeneous_robots_triangular_payload.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_rigidlink_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 3
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 11. 2 dragonfly with long bar payload using rigid link mechanisms
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/long_bar_wood_payload.yaml"
  # mechanism_params_path: "attach_mechanism/rigid_link/2_robots_long_bar.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_rigidlink_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 2
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 12. 3 dragonfly with triangular payload using rigid link mechanisms
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/triangular_payload.yaml"
  # mechanism_params_path: "attach_mechanism/rigid_link/3_robots_triangular_payload.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_rigidlink_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 3
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

  # 13. 3 dragonfly bullnose propeller with triangular payload using rigid link mechanisms
  # uav_params_path: "uav_params/"
  # payload_params_path: "load_params/triangular_payload.yaml"
  # mechanism_params_path: "attach_mechanism/rigid_link/3_robots_triangular_payload_bullnose_dragonfly.yaml"
  # payload_control_gain_path: "control_params/triangular_payload_cooperative_rigidlink_gains.yaml"
  # uav_control_gain_path: "control_params/"
  # number_of_robots: 3
  # nmpc_filename: "control_params/payload_nmpc_params.yaml"

