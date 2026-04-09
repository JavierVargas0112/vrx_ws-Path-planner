# 1) T1 - Iniciar mundo (Gazebo)
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta

# 2) T2 - Spawnear WAM-V con bridges ROS2 (IMPORTANTE: usar full)
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch vrx_gz spawn.launch.py world:=sydney_regatta sim_mode:=full name:=wamv model:=wam-v

# 3) T3 - Publicar waypoint GPS desde JSON
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run gps_waypoints gps_waypoint_node --ros-args -p checkpoints_file:=/home/javier/vrx_ws/src/gps_waypoints/checkpoints/sample_checkpoints1.json

# 4) T4 - Convertir GPS -> ENU
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run gps_waypoints gps_waypoint_converter

# 5) T5 - Controlador (modo thrusters, recomendado para este setup)
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 run gps_waypoints gps_waypoint_controller --ros-args -p goal_topic:=/wamv/goal_pose -p state_source:=gps_imu -p gps_topic:=/wamv/sensors/gps/gps/fix -p imu_topic:=/wamv/sensors/imu/imu/data -p control_mode:=thrusters -p left_thrust_topic:=/wamv/thrusters/left/thrust -p right_thrust_topic:=/wamv/thrusters/right/thrust -p left_pos_topic:=/wamv/thrusters/left/pos -p right_pos_topic:=/wamv/thrusters/right/pos -p k_thrust_lin:=82.0 -p k_thrust_ang:=28.0 -p max_thrust:=235.0 -p heading_slowdown_rad:=0.9 -p heading_inplace_rad:=1.35 -p heading_deadband_rad:=0.2 -p turn_close_dist:=20.0 -p min_forward_thrust:=10.0 -p turn_to_forward_ratio:=0.55 -p turn_bias_thrust:=24.0

# Verificación rápida (opcional, en otra terminal)
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic list | grep '^/wamv/'

# Debe haber datos en estos tópicos para que se mueva:
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo --once /wamv/sensors/gps/gps/fix
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo --once /wamv/pose
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo --once /wamv/goal_pose
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo /wamv/thrusters/left/thrust
cd ~/vrx_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 topic echo /wamv/thrusters/right/thrust
