search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=braccio_morpheus.srdf
robot_name_in_srdf=braccio_morpheus
moveit_config_pkg=braccio_morpheus_moveit_config
robot_name=braccio_morpheus
planning_group_name=braccio
ikfast_plugin_pkg=braccio_morpheus_braccio_ikfast_plugin
base_link_name=base
eef_link_name=link3
ikfast_output_path=/home/giacomo/catkin_ws/src/braccio_morpheus_braccio_ikfast_plugin/src/braccio_morpheus_braccio_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
