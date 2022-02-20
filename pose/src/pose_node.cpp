#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("braccio");
  
  // move group interface
  
  static const std::string PLANNING_GROUP = "braccio";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  
  ROS_INFO("Reference frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", move_group_interface.getEndEffectorLink().c_str());
 
  //move_group_interface.setMaxVelocityScalingFactor(1);
  //move_group_interface.setMaxAccelerationScalingFactor(1);
  
  // construct a robot model
  
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str()); 
   
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("braccio");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Planning to a Pose goal in cartesian space
  
  //move_group_interface.setGoalJointTolerance(0.001);

  double x1, y1, z1, pitch1, x2, y2, z2, pitch2, x3, y3, z3, pitch3, x4, y4, z4, pitch4, x5, y5, z5, pitch5;
 
  node_handle.getParam("/pose_node/x1", x1);      
  node_handle.getParam("/pose_node/y1", y1);      
  node_handle.getParam("/pose_node/z1", z1);      
  node_handle.getParam("/pose_node/pitch1", pitch1);
  
  node_handle.getParam("/pose_node/x2", x2);      
  node_handle.getParam("/pose_node/y2", y2);      
  node_handle.getParam("/pose_node/z2", z2);      
  node_handle.getParam("/pose_node/pitch2", pitch2);
  
  node_handle.getParam("/pose_node/x3", x3);      
  node_handle.getParam("/pose_node/y3", y3);      
  node_handle.getParam("/pose_node/z3", z3);      
  node_handle.getParam("/pose_node/pitch3", pitch3);
  
  node_handle.getParam("/pose_node/x4", x4);      
  node_handle.getParam("/pose_node/y4", y4);      
  node_handle.getParam("/pose_node/z4", z4);      
  node_handle.getParam("/pose_node/pitch4", pitch4);
  
  node_handle.getParam("/pose_node/x5", x5);      
  node_handle.getParam("/pose_node/y5", y5);      
  node_handle.getParam("/pose_node/z5", z5);      
  node_handle.getParam("/pose_node/pitch5", pitch5);
              
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  
  std::vector<double> joint_values_initial = {0, -2, 0.2, 0};  // alzo giunto 2 mantenendo fermi gli altri
  move_group_interface.setJointValueTarget(joint_values_initial);
  move_group_interface.move();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  
  geometry_msgs::PoseStamped target_pose1; // posizione 1
  tf2::Quaternion quaternion1;
  quaternion1.setRPY(0.0, pitch1, 0);
  quaternion1.normalize();
  
  target_pose1.header.frame_id = "world";
  target_pose1.pose.position.x = x1;
  target_pose1.pose.position.y = y1;
  target_pose1.pose.position.z = z1;
  target_pose1.pose.orientation.x = quaternion1.x();
  target_pose1.pose.orientation.y = quaternion1.y();
  target_pose1.pose.orientation.z = quaternion1.z();
  target_pose1.pose.orientation.w = quaternion1.w();
  
  move_group_interface.setJointValueTarget(target_pose1.pose, "link3");
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan1;

  bool joint_success1 = (move_group_interface.plan(joint_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (pose goal) %s", joint_success1 ? "" : "FAILED");
  
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  move_group_interface.move();
 
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  
  geometry_msgs::PoseStamped target_pose2; // posizione 2
  tf2::Quaternion quaternion2;
  quaternion2.setRPY(0.0, pitch2, 0);
  quaternion2.normalize();
  
  target_pose2.header.frame_id = "world";
  target_pose2.pose.position.x = x2;
  target_pose2.pose.position.y = y2;
  target_pose2.pose.position.z = z2;
  target_pose2.pose.orientation.x = quaternion2.x();
  target_pose2.pose.orientation.y = quaternion2.y();
  target_pose2.pose.orientation.z = quaternion2.z();
  target_pose2.pose.orientation.w = quaternion2.w();
  
  move_group_interface.setJointValueTarget(target_pose2.pose, "link3");
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan2;

  bool joint_success2 = (move_group_interface.plan(joint_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (pose goal) %s", joint_success2 ? "" : "FAILED");
  move_group_interface.move();
 
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  
  geometry_msgs::PoseStamped target_pose3; // posizione 3
  tf2::Quaternion quaternion3;
  quaternion3.setRPY(0.0, pitch3, 0);
  quaternion3.normalize();
  
  target_pose3.header.frame_id = "world";
  target_pose3.pose.position.x = x3;
  target_pose3.pose.position.y = y3;
  target_pose3.pose.position.z = z3;
  target_pose3.pose.orientation.x = quaternion3.x();
  target_pose3.pose.orientation.y = quaternion3.y();
  target_pose3.pose.orientation.z = quaternion3.z();
  target_pose3.pose.orientation.w = quaternion3.w();
  
  move_group_interface.setJointValueTarget(target_pose3.pose, "link3");
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan3;

  bool joint_success3 = (move_group_interface.plan(joint_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (pose goal) %s", joint_success3 ? "" : "FAILED");
  move_group_interface.move();
 
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  
  geometry_msgs::PoseStamped target_pose4; // posizione 4
  tf2::Quaternion quaternion4;
  quaternion4.setRPY(0.0, pitch4, 0);
  quaternion4.normalize();
  
  target_pose4.header.frame_id = "world";
  target_pose4.pose.position.x = x4;
  target_pose4.pose.position.y = y4;
  target_pose4.pose.position.z = z4;
  target_pose4.pose.orientation.x = quaternion4.x();
  target_pose4.pose.orientation.y = quaternion4.y();
  target_pose4.pose.orientation.z = quaternion4.z();
  target_pose4.pose.orientation.w = quaternion4.w();
  
  move_group_interface.setJointValueTarget(target_pose4.pose, "link3");
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan4;

  bool joint_success4 = (move_group_interface.plan(joint_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (pose goal) %s", joint_success4 ? "" : "FAILED");
  move_group_interface.move();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");
  
  geometry_msgs::PoseStamped target_pose5; // posizione 5
  tf2::Quaternion quaternion5;
  quaternion5.setRPY(0.0, pitch5, 0);
  quaternion5.normalize();
  
  target_pose5.header.frame_id = "world";
  target_pose5.pose.position.x = x5;
  target_pose5.pose.position.y = y5;
  target_pose5.pose.position.z = z5;
  target_pose5.pose.orientation.x = quaternion5.x();
  target_pose5.pose.orientation.y = quaternion5.y();
  target_pose5.pose.orientation.z = quaternion5.z();
  target_pose5.pose.orientation.w = quaternion5.w();
  
  move_group_interface.setJointValueTarget(target_pose5.pose, "link3");
  moveit::planning_interface::MoveGroupInterface::Plan joint_plan5;

  bool joint_success5 = (move_group_interface.plan(joint_plan5) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan 1 (pose goal) %s", joint_success5 ? "" : "FAILED");
  move_group_interface.move();
 
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue");

  std::vector<double> joint_values_retracted = {0, 0, 0, 0};  // ritorno configurazione 0
  move_group_interface.setJointValueTarget(joint_values_retracted);
  move_group_interface.move();
      
  ros::shutdown();
  return 0;
}
