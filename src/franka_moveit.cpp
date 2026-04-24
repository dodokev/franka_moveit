#include <memory>
#include <thread> 

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/empty.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>

void publishTcpTrajectory(
  const robot_trajectory::RobotTrajectory& trajectory,
  const std::string& tcp_link,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";  // base frame
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "tcp_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.scale.x = 0.005; // line thickness

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.orientation.w = 1.0;

    // Iterate over trajectory waypoints
    for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
      const moveit::core::RobotState& state = trajectory.getWayPoint(i);

      // From state at a waypoint, get transform of tcp_link from panda_tool0
      const Eigen::Isometry3d& tcp_pose = state.getGlobalLinkTransform(tcp_link);
      
      // Create new point from the pose
      geometry_msgs::msg::Point p;
      p.x = tcp_pose.translation().x();
      p.y = tcp_pose.translation().y();
      p.z = tcp_pose.translation().z();

      marker.points.push_back(p);
    }

    pub->publish(marker);
  }


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
    "franka",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const LOGGER = rclcpp::get_logger("franka");

  // Publisher for path of another link
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("tcp_trajectory", 10);
  auto start_pub = node->create_publisher<geometry_msgs::msg::Pose>("start_pose", 10);
  
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(node);
  // auto spinner = std::thread([&executor]() { executor.spin(); });
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  const std::string plannerGroup = "panda_arm";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, plannerGroup);
  move_group.startStateMonitor();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
  const moveit::core::JointModelGroup* joint_model = robot_state->getJointModelGroup(plannerGroup);

  // moveit::core::RobotModelPtr robot_model = move_group.getRobotModel();
  // Eigen::Vector3d ref_pt(0.0, 0.0, 0.0);
  // Eigen::MatrixXd jacobian;
  // robot_state->getJacobian(joint_model, robot_state->getLinkModel(joint_model->getLinkModelNames().back()),
  //                        ref_pt, jacobian);
  // RCLCPP_INFO_STREAM(LOGGER, "Jacobian: \n" << jacobian << "\n");

  // Settings
  double Ts = 0.1;

  move_group.setPlanningTime(30.0);

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnectkConfigDefault");
  
  // RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  // move_group.setEndEffectorLink("panda_tool");
  // RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  auto goal_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "target_pose",
    10,
    [&move_group, robot_state, joint_model, &LOGGER](geometry_msgs::msg::Pose p)
    {
      RCLCPP_INFO(LOGGER, "Received target pose");
      bool foundIK = robot_state->setFromIK(joint_model, p);
      if (!foundIK)
        RCLCPP_ERROR(LOGGER, "Not joint configuration found");
      else
      {
        std::vector<double> joints_positions;
        robot_state->copyJointGroupPositions(joint_model, joints_positions);

        move_group.setJointValueTarget(joints_positions);
        RCLCPP_INFO(LOGGER, "Target pose done");
      }
    }
  );

  auto start_sub = node->create_subscription<geometry_msgs::msg::Pose>(
    "start_pose",
    10,
    [&move_group, robot_state, joint_model, &LOGGER](geometry_msgs::msg::Pose p)
    {
      RCLCPP_INFO(LOGGER, "Received start pose");
      bool foundIK = robot_state->setFromIK(joint_model, p);
      if (!foundIK)
        RCLCPP_ERROR(LOGGER, "Not joint configuration found");
      else
      {
        move_group.setStartState(*robot_state);
        RCLCPP_INFO(LOGGER, "Start pose done");
      }
    }
  );

  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  bool pathFound = false;
  auto plan_sub = node->create_subscription<std_msgs::msg::Empty>(
    "start_planning",
    10,
    [&move_group, robot_state, joint_model, &plan, &LOGGER, &pathFound, &Ts, &moveit_visual_tools, &marker_pub](std_msgs::msg::Empty){ 
      pathFound = false;
      unsigned int iter = 0;
      do
      {
        ++iter;
        
        pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        RCLCPP_INFO(LOGGER, "Path %d : %s", iter, pathFound ? "SUCCESS" : "FAILED");

        Eigen::VectorXd joints_positions;
        robot_state->copyJointGroupPositions(joint_model, joints_positions);
        
        Eigen::Vector3d ref_pt(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        robot_state->getJacobian(joint_model, robot_state->getLinkModel(joint_model->getLinkModelNames().back()), ref_pt, jacobian);

        auto N = Eigen::MatrixXd::Identity(7, 7) -  jacobian.transpose() * jacobian;
        Eigen::VectorXd q_o(7);
        for (int i = 0; i < 7; i++)
          q_o(i) = 1;

        auto dq = N * q_o;
        joints_positions += Ts*dq;

        robot_state->setJointGroupPositions(joint_model, joints_positions);
        move_group.setStartState(*robot_state);

        moveit_visual_tools.prompt("WAIT");
        moveit_visual_tools.prompt("BUG");
      } while (iter < 30);

      RCLCPP_INFO(LOGGER, "Let's draw ?");
      
      moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model);
      robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), "panda_arm");
      // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

      rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);

      publishTcpTrajectory(rt, "panda_tool", marker_pub);
      moveit_visual_tools.trigger();
    }
  );


  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    // robot_state = move_group.getCurrentState();
    rate.sleep();
  }

  // auto txt_pose = Eigen::Isometry3d::Identity();
  // txt_pose.translation().z() = 1.0;

  // auto current_pose = move_group.getCurrentPose();
  
  // ================================================================================
  
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // std::vector<std::string> objects_id;

  // moveit_msgs::msg::CollisionObject collision_obj;
  // collision_obj.header.frame_id = "world";
  // collision_obj.id = "simple_wall";

  // shape_msgs::msg::SolidPrimitive solid;
  // solid.type = solid.BOX;
  // solid.dimensions.resize(3);
  // solid.dimensions[solid.BOX_X] = 0.5;
  // solid.dimensions[solid.BOX_Y] = 0.05;
  // solid.dimensions[solid.BOX_Z] = 1.0;

  // geometry_msgs::msg::Pose wall_pose;
  // wall_pose.orientation.x = 0.0;
  // wall_pose.orientation.y = 0.0;
  // wall_pose.orientation.z = 0.0;
  // wall_pose.orientation.w = 0.0;
  // wall_pose.position.x = 0.6;
  // wall_pose.position.y = 0.0;
  // wall_pose.position.z = 0.1;

  // collision_obj.primitives.emplace_back(solid);
  // collision_obj.primitive_poses.emplace_back(wall_pose);
  // collision_obj.operation = collision_obj.ADD;
  
  // objects_id.push_back(collision_obj.id);

  // planning_scene_interface.applyCollisionObject(collision_obj);
  // moveit_visual_tools.prompt("Wall constructed, press Next to plan");
  // RCLCPP_INFO(LOGGER, "Wall constructed");
  
  // ================================================================================

  // geometry_msgs::msg::Pose start_pose;
  // start_pose.orientation.x = 0.71;
  // start_pose.orientation.y = 0.0;
  // start_pose.orientation.z = 0.71;
  // start_pose.orientation.w = 0.0;
  // start_pose.position.x = current_pose.pose.position.x + 0.2; // .507020
  // start_pose.position.y = current_pose.pose.position.y - 0.5; // -.5
  // start_pose.position.z = current_pose.pose.position.z - 0.1; // .379870

  // RCLCPP_INFO(LOGGER, "Start pose : %f, %f, %f, %f, %f, %f, %f",
  //   start_pose.orientation.x,
  //   start_pose.orientation.y,
  //   start_pose.orientation.z,
  //   start_pose.orientation.w,
  //   start_pose.position.x,
  //   start_pose.position.y,
  //   start_pose.position.z
  // );

  // bool foundIK = robotState->setFromIK(joint_model, start_pose);
  // if (!foundIK)
  //   RCLCPP_ERROR(LOGGER, "Not joint configuration found");
  // else
  //   move_group.setStartState(*robotState);
  
  
  // Set a target Pose
  // geometry_msgs::msg::Pose target_pose;
  // target_pose.orientation.x = 0.71;
  // target_pose.orientation.y = 0.0;
  // target_pose.orientation.z = 0.71;
  // target_pose.orientation.w = 0.0;
  // target_pose.position.x = start_pose.position.x;
  // target_pose.position.y = start_pose.position.y + 1.0;
  // target_pose.position.z = start_pose.position.z;

  // auto target_state = *robotState;
  // foundIK = target_state.setFromIK(joint_model, target_pose);
  // if (!foundIK)
  //   RCLCPP_ERROR(LOGGER, "Not joint configuration found");
  // else
  // {
  //   std::vector<double> joints_positions;
  //   target_state.copyJointGroupPositions(joint_model, joints_positions);

  //   move_group.setJointValueTarget(joints_positions);
  // }

  // bool pathFound = false;
  // unsigned int numPath = 10;
  // unsigned int numFailed = 0;
  // unsigned int numSuccess = 0;

  // for (unsigned int i = 0; i < numPath; i++)
  // {
  //   moveit_visual_tools.publishText(txt_pose, "Generate_Path_" + std::to_string(i), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  //   moveit_visual_tools.trigger();

  //   pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  //   RCLCPP_INFO(LOGGER, "Path %d : %s", i, pathFound ? "SUCCESS" : "FAILED");
    
  //   if (pathFound)
  //   {
  //     ++numSuccess;
  //     // Normal path (panda_link8)
  //     moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model);

  //     // Create robot_trajectory::RobotTrajectory to be able to visualize path from another point
  //     robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), "panda_arm");
  //     rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), plan.trajectory_);
  //     // publish TcpTrajectory for the panda_tool link
  //     publishTcpTrajectory(rt, "panda_tool", marker_pub);

  //     moveit_visual_tools.trigger();

  //     // moveit_visual_tools.prompt("What");
  //     break;
  //   }
  //   else
  //     ++numFailed;
  // }

  // RCLCPP_INFO(LOGGER, "Number Success Path %d", numSuccess);
  // RCLCPP_INFO(LOGGER, "Number Failed Path %d", numFailed);

  // moveit_visual_tools.prompt("Press Next to End");
  // planning_scene_interface.removeCollisionObjects(objects_id);

  // Shutdown ROS

  rclcpp::shutdown();
  spinner.join();
  return 0;
}