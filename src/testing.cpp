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

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>

double cosRialzato(double sigma, double lambda, double threshold)
{
  double tmp, reg;

  sigma = abs(sigma);
  if (sigma < threshold)
  {

    tmp = (sigma / threshold) * M_PI;
    reg = lambda * (0.5 * cos(tmp) + 0.5);
    //		cout << "Damping! " << rand();
  }

  else
  {
    //	cout << "No Damping! " << rand();
    reg = 0.0;
  }
  //	cout << reg << "\t" << sigma << "\n";

  return reg;
}

Eigen::MatrixXd eig_pinv(Eigen::MatrixXd J, double threshold, double lambda)
{

  int rowJ = J.rows();
  int colJ = J.cols();
  Eigen::MatrixXd pinvJ(colJ, rowJ);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::VectorXd eigValues(rowJ);
  std::complex<double> app;
  for (int i = 0; i < rowJ; i++)
  {
    app = svd.singularValues()(i);
    eigValues(i) = app.real();
  }

  Eigen::VectorXd p(rowJ);
  Eigen::MatrixXd Sinv(colJ, rowJ);
  Sinv.setZero();
  for (int i = 0; i < rowJ; i++)
  {

    p(i) = cosRialzato(eigValues(i), lambda, threshold);

    Sinv(i, i) = eigValues(i) / (std::pow(eigValues(i), 2) + p(i));
  }

  if (J.rows() == 1)
  {
    // cout << "J: \n" << J << "\n\n";
    // cout << "V: \n" << svd.matrixV() << "\n\nSinv\n" << Sinv << "\n\nU trasposto: \n" << svd.matrixU().transpose() << "\n\n\n\n\n";
  }

  pinvJ = svd.matrixV() * Sinv * svd.matrixU().transpose();

  return pinvJ;
}

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

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "franka",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const LOGGER = rclcpp::get_logger("franka");
  
  // Publisher for path of another link
  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("tcp_trajectory", 10);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  const std::string plannerGroup = "panda_arm";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, plannerGroup);
  move_group.startStateMonitor();
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
  const moveit::core::JointModelGroup *joint_model = robot_state->getJointModelGroup(plannerGroup);

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  
  // Settings
  move_group.setPlanningTime(0.1);
  // move_group.setPlanningTime(1.0);
  move_group.setNumPlanningAttempts(10);

  move_group.setPlanningPipelineId("ompl");
  // move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  // move_group.setPlannerId("PTP");
  // move_group.setPlannerId("RRTstarkConfigDefault");
  move_group.setPlannerId("RRTConnectkConfigDefault");

  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // ================================================================================
  // ================================================================================

  auto current_pose = move_group.getCurrentPose().pose;
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = current_pose.orientation.x;
  start_pose.orientation.y = current_pose.orientation.y;
  start_pose.orientation.z = current_pose.orientation.z;
  start_pose.orientation.w = current_pose.orientation.w;

  // double angle = 90;
  // start_pose.orientation.x = sin(angle * M_PI_2 / 180) * 0.0;
  // start_pose.orientation.y = sin(angle * M_PI_2 / 180) * 1.0;
  // start_pose.orientation.z = sin(angle * M_PI_2 / 180) * 0.0;
  // start_pose.orientation.w = cos(angle * M_PI_2 / 180);

  start_pose.position.x = current_pose.position.x + 0.20;
  start_pose.position.y = current_pose.position.y;
  start_pose.position.z = current_pose.position.z - 0.40;

  geometry_msgs::msg::Pose second_pose;

  bool foundIK = robot_state->setFromIK(joint_model, start_pose);
  if (!foundIK)
    RCLCPP_ERROR(LOGGER, "Not joint configuration found");
  else
  {
    move_group.setStartState(*robot_state);
    RCLCPP_INFO(LOGGER, "Start pose done");
  }

  Eigen::VectorXd start_joint;
  robot_state->copyJointGroupPositions(joint_model, start_joint);

  moveit_visual_tools.publishRobotState(*robot_state);
  moveit_visual_tools.trigger();

  // Set a target Pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = start_pose.orientation.x;
  target_pose.orientation.y = start_pose.orientation.y;
  target_pose.orientation.z = start_pose.orientation.z;
  target_pose.orientation.w = start_pose.orientation.w;

  // double angle = 90;
  // target_pose.orientation.x = sin(angle * M_PI_2 / 180) * 0.0;
  // target_pose.orientation.y = sin(angle * M_PI_2 / 180) * 1.0;
  // target_pose.orientation.z = sin(angle * M_PI_2 / 180) * 0.0;
  // target_pose.orientation.w = cos(angle * M_PI_2 / 180);

  target_pose.position.x = start_pose.position.x + 0.25;
  target_pose.position.y = start_pose.position.y;
  target_pose.position.z = start_pose.position.z;

  move_group.setPoseTarget(target_pose);

  // ================================================================================
  // ================================================================================

  
  Eigen::VectorXd joints_positions;
  robot_state->copyJointGroupPositions(joint_model, joints_positions);
  
  Eigen::VectorXd dq_o(7);
  
  Eigen::Vector3d start_p(start_pose.position.x, start_pose.position.y, start_pose.position.z);
  Eigen::Vector3d start_quat(start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z);
  
  Eigen::VectorXd err(6);

  for (int i = 0; i < 7; i++)
    dq_o(i) = 1;
  
  bool pathFound = false;
  unsigned int iter = 0;
  double Ts = 0.1;

  // ================================================================================
  // Orientation constraint
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "panda_tool";
  ocm.header.frame_id = "world";
  ocm.orientation.x = start_pose.orientation.x;
  ocm.orientation.y = start_pose.orientation.y;
  ocm.orientation.z = start_pose.orientation.z;
  ocm.orientation.w = start_pose.orientation.w;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  // move_group.setPathConstraints(test_constraints);
  // ================================================================================

  moveit_msgs::msg::RobotTrajectory cartesian_traj;
  // double eef_step = 1e-3;
  // double frac = 0.0;
  // double jump = 3.0;

  std::vector<geometry_msgs::msg::Pose> pts;
  pts.push_back(start_pose);
  pts.push_back(target_pose);

  unsigned int counter = 0;
  unsigned int maxPath = 500;

  moveit_visual_tools.prompt("Begin plannnig");
  
  do
  {
    ++iter;

    // frac = move_group.computeCartesianPath(pts, eef_step, jump, cartesian_traj);
    // RCLCPP_INFO(LOGGER, "Cartesian path compute at : %f pourcent", frac);
    // if (frac > 0.95)
    // {
    //   pathFound = true;
    //   RCLCPP_INFO(LOGGER, "Cartesian Path %d : %s", iter, pathFound ? "SUCCESS" : "FAILED");
    //   ++counter;
    // }
    // else
    //   pathFound = false;
    
    // --------------------------------------------------------------------------------
    
    pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (pathFound)
    {
      RCLCPP_INFO(LOGGER, "OMPL Path %d : %s", iter, pathFound ? "SUCCESS" : "FAILED");
      ++counter;
    }
    
    // --------------------------------------------------------------------------------

    Eigen::Vector3d ref_pt(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    robot_state->getJacobian(joint_model, robot_state->getLinkModel(joint_model->getLinkModelNames().back()), ref_pt, jacobian);

    // double lambda = 1e-3;
    // Eigen::MatrixXd J_pinv = jacobian.transpose() * (jacobian * jacobian.transpose() + lambda * Eigen::MatrixXd::Identity(6, 6)).inverse();
    Eigen::MatrixXd J_pinv = eig_pinv(jacobian, 0.01, 0.001);
    auto N = Eigen::MatrixXd::Identity(7, 7) - J_pinv * jacobian;

    auto current_tf = robot_state->getGlobalLinkTransform("panda_tool");
    Eigen::Quaterniond tmp_quat(current_tf.rotation());
    Eigen::Vector3d current_quat(tmp_quat.x(), tmp_quat.y(), tmp_quat.z());

    Eigen::Vector3d pos_err = start_p - current_tf.translation();
    Eigen::Vector3d quat_err = (start_pose.orientation.w * -current_quat) + (tmp_quat.w() * start_quat) + start_quat.cross(current_quat);

    err.head<3>() = pos_err;
    err.tail<3>() = quat_err;

    auto dq = J_pinv * (0.3 * err) + N * dq_o;
    joints_positions += Ts * dq;

    robot_state->setJointGroupPositions(joint_model, joints_positions);

    if (!robot_state->satisfiesBounds(joint_model))
    {
      RCLCPP_INFO(LOGGER, "%d", iter);
      robot_state->setJointGroupPositions(joint_model, start_joint);
      joints_positions = start_joint;

      dq_o = -dq_o;
    }

    move_group.setStartState(*robot_state);

    robot_state->update();
    moveit_visual_tools.publishRobotState(*robot_state);
    moveit_visual_tools.trigger();
    rclcpp::sleep_for(std::chrono::milliseconds(10));

  } while (iter < maxPath && !pathFound);
 
  RCLCPP_INFO(LOGGER, "Error position : %f meter(s)", err.head<3>().norm());
  RCLCPP_INFO_STREAM(LOGGER, "Error quaternion : \n" << err.tail<3>() << "\n");
  
  RCLCPP_INFO(LOGGER, "Nb waypoints OMPL : %ld", plan.trajectory_.joint_trajectory.points.size());
  RCLCPP_INFO(LOGGER, "Nb waypoints Cartesian : %ld", cartesian_traj.joint_trajectory.points.size());

  // ================================================================================
  // auto joints = plan.trajectory_.joint_trajectory;
  // auto points = joints.points;

  // for (std::size_t i = 0; i < points.size(); i++)
  // {
  //   RCLCPP_INFO(LOGGER, "Waypt %ld - vel : %f", i, points[i].velocities[3]);
  //   RCLCPP_INFO(LOGGER, "Waypt %ld - accel : %f", i, points[i].accelerations[3]);
  // }

  // ================================================================================
  // Compute plan from the current pose to the new start pose
  
  // move_group.setPlanningPipelineId("ompl");
  // move_group.setPlannerId("RRTConnectkConfigDefault");

  // moveit::planning_interface::MoveGroupInterface::Plan new_start;  
  // std::vector<double> jp;
  // for (int i = 0; i < 7; i++)
  //   jp.push_back(joints_positions(i));
    
  // move_group.setStartStateToCurrentState();
  // move_group.setJointValueTarget(jp);
  
  // moveit_visual_tools.prompt("From ready pose to new start");
  // do
  // {
  //   pathFound = move_group.plan(new_start) == moveit::core::MoveItErrorCode::SUCCESS;
  //   RCLCPP_INFO(LOGGER, "OMPL Path %s", pathFound ? "SUCCESS" : "FAILED");
  // } while (!pathFound);

  // ================================================================================
  // Compute plan second tim from new start pose to goal

  // move_group.setPlanningPipelineId("ompl");
  // move_group.setPlannerId("RRTstarkConfigDefault");

  // robot_state->setJointGroupPositions(joint_model, jp);
  // move_group.setStartState(*robot_state);
  // move_group.setPoseTarget(target_pose);

  // do
  // {
  //   pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  //   RCLCPP_INFO(LOGGER, "OMPL Path %s", pathFound ? "SUCCESS" : "FAILED");
  // } while (!pathFound);

  // ================================================================================
  // Draw panda_link8 path + end_effector
  
  // moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model);
  robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), "panda_arm");
  rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);
  publishTcpTrajectory(rt, "panda_tool", marker_pub);
  moveit_visual_tools.trigger();
  
  // moveit_visual_tools.publishTrajectoryLine(new_start.trajectory_, joint_model);
  // moveit_visual_tools.trigger();

  // moveit_visual_tools.prompt("Ready to execute");
  // move_group.execute(new_start);
  // moveit_visual_tools.prompt("Ready to execute _2");
  // move_group.execute(plan);

  // ================================================================================
  // trajectory processing (acceleration - velocity)

  /**
   * Replace this "part" with s-curve of Clément (need adaptation first)
   */
  

  // trajectory_processing::RuckigSmoothing traj_proc;
  // traj_proc.applySmoothing(rt);

  // trajectory_processing::TimeOptimalTrajectoryGeneration traj_proc;
  // traj_proc.computeTimeStamps(rt);  
  
  // moveit_msgs::msg::RobotTrajectory new_traj;
  // rt.getRobotTrajectoryMsg(new_traj);
  // moveit_visual_tools.publishTrajectoryLine(new_traj, joint_model);
  // moveit_visual_tools.trigger();

  // RCLCPP_INFO(LOGGER, "Nb waypoints time_param plan : %ld", rt.getWayPointCount());
  // for (std::size_t i = 0; i < rt.getWayPointCount(); i++)
  // {
  //   if (rt.getWayPoint(i).hasVelocities())
  //     RCLCPP_INFO(LOGGER, "Waypt %ld - vel : %f", i, *(rt.getWayPoint(i).getJointVelocities("panda_joint4")));
  //   if (rt.getWayPoint(i).hasAccelerations())
  //     RCLCPP_INFO(LOGGER, "Waypt %ld - accel : %f", i, *(rt.getWayPoint(i).getJointAccelerations("panda_joint4")));
  // }
  
  // ================================================================================
  // NOTES
  
  //  RobotTrajectory.trajectory_.joint_trajectory.points --> waypoints
  //  waypoint :
  //    time_from_start
  //    positions[] --> for each joint
  //    velocities[]
  //    accelerations[]
  //    effort[]

   
  /**
  * computeCartesianPath :
  *   - very fast
  *   - straight line
  *   - 
  * OMPL plan :
  *   - could found solution before cartesian (nullspace search)
  *   - time depend algo (RRT* need optimization time, connect doesn't care)
  *   - 
  **/

  move_group.clearPathConstraints();
  
  rclcpp::shutdown();
  spinner.join();
  return 0;
}