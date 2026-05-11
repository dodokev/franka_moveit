#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>

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
  rclcpp::init(argc, argv);
  auto const LOGGER = rclcpp::get_logger("franka");
  RCLCPP_INFO(LOGGER, "RCLCPP ON ...");

  auto const node = std::make_shared<rclcpp::Node>(
      "franka",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  
  RCLCPP_INFO(LOGGER, "NODE OK ...");

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>("tcp_trajectory", 10);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
                             { executor.spin(); });

  const std::string plannerGroup = "fr3_arm";

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "fr3_arm");
  move_group.startStateMonitor();
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
  const moveit::core::JointModelGroup *joint_model = robot_state->getJointModelGroup(plannerGroup);
  
  move_group.setPlanningTime(1.0);
  move_group.setNumPlanningAttempts(10);

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("RRTConnectkConfigDefault");

  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "world", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // ================================================================================
  // ================================================================================

  auto current_pose = move_group.getCurrentPose().pose;
  geometry_msgs::msg::Pose start_pose;
  start_pose = current_pose;

  // start_pose.position.x -= 0.20;
  start_pose.position.z -= 0.40;

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
  target_pose = start_pose;

  target_pose.position.x += 0.20;

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

  std::vector<geometry_msgs::msg::Pose> pts;
  pts.push_back(start_pose);
  pts.push_back(target_pose);

  unsigned int counter = 0;
  unsigned int maxPath = 500;

  moveit_visual_tools.prompt("Begin plannnig");
  
  do
  {
    ++iter;

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

    Eigen::MatrixXd J_pinv = eig_pinv(jacobian, 0.01, 0.001);
    auto N = Eigen::MatrixXd::Identity(7, 7) - J_pinv * jacobian;

    auto current_tf = robot_state->getGlobalLinkTransform("fr3_hand_tcp");
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

  // ================================================================================
  // auto joints = plan.trajectory_.joint_trajectory;
  // auto points = joints.points;

  // ================================================================================

  // moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model);
  robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), plannerGroup);
  rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);
  publishTcpTrajectory(rt, "fr3_hand_tcp", marker_pub);
  moveit_visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "RCLCPP SHUTDOWN ...");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}