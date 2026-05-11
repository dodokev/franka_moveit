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
    const robot_trajectory::RobotTrajectory &trajectory,
    const std::string &tcp_link,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world"; // base frame
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
        const moveit::core::RobotState &state = trajectory.getWayPoint(i);

        // From state at a waypoint, get transform of tcp_link from panda_tool0
        const Eigen::Isometry3d &tcp_pose = state.getGlobalLinkTransform(tcp_link);

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

    const std::string plannerGroup = "fr3_arm";

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, plannerGroup);
    move_group.startStateMonitor();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();
    const moveit::core::JointModelGroup *joint_model = robot_state->getJointModelGroup(plannerGroup);

    // Settings
    move_group.setPlanningTime(1.0);
    move_group.setNumPlanningAttempts(10);

    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");

    RCLCPP_INFO(LOGGER, "EE name : %s", move_group.getEndEffector().c_str());
    RCLCPP_INFO(LOGGER, "EE link : %s", move_group.getEndEffectorLink().c_str());
    // move_group.setEndEffector()

    namespace rvt = rviz_visual_tools;
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{node, "fr3_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group.getRobotModel()};
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // ================================================================================
    // ================================================================================

    auto current_pose = move_group.getCurrentPose().pose;
    geometry_msgs::msg::Pose start_pose;
    start_pose = current_pose;

    bool foundIK = robot_state->setFromIK(joint_model, start_pose);
    if (!foundIK)
        RCLCPP_ERROR(LOGGER, "Not joint configuration found");
    else
    {
        move_group.setStartState(*robot_state);
        RCLCPP_INFO(LOGGER, "Start pose done");
    }

    moveit_visual_tools.publishRobotState(*robot_state);
    moveit_visual_tools.trigger();

    // Set a target Pose
    geometry_msgs::msg::Pose target_pose;
    target_pose = start_pose;

    target_pose.position.z = 0.3;

    move_group.setPoseTarget(target_pose);

    // ================================================================================
    // ================================================================================

    bool pathFound = false;
    unsigned int iter = 0;
    do
    {
        ++iter;
        pathFound = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    } while (!pathFound);

    RCLCPP_INFO(LOGGER, "Path found at iteration : %d", iter);

    // moveit_visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model);
    robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), plannerGroup);
    rt.setRobotTrajectoryMsg(*robot_state, plan.trajectory_);
    publishTcpTrajectory(rt, "fr3_hand_tcp", marker_pub);
    moveit_visual_tools.trigger();

    // moveit_visual_tools.prompt("Press next to execute the trajectory");
    // move_group.execute(plan);

    rclcpp::shutdown();
    spinner.join();
    return 0;
}