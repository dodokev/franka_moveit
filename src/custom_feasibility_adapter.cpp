#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/planning_scene/planning_scene.h>

class CustomFeasibilityAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  std::string getDescription() const override
  {
    return "Custom Feasibility Adapter";
  }

  void initialize(const rclcpp::Node::SharedPtr &, const std::string &) override
  {}

  bool adaptAndPlan(
      const PlannerFn& planner,
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      planning_interface::MotionPlanResponse& res,
      std::vector<std::size_t>&) const override
  {
    RCLCPP_WARN(rclcpp::get_logger("adapter"), "Adapter is running!");
    // Clone the scene (important: PlanningScene is const)
    planning_scene::PlanningScenePtr scene = planning_scene->diff();

    scene->setStateFeasibilityPredicate(
      [](const moveit::core::RobotState& state, bool)
      {
        RCLCPP_WARN(rclcpp::get_logger("adapter"), "Feasibility called!");
      
        const auto* link = state.getLinkModel("panda_tool");
        const Eigen::Isometry3d& tf = state.getGlobalLinkTransform(link);

        RCLCPP_WARN(rclcpp::get_logger("adapter"), "panda_tool height : %f", tf.translation().z());        

        return tf.translation().z() > 0.025;
        return true;
      });

    // Call the actual planner with modified scene
    return planner(scene, req, res);
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(CustomFeasibilityAdapter, planning_request_adapter::PlanningRequestAdapter)