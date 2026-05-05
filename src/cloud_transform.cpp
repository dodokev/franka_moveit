#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class CloudToWorld : public rclcpp::Node
{
private:
    /* data */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string input_topic_;
    std::string output_topic_;

    void callback(const sensor_msgs::msg::PointCloud2);

public:
    CloudToWorld(/* args */);
    ~CloudToWorld() = default;
};

static auto const LOGGER = rclcpp::get_logger("cloud_to_world");

CloudToWorld::CloudToWorld()
  : Node("cloud_to_world"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    // Declare parameters with default values
    this->declare_parameter<std::string>("input_topic", "/cloud_in");
    this->declare_parameter<std::string>("output_topic", "/cloud_out");

    // Get parameter values
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_topic_, rclcpp::SensorDataQoS(), std::bind(&CloudToWorld::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
}

void CloudToWorld::callback(const sensor_msgs::msg::PointCloud2 msg)
{
    RCLCPP_INFO(LOGGER, "Cloud get");
    try
    {
        geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform(
            "world",                      // target
            msg.header.frame_id,         // source
            msg.header.stamp);           // timestamp

        sensor_msgs::msg::PointCloud2 cloud_out;
        tf2::doTransform(msg, cloud_out, transform);

        cloud_out.header.frame_id = "world";
        pub_->publish(cloud_out);
        RCLCPP_INFO(LOGGER, "Publish");
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CloudToWorld>();
    
    RCLCPP_INFO(LOGGER, "Transform Node ON");
    RCLCPP_INFO(LOGGER, "Cloud frame --> World frame");

    rclcpp::spin(node);

    RCLCPP_INFO(LOGGER, "Stop Transform Node");
    rclcpp::shutdown();
    return 0;
}