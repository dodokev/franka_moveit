#include "franka_moveit/outlier_filter.hpp"

static auto const LOGGER = rclcpp::get_logger("outlier_filter");

OutlierFilter::OutlierFilter()
: Node("outlier_filter")
{
    // sub_unfilter = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&OutlierFilter::filter_callback, this, std::placeholders::_1));
    sub_unfilter = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_filtered_world", rclcpp::SensorDataQoS(), std::bind(&OutlierFilter::filter_callback, this, std::placeholders::_1));
    pub_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_used", rclcpp::SensorDataQoS());

    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    downsampled_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

    pass_.setFilterFieldName("x");
    pass_.setFilterLimits(-1.0, 1.5); // TUNE

    vg_.setLeafSize(0.02f, 0.02f, 0.02f); // TUNE

    ec_.setClusterTolerance(0.1); // TUNE
    ec_.setMinClusterSize(100); // TUNE

    min_keep_size_ = 100; // TUNE
}

void OutlierFilter::filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // RCLCPP_INFO(LOGGER, "Cloud get ...");
    pcl::fromROSMsg(*cloud_msg, *cloud_);

    pass_.setInputCloud(cloud_);
    pass_.filter(*cropped_);

    vg_.setInputCloud(cropped_);
    vg_.filter(*downsampled_);

    tree_->setInputCloud(downsampled_);

    ec_.setSearchMethod(tree_);
    ec_.setInputCloud(downsampled_);
    ec_.extract(cluster_indices_);
    
    // ================================================================================
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& indices : cluster_indices_)
    {
        if (indices.indices.size() > min_keep_size_)
            for ( int idx : indices.indices )
                filtered->points.push_back(downsampled_->points[idx]);
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;

    filtered->is_dense = true;

    // ================================================================================

    // RCLCPP_INFO(LOGGER, "SIZE : %ld", filtered->points.size());

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = cloud_msg->header;
    
    pub_filtered->publish(output);
    
    cluster_indices_.clear();
    filtered->clear();
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<OutlierFilter>();
    
    RCLCPP_INFO(LOGGER, "Filter Node ON");

    rclcpp::spin(node);

    RCLCPP_INFO(LOGGER, "Stop Filter Node");
    rclcpp::shutdown();
    return 0;
}