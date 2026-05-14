#include "outlier_filter.hpp"

static auto const LOGGER = rclcpp::get_logger("outlier_filter");

OutlierFilter::OutlierFilter()
: Node("outlier_filter")
{
    sub_unfilter = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_filtered_world", rclcpp::SensorDataQoS(), std::bind(&OutlierFilter::filter_callback, this, std::placeholders::_1));
    pub_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_filtered_usable", rclcpp::SensorDataQoS());

    cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cropped_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    downsampled_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

    pass_.setFilterFieldName("z");
    pass_.setFilterLimits(0.0, 2.0); // TUNE

    vg_.setLeafSize(0.1f, 0.1f, 0.1f); // TUNE

    ec_.setClusterTolerance(0.05); // TUNE
    ec_.setMinClusterSize(0); // TUNE
    ec_.setMaxClusterSize(250000);
    ec_.setSearchMethod(tree_);

    min_keep_size_ = 1000; // TUNE
}

void OutlierFilter::filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // RCLCPP_INFO(LOGGER, "Cloud get ...");
    pcl::fromROSMsg(*cloud_msg, *cloud_);

    // pass_.setInputCloud(cloud_);
    // pass_.filter(*cropped_);


    vg_.setInputCloud(cloud_);
    // vg_.setInputCloud(cropped_);
    vg_.filter(*downsampled_);

    ec_.setInputCloud(downsampled_);
    ec_.extract(cluster_indices_);

    // ================================================================================

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& indices : cluster_indices_)
    {
        if ( indices.indices.size() < min_keep_size_ )
            continue;

        for ( int idx : indices.indices )
            filtered->points.push_back(downsampled_->points[idx]);
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;

    filtered->is_dense = true;

    // ================================================================================

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = cloud_msg->header;

    pub_filtered->publish(output);
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