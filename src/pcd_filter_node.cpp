#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/filters/radius_outlier_removal.h>
#include <pcl_ros/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudFilter
{
public:
    PointCloudFilter(ros::NodeHandle& nh) : nh_(nh)
    {
        nh_.param("input_pcd_file", input_pcd_file_, std::string("input_cloud.pcd"));
        nh_.param("leaf_size", leaf_size_, 0.05);
        nh_.param("radius_search", radius_search_, 1.0);
        nh_.param("min_neighbors", min_neighbors_, 1);
        nh_.param("pass_through_min", pass_through_min_, -1000.0);
        nh_.param("pass_through_max", pass_through_max_, 1000.0);

        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_cloud_z", 1);
        origin_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("origin_cloud", 1);
    }

    void processPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file_, *input_cloud) == -1)
        {
            ROS_ERROR("Failed to load PCD file");
            return;
        }

        int batch_size = 5000; // Set the batch size (adjust this based on your memory and performance constraints)

        // Create a vector to store the filtered point clouds from each batch
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_clouds;

        // Split the input point cloud into batches and process each batch separately
        for (int start_idx = 0; start_idx < input_cloud->size(); start_idx += batch_size)
        {
            int end_idx = std::min(start_idx + batch_size, static_cast<int>(input_cloud->size()));
            pcl::PointCloud<pcl::PointXYZ>::Ptr batch_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            batch_cloud->resize(end_idx - start_idx);

            // Copy the data from the original point cloud to the batch cloud
            for (int i = start_idx; i < end_idx; ++i)
            {
                (*batch_cloud)[i - start_idx] = (*input_cloud)[i];
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_z = voxelFilter(batch_cloud);

            // Add the filtered point cloud from this batch to the vector
            filtered_clouds.push_back(filtered_cloud_z);
        }

        // Publish the filtered point clouds from all batches
        for (const auto& filtered_cloud : filtered_clouds)
        {
            sensor_msgs::PointCloud2 filtered_cloud_msg;
            pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
            filtered_cloud_msg.header.frame_id = "map"; // Replace "map" with the correct frame ID
            filtered_cloud_pub_.publish(filtered_cloud_msg);
        }

        // Also publish the original point cloud (optional)
        sensor_msgs::PointCloud2 origin_cloud_msg;
        pcl::toROSMsg(*input_cloud, origin_cloud_msg);
        origin_cloud_msg.header.frame_id = "map"; // Replace "map" with the correct frame ID
        origin_cloud_pub_.publish(origin_cloud_msg);
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
    {
        // Voxel grid filter
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);
        voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter(*filtered_cloud);

        // Radius outlier removal filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_filter;
        radius_outlier_filter.setInputCloud(filtered_cloud);
        radius_outlier_filter.setRadiusSearch(radius_search_);
        radius_outlier_filter.setMinNeighborsInRadius(min_neighbors_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr radius_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        radius_outlier_filter.filter(*radius_filtered_cloud);

        // PassThrough filter in the z-axis
        pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
        pass_through_filter.setInputCloud(radius_filtered_cloud);
        pass_through_filter.setFilterFieldName("z");
        pass_through_filter.setFilterLimits(pass_through_min_, pass_through_max_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
        pass_through_filter.filter(*filtered_cloud_z);

        return filtered_cloud_z;
    }

    ros::NodeHandle nh_;
    ros::Publisher filtered_cloud_pub_;
    ros::Publisher origin_cloud_pub_;
    double leaf_size_;
    double radius_search_;
    int min_neighbors_;
    double pass_through_min_;
    double pass_through_max_;
    std::string input_pcd_file_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_filter_node");
    ros::NodeHandle nh("~");

    PointCloudFilter filter(nh);
    filter.processPointCloud();

    return 0;
}
