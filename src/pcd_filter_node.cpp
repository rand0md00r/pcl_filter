#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/filters/radius_outlier_removal.h>
#include <pcl_ros/filters/passthrough.h>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_filter_node");

    std::string input_pcd_file;
    ros::NodeHandle nh("~"); // 加上private namespace

    if (!nh.getParam("input_pcd_file", input_pcd_file))
    {
        ROS_ERROR("Failed to get the 'input_pcd_file' parameter.");
        return 1;
    }

    // Voxel filter parameters
    double leaf_size;
    nh.param("leaf_size", leaf_size, 0.05);

    // Radius outlier removal parameters
    double radius_search;
    int min_neighbors;
    nh.param("radius_search", radius_search, 1.0);
    nh.param("min_neighbors", min_neighbors, 1);

    // PassThrough filter parameters
    double pass_through_min;
    double pass_through_max;
    nh.param("pass_through_min", pass_through_min, -1000.0);
    nh.param("pass_through_max", pass_through_max, 1000.0);

    // Load the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *input_cloud) == -1)
    {
        ROS_ERROR("Failed to load PCD file");
        return 1;
    }

    sensor_msgs::PointCloud2 origin_cloud_msg;
    pcl::toROSMsg(*input_cloud, origin_cloud_msg);
    origin_cloud_msg.header.frame_id = "map";

    

    // Voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*filtered_cloud);





    // Radius outlier removal filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier_filter;
    radius_outlier_filter.setInputCloud(filtered_cloud);
    radius_outlier_filter.setRadiusSearch(radius_search);
    radius_outlier_filter.setMinNeighborsInRadius(min_neighbors);
    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    radius_outlier_filter.filter(*radius_filtered_cloud);



    // PassThrough filter in the z-axis
    pcl::PassThrough<pcl::PointXYZ> pass_through_filter;
    pass_through_filter.setInputCloud(radius_filtered_cloud);
    pass_through_filter.setFilterFieldName("z");
    pass_through_filter.setFilterLimits(pass_through_min, pass_through_max);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
    pass_through_filter.filter(*filtered_cloud_z);



    // Publish the filtered point cloud
    ros::Publisher pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud_z", 1);
    ros::Publisher pub_origin_cloud = nh.advertise<sensor_msgs::PointCloud2>("origin_cloud", 1);
    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud_z, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "map"; // Replace "map" with the correct frame ID
    ros::Rate loop_rate(10); // Adjust the publishing rate if needed

    while (ros::ok())
    {
        pub_filtered_cloud.publish(filtered_cloud_msg);
        pub_origin_cloud.publish(origin_cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Get the directory path of the input file
    size_t last_slash = input_pcd_file.find_last_of("/");
    std::string input_dir = input_pcd_file.substr(0, last_slash + 1);

    // Save the filtered point cloud to "filtered.pcd" file in the input directory
    std::string output_pcd_file = input_dir + "filtered.pcd";
    pcl::io::savePCDFileASCII(output_pcd_file, *filtered_cloud);

    return 0;
}
