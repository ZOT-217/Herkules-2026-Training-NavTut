#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");  // 私有节点句柄
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_pcd", 1, true);


    pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
    
    std::string file_path;
    if (!private_nh.getParam("pcd_file_path", file_path))
    {
        file_path = "/home/wy114/NavTut/lidar_pcd_ws/src/pcd_publisher/pcd/RMUC.pcd";
        ROS_WARN("Parameter 'pcd_file_path' not found, using default path: %s", file_path.c_str());
    }
    ROS_INFO("Using PCD file: %s", file_path.c_str());

    float scale;
    if (!private_nh.getParam("scale", scale))
    {
        scale = 1000.0;
        ROS_WARN("Parameter 'scale' not found, using default scale: %f", scale);
    }
    ROS_INFO("Using scale: %f", scale);

    double roll, pitch, yaw;
    private_nh.param("roll", roll, 0.0);   // 绕x轴旋转（弧度）
    private_nh.param("pitch", pitch, 0.0); // 绕y轴旋转（弧度）
    private_nh.param("yaw", yaw, 0.0);     // 绕z轴旋转（弧度）

    std::string frame_id;
    private_nh.param("frame_id", frame_id, std::string("map"));

    ROS_INFO("Using rotations - roll: %f, pitch: %f, yaw: %f (radians)", roll, pitch, yaw);
    ROS_INFO("Using frame_id: %s", frame_id.c_str());

    if (pcl::io::loadPCDFile<pcl::PointNormal>(file_path, *cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s", file_path.c_str());
        return -1;
    }

    ROS_INFO("Loaded PCD file with %lu points", cloud->points.size());
    ROS_INFO("Example point: x=%f, y=%f, z=%f, normal_x=%f",
             cloud->points[0].x, cloud->points[0].y, cloud->points[0].z, cloud->points[0].normal_x);
    
    float min_x = 1e9, max_x = -1e9;
    float min_y = 1e9, max_y = -1e9;
    float min_z = 1e9, max_z = -1e9;

    for (auto &p : cloud->points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
        min_z = std::min(min_z, p.z);
        max_z = std::max(max_z, p.z);
    }

    ROS_INFO("x range: [%f, %f]", min_x, max_x);
    ROS_INFO("y range: [%f, %f]", min_y, max_y);
    ROS_INFO("z range: [%f, %f]", min_z, max_z);

    for (auto &p : cloud->points) {
        p.x *= scale;
        p.y *= scale;
        p.z *= scale;
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    // 按照ZYX顺序应用旋转（先yaw，再pitch，最后roll）
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

    pcl::PointCloud<pcl::PointNormal>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloudWithNormals(*cloud, *transformed_cloud, transform);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*transformed_cloud, output);
    output.header.frame_id = frame_id;

    ros::Rate rate(1);
    while (ros::ok())
    {
        output.header.stamp = ros::Time::now();
        pub.publish(output);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
