#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class PCDDownsampler
{
public:
    PCDDownsampler() : private_nh_("~")  // 初始化私有节点句柄
    {
        sub_ = nh_.subscribe("/cloud_pcd", 1, &PCDDownsampler::cloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_pcd_downsampled", 1);
        
        // 从私有参数服务器获取体素大小，如果未设置则使用默认值
        if (!private_nh_.getParam("voxel_size", voxel_size_))
        {
            voxel_size_ = 1.0; // 默认体素大小
            ROS_WARN("Parameter 'voxel_size' not found, using default value: %f", voxel_size_);
        }
        ROS_INFO("Using voxel size: %f", voxel_size_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
        pcl::VoxelGrid<pcl::PointNormal> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        sor.filter(*cloud_filtered);

        ROS_INFO("Original points: %lu, Downsampled points: %lu", cloud->points.size(), cloud_filtered->points.size());

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = msg->header;
        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;  // 私有节点句柄
    ros::Subscriber sub_;
    ros::Publisher pub_;
    float voxel_size_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_downsample");
    PCDDownsampler node;
    ros::spin();
    return 0;
}
