#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

class OutlierFilter
{
public:
    OutlierFilter() : private_nh_("~")
    {
        // 订阅降采样后的点云
        sub_ = nh_.subscribe("/cloud_pcd_downsampled", 1, &OutlierFilter::cloudCallback, this);
        // 发布滤波后的点云
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_pcd_filtered", 1);
        
        // 从私有参数获取滤波参数
        private_nh_.param<int>("mean_k", mean_k_, 50);
        private_nh_.param<double>("std_dev_thresh", std_dev_thresh_, 1.0);
        
        ROS_INFO("Statistical Outlier Filter initialized:");
        ROS_INFO("  - Mean K neighbors: %d", mean_k_);
        ROS_INFO("  - Std dev threshold: %f", std_dev_thresh_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // 转换ROS消息到PCL格式
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->points.empty())
        {
            ROS_WARN("Received empty point cloud");
            return;
        }

        // 应用统计离群点移除滤波器
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
        pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(mean_k_);                    // 设置邻居数量
        sor.setStddevMulThresh(std_dev_thresh_);  // 设置标准差倍数阈值
        sor.filter(*cloud_filtered);

        ROS_INFO("Outlier filter: %lu -> %lu points (removed %lu outliers)", 
                 cloud->points.size(), 
                 cloud_filtered->points.size(),
                 cloud->points.size() - cloud_filtered->points.size());

        // 转换回ROS消息并发布
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = msg->header;
        pub_.publish(output);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    // 滤波器参数
    int mean_k_;           // 用于计算距离的邻居点数量
    double std_dev_thresh_; // 标准差倍数阈值
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "outlier_filter");
    OutlierFilter node;
    ros::spin();
    return 0;
}