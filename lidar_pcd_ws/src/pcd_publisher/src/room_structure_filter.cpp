#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <limits>

class RoomStructureFilter
{
public:
    RoomStructureFilter() : private_nh_("~")
    {
        // 订阅离群点滤波后的点云
        sub_ = nh_.subscribe("/cloud_pcd_downsampled", 1, &RoomStructureFilter::cloudCallback, this);
        // 发布去除房间结构后的点云
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_pcd_room_filtered", 1);
        
        // 从参数获取滤波方式选择
        private_nh_.param<bool>("auto_detect_bounds", auto_detect_bounds_, true);  // 是否自动检测边界
        
        if (!auto_detect_bounds_)
        {
            // 手动参数化模式
            private_nh_.param<double>("x_min", x_min_, -5.0);
            private_nh_.param<double>("x_max", x_max_, 5.0);
            private_nh_.param<double>("y_min", y_min_, -5.0);
            private_nh_.param<double>("y_max", y_max_, 5.0);
            private_nh_.param<double>("z_min", z_min_, 0.2);
            private_nh_.param<double>("z_max", z_max_, 2.5);
            ROS_INFO("Using manual bounds mode");
        }
        else
        {
            ROS_INFO("Using auto-detect bounds mode");
        }
        
        // 墙壁去除参数
        private_nh_.param<double>("wall_thickness", wall_thickness_, 0.3);
        private_nh_.param<double>("floor_clearance", floor_clearance_, 0.1);    // 地板清除高度
        private_nh_.param<double>("ceiling_clearance", ceiling_clearance_, 0.3); // 天花板清除高度
        
        ROS_INFO("Room Structure Filter initialized:");
        ROS_INFO("  Auto detect bounds: %s", auto_detect_bounds_ ? "true" : "false");
        ROS_INFO("  Wall thickness: %.2f", wall_thickness_);
        ROS_INFO("  Floor clearance: %.2f", floor_clearance_);
        ROS_INFO("  Ceiling clearance: %.2f", ceiling_clearance_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->points.empty())
        {
            ROS_WARN("Received empty point cloud");
            return;
        }

        size_t original_size = cloud->points.size();
        pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud = cloud;

        // 如果启用自动检测，先计算边界
        if (auto_detect_bounds_)
        {
            calculateBounds(cloud);
        }

        // 步骤1: 去除天花板和地板（Z轴过滤）
        filtered_cloud = filterByZ(filtered_cloud);
        
        // 步骤2: 去除外墙（X和Y轴过滤）
        filtered_cloud = filterWalls(filtered_cloud);

        ROS_INFO("Room filter: %lu -> %lu points (removed %lu structure points)", 
                 original_size, 
                 filtered_cloud->points.size(),
                 original_size - filtered_cloud->points.size());

        // 转换并发布
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header = msg->header;
        pub_.publish(output);
    }

private:
    // 自动计算点云边界
    void calculateBounds(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
    {
        if (cloud->points.empty()) return;
        
        // 初始化边界值
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
        
        // 遍历所有点找到边界
        for (const auto& point : cloud->points)
        {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }
        
        // 设置边界值，内缩墙壁厚度
        x_min_ = min_x;
        x_max_ = max_x;
        y_min_ = min_y;
        y_max_ = max_y;
        z_min_ = min_z + floor_clearance_;      // 地板上方一点
        z_max_ = max_z - ceiling_clearance_;    // 天花板下方一点
        
        ROS_INFO("Auto-detected bounds:");
        ROS_INFO("  X: [%.3f, %.3f] -> filter: [%.3f, %.3f]", 
                 min_x, max_x, x_min_ + wall_thickness_, x_max_ - wall_thickness_);
        ROS_INFO("  Y: [%.3f, %.3f] -> filter: [%.3f, %.3f]", 
                 min_y, max_y, y_min_ + wall_thickness_, y_max_ - wall_thickness_);
        ROS_INFO("  Z: [%.3f, %.3f] -> filter: [%.3f, %.3f]", 
                 min_z, max_z, z_min_, z_max_);
    }

    // Z轴过滤：去除地板和天花板
    pcl::PointCloud<pcl::PointNormal>::Ptr filterByZ(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PassThrough<pcl::PointNormal> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min_, z_max_);
        pass.filter(*filtered);
        return filtered;
    }

    // 去除外墙
    pcl::PointCloud<pcl::PointNormal>::Ptr filterWalls(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr filtered = cloud;
        
        // X轴过滤
        pcl::PointCloud<pcl::PointNormal>::Ptr x_filtered(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PassThrough<pcl::PointNormal> pass_x;
        pass_x.setInputCloud(filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min_ + wall_thickness_, x_max_ - wall_thickness_);
        pass_x.filter(*x_filtered);
        
        // Y轴过滤
        pcl::PointCloud<pcl::PointNormal>::Ptr xy_filtered(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PassThrough<pcl::PointNormal> pass_y;
        pass_y.setInputCloud(x_filtered);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min_ + wall_thickness_, y_max_ - wall_thickness_);
        pass_y.filter(*xy_filtered);
        
        return xy_filtered;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    // 滤波参数
    bool auto_detect_bounds_;
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double wall_thickness_;
    double floor_clearance_, ceiling_clearance_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "room_structure_filter");
    RoomStructureFilter node;
    ros::spin();
    return 0;
}