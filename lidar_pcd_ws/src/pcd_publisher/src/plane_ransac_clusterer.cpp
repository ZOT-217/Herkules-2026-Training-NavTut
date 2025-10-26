#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <cmath>

class PlaneRANSACClusterer
{
public:
    struct PlaneInfo
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;  // 使用XYZRGB类型支持颜色
        pcl::ModelCoefficients::Ptr coefficients;
        Eigen::Vector3f normal;
        Eigen::Vector3f centroid;
        std::string type;  // "horizontal", "inclined", "vertical"
        double area;
        double angle_with_vertical;  // 与垂直方向的角度
        int id;
        uint32_t color;  // RGB颜色值
    };

    PlaneRANSACClusterer() : private_nh_("~"), plane_id_counter_(0)
    {
        // 订阅房间结构滤波后的点云
        sub_ = nh_.subscribe("/cloud_pcd_room_filtered", 1, &PlaneRANSACClusterer::cloudCallback, this);
        
        // 发布结果
        planes_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detected_planes", 1);
        plane_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("plane_markers", 1);
        
        // 获取缩放参数
        private_nh_.param<double>("point_cloud_scale", point_cloud_scale_, 1000.0);
        
        // RANSAC参数 - 根据实际数据调整
        private_nh_.param<double>("plane_distance_thresh", plane_distance_thresh_, 20.0);  // 减小阈值
        private_nh_.param<int>("plane_max_iterations", plane_max_iterations_, 1000);
        private_nh_.param<int>("min_plane_points", min_plane_points_, 500);  // 增加最小点数
        private_nh_.param<int>("max_planes", max_planes_, 10);
        
        // 平面分类参数 - 角度阈值
        private_nh_.param<double>("horizontal_angle_thresh", horizontal_angle_thresh_, 10.0);  // 0-10度为水平
        private_nh_.param<double>("vertical_angle_thresh", vertical_angle_thresh_, 10.0);    // 80-90度为垂直
        private_nh_.param<double>("min_plane_area", min_plane_area_, 100.0);  // 大幅减小面积阈值
        
        ROS_INFO("Plane RANSAC Clusterer initialized:");
        ROS_INFO("  Point cloud scale: %.1f", point_cloud_scale_);
        ROS_INFO("  Plane distance threshold: %.3f", plane_distance_thresh_);
        ROS_INFO("  Max iterations: %d", plane_max_iterations_);
        ROS_INFO("  Min plane points: %d", min_plane_points_);
        ROS_INFO("  Max planes: %d", max_planes_);
        ROS_INFO("  Horizontal angle threshold: %.1f degrees", horizontal_angle_thresh_);
        ROS_INFO("  Vertical angle threshold: %.1f degrees", vertical_angle_thresh_);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->points.empty())
        {
            ROS_WARN("Received empty point cloud");
            return;
        }

        ROS_INFO("Processing point cloud with %lu points", input_cloud->points.size());

        // 分析点云的Z坐标分布
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
        std::vector<float> z_values;
        for (const auto& point : input_cloud->points) {
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
            z_values.push_back(point.z);
        }
        std::sort(z_values.begin(), z_values.end());
        float median_z = z_values[z_values.size() / 2];
        
        ROS_INFO("Z distribution: min=%.3f, median=%.3f, max=%.3f", min_z, median_z, max_z);

        // 重置ID计数器
        plane_id_counter_ = 0;
        
        float z_range = max_z - min_z;
        // 使用固定的距离阈值，不进行自适应调整
        double adaptive_distance_thresh = plane_distance_thresh_;
        
        ROS_INFO("Using distance threshold: %.3f", adaptive_distance_thresh);

        // 检测平面
        std::vector<PlaneInfo> planes;
        detectPlanes(input_cloud, planes, adaptive_distance_thresh);
        
        // 分析和发布结果
        analyzeAndPublish(planes, msg->header);
        
        ROS_INFO("Detected %lu planes", planes.size());
        for (const auto& plane : planes) {
            ROS_INFO("  Plane %d: %s, angle: %.1f°, points: %lu, area: %.2f", 
                     plane.id, plane.type.c_str(), plane.angle_with_vertical, 
                     plane.points->points.size(), plane.area);
        }
    }

private:
    // 主要的平面检测函数
    void detectPlanes(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud, 
        std::vector<PlaneInfo>& planes,
        double distance_threshold)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr working_cloud(new pcl::PointCloud<pcl::PointNormal>);
        *working_cloud = *cloud;
        
        // 预定义颜色数组
        std::vector<uint32_t> colors = {
            0xFF0000,  // 红色 - 水平面
            0x00FF00,  // 绿色 - 垂直面（墙）
            0x0000FF,  // 蓝色 - 斜面
            0xFFFF00,  // 黄色
            0xFF00FF,  // 紫色
            0x00FFFF,  // 青色
            0xFFA500,  // 橙色
            0x800080,  // 紫色
            0xFFC0CB,  // 粉色
            0x808080   // 灰色
        };
        
        for (int i = 0; i < max_planes_ && working_cloud->points.size() > min_plane_points_; ++i)
        {
            ROS_INFO("Iteration %d: working cloud has %lu points", i, working_cloud->points.size());
            
            // 设置RANSAC分割器
            pcl::SACSegmentation<pcl::PointNormal> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(plane_max_iterations_);
            seg.setDistanceThreshold(distance_threshold);
            
            seg.setInputCloud(working_cloud);
            seg.segment(*inliers, *coefficients);
            
            ROS_INFO("RANSAC found %lu inliers", inliers->indices.size());
            
            if (inliers->indices.size() < min_plane_points_)
            {
                ROS_INFO("No more significant planes found (inliers: %lu)", inliers->indices.size());
                break;
            }
            
            // 创建带颜色的平面点云
            PlaneInfo plane_info;
            plane_info.points.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            plane_info.coefficients = coefficients;
            plane_info.id = plane_id_counter_++;
            plane_info.color = colors[i % colors.size()];
            
            // 提取平面点并设置颜色
            for (const auto& idx : inliers->indices) {
                pcl::PointXYZRGB colored_point;
                colored_point.x = working_cloud->points[idx].x;
                colored_point.y = working_cloud->points[idx].y;
                colored_point.z = working_cloud->points[idx].z;
                
                // 设置颜色
                colored_point.r = (plane_info.color >> 16) & 0xFF;
                colored_point.g = (plane_info.color >> 8) & 0xFF;
                colored_point.b = plane_info.color & 0xFF;
                
                plane_info.points->points.push_back(colored_point);
            }
            plane_info.points->width = plane_info.points->points.size();
            plane_info.points->height = 1;
            plane_info.points->is_dense = true;
            
            // 分析平面属性
            analyzePlane(plane_info);
            
            // 过滤小面积平面
            if (plane_info.area >= min_plane_area_) {
                planes.push_back(plane_info);
                ROS_INFO("Accepted plane %d: %s, angle: %.1f°, area: %.2f", 
                         plane_info.id, plane_info.type.c_str(), 
                         plane_info.angle_with_vertical, plane_info.area);
            } else {
                ROS_INFO("Rejected plane %d: area %.2f < threshold %.2f", 
                         plane_info.id, plane_info.area, min_plane_area_);
            }
            
            // 从工作点云中移除已检测的平面
            pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointNormal>);
            pcl::ExtractIndices<pcl::PointNormal> extract;
            extract.setInputCloud(working_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*temp_cloud);
            working_cloud = temp_cloud;
        }
    }
    
    // 分析平面属性
    void analyzePlane(PlaneInfo& plane_info)
    {
        // 提取平面法向量
        float a = plane_info.coefficients->values[0];
        float b = plane_info.coefficients->values[1];
        float c = plane_info.coefficients->values[2];
        plane_info.normal = Eigen::Vector3f(a, b, c);
        plane_info.normal.normalize();
        
        // 计算平面质心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*plane_info.points, centroid);
        plane_info.centroid = Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
        
        // 计算面积（使用边界框方法）
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(*plane_info.points, min_pt, max_pt);
        
        float dx = max_pt.x - min_pt.x;
        float dy = max_pt.y - min_pt.y;
        float dz = max_pt.z - min_pt.z;
        
        // 根据平面法向量确定主要方向并计算面积
        Eigen::Vector3f abs_normal = plane_info.normal.cwiseAbs();
        
        if (abs_normal.z() > 0.8) {
            plane_info.area = dx * dy;  // 水平平面
        } else if (abs_normal.x() > 0.8) {
            plane_info.area = dy * dz;  // YZ平面
        } else if (abs_normal.y() > 0.8) {
            plane_info.area = dx * dz;  // XZ平面
        } else {
            // 倾斜平面，使用最大的两个维度
            std::vector<float> dims = {dx, dy, dz};
            std::sort(dims.begin(), dims.end(), std::greater<float>());
            plane_info.area = dims[0] * dims[1];
        }
        
        // 计算与垂直方向（Z轴）的夹角
        Eigen::Vector3f z_axis(0, 0, 1);
        float cos_angle = std::abs(plane_info.normal.dot(z_axis));
        cos_angle = std::min(1.0f, std::max(0.0f, cos_angle));  // 限制在[0,1]范围内
        plane_info.angle_with_vertical = std::acos(cos_angle) * 180.0 / M_PI;
        
        // 重新分类平面 - 基于角度
        if (plane_info.angle_with_vertical <= horizontal_angle_thresh_) {
            // 0-10度：水平面
            plane_info.type = "horizontal";
            // 根据类型调整颜色（红色）
            plane_info.color = 0xFF0000;
        } else if (plane_info.angle_with_vertical >= (90.0 - vertical_angle_thresh_)) {
            // 80-90度：垂直面（墙）
            plane_info.type = "vertical";
            // 根据类型调整颜色（绿色）
            plane_info.color = 0x00FF00;
        } else {
            // 10-80度：斜面
            plane_info.type = "inclined";
            // 根据类型调整颜色（蓝色）
            plane_info.color = 0x0000FF;
        }
        
        // 重新设置点云颜色
        for (auto& point : plane_info.points->points) {
            point.r = (plane_info.color >> 16) & 0xFF;
            point.g = (plane_info.color >> 8) & 0xFF;
            point.b = plane_info.color & 0xFF;
        }
        
        ROS_INFO("Plane %d: normal=(%.2f,%.2f,%.2f), angle=%.1f°, centroid=(%.1f,%.1f,%.1f), area=%.1f, type=%s", 
                 plane_info.id, plane_info.normal[0], plane_info.normal[1], plane_info.normal[2],
                 plane_info.angle_with_vertical, plane_info.centroid[0], plane_info.centroid[1], plane_info.centroid[2],
                 plane_info.area, plane_info.type.c_str());
    }
    
    // 分析和发布结果
    void analyzeAndPublish(const std::vector<PlaneInfo>& planes, 
                          const std_msgs::Header& header)
    {
        // 合并所有平面点云（带颜色）
        pcl::PointCloud<pcl::PointXYZRGB> all_planes;
        for (const auto& plane : planes)
        {
            all_planes += *plane.points;
        }
        
        // 发布彩色平面点云
        if (!all_planes.points.empty())
        {
            sensor_msgs::PointCloud2 planes_msg;
            pcl::toROSMsg(all_planes, planes_msg);
            planes_msg.header = header;
            planes_pub_.publish(planes_msg);
        }
        
        // 发布可视化标记
        publishPlaneMarkers(planes, header);
        
        // 输出统计信息
        int horizontal_count = 0, vertical_count = 0, inclined_count = 0;
        for (const auto& plane : planes) {
            if (plane.type == "horizontal") horizontal_count++;
            else if (plane.type == "vertical") vertical_count++;
            else if (plane.type == "inclined") inclined_count++;
        }
        
        ROS_INFO("Plane statistics: %d horizontal, %d vertical, %d inclined", 
                 horizontal_count, vertical_count, inclined_count);
    }
    
    // 发布平面可视化标记
    void publishPlaneMarkers(const std::vector<PlaneInfo>& planes, const std_msgs::Header& header)
    {
        visualization_msgs::MarkerArray markers;
        
        for (const auto& plane : planes)
        {
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "planes";
            marker.id = plane.id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = plane.centroid[0];
            marker.pose.position.y = plane.centroid[1];
            marker.pose.position.z = plane.centroid[2];
            marker.pose.orientation.w = 1.0;
            
            // 根据平面大小设置标记大小
            marker.scale.x = 0.1; // 薄标记
            marker.scale.y = std::max(0.5, std::sqrt(plane.area) * 0.1);
            marker.scale.z = std::max(0.5, std::sqrt(plane.area) * 0.1);
            
            // 根据平面类型设置颜色
            if (plane.type == "horizontal") {
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // 红色
            } else if (plane.type == "vertical") {
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // 绿色
            } else if (plane.type == "inclined") {
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; // 蓝色
            } else {
                marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; // 黄色
            }
            marker.color.a = 0.8;
            
            // 添加文本标记
            visualization_msgs::Marker text_marker;
            text_marker.header = header;
            text_marker.ns = "plane_labels";
            text_marker.id = plane.id;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            
            text_marker.pose.position.x = plane.centroid[0];
            text_marker.pose.position.y = plane.centroid[1];
            text_marker.pose.position.z = plane.centroid[2] + 0.2;
            text_marker.pose.orientation.w = 1.0;
            
            text_marker.scale.z = 0.3;  // 文字大小
            text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; text_marker.color.a = 1.0;
            
            // 设置文本内容
            char text_buffer[100];
            snprintf(text_buffer, sizeof(text_buffer), "%s\n%.1f°\nID:%d", 
                     plane.type.c_str(), plane.angle_with_vertical, plane.id);
            text_marker.text = text_buffer;
            
            markers.markers.push_back(marker);
            markers.markers.push_back(text_marker);
        }
        
        plane_markers_pub_.publish(markers);
    }
    
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Publisher planes_pub_;
    ros::Publisher plane_markers_pub_;
    
    // RANSAC参数
    double plane_distance_thresh_;
    int plane_max_iterations_;
    int min_plane_points_;
    int max_planes_;
    
    // 缩放相关参数
    double point_cloud_scale_;
    double min_plane_area_;
    double horizontal_angle_thresh_;
    double vertical_angle_thresh_;

    // ID计数器
    int plane_id_counter_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_ransac_clusterer");
    PlaneRANSACClusterer node;
    ros::spin();
    return 0;
}