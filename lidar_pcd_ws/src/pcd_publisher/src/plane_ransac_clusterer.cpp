#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
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
        pcl::PointCloud<pcl::PointNormal>::Ptr points;
        pcl::ModelCoefficients::Ptr coefficients;
        Eigen::Vector3f normal;
        Eigen::Vector3f centroid;
        std::string type;  // "horizontal", "vertical", "inclined"
        double area;
        int id;
    };

    struct ObjectCluster
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr points;
        Eigen::Vector3f centroid;
        Eigen::Vector3f min_pt, max_pt;
        std::string type;
        int id;
    };

    PlaneRANSACClusterer() : private_nh_("~"), plane_id_counter_(0), object_id_counter_(0)
    {
        // 订阅房间结构滤波后的点云
        sub_ = nh_.subscribe("/cloud_pcd_room_filtered", 1, &PlaneRANSACClusterer::cloudCallback, this);
        
        // 发布结果
        planes_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detected_planes", 1);
        objects_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("detected_objects", 1);
        plane_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("plane_markers", 1);
        object_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("object_markers", 1);
        
        // 获取缩放参数
        private_nh_.param<double>("point_cloud_scale", point_cloud_scale_, 1000.0);
        
        // RANSAC参数 - 根据缩放调整
        private_nh_.param<double>("plane_distance_thresh", plane_distance_thresh_, 0.02 * point_cloud_scale_);
        private_nh_.param<int>("plane_max_iterations", plane_max_iterations_, 1000);
        private_nh_.param<int>("min_plane_points", min_plane_points_, 200);
        private_nh_.param<int>("max_planes", max_planes_, 5);  // 减少平面数量
        
        // 聚类参数 - 根据缩放调整
        private_nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.05 * point_cloud_scale_);
        private_nh_.param<int>("min_cluster_size", min_cluster_size_, 50);
        private_nh_.param<int>("max_cluster_size", max_cluster_size_, 5000);
        
        // 平面分类参数 - 根据缩放调整
        private_nh_.param<double>("horizontal_angle_thresh", horizontal_angle_thresh_, 15.0);
        private_nh_.param<double>("vertical_angle_thresh", vertical_angle_thresh_, 15.0);
        private_nh_.param<double>("floor_height_thresh", floor_height_thresh_, 0.5 * point_cloud_scale_);
        private_nh_.param<double>("table_height_min", table_height_min_, 0.5 * point_cloud_scale_);
        private_nh_.param<double>("table_height_max", table_height_max_, 1.2 * point_cloud_scale_);
        private_nh_.param<double>("min_plane_area", min_plane_area_, 0.2 * point_cloud_scale_ * point_cloud_scale_);
        
        ROS_INFO("Plane RANSAC Clusterer initialized:");
        ROS_INFO("  Point cloud scale: %.1f", point_cloud_scale_);
        ROS_INFO("  Plane distance threshold: %.3f", plane_distance_thresh_);
        ROS_INFO("  Max iterations: %d", plane_max_iterations_);
        ROS_INFO("  Min plane points: %d", min_plane_points_);
        ROS_INFO("  Max planes: %d", max_planes_);
        ROS_INFO("  Cluster tolerance: %.3f", cluster_tolerance_);
        ROS_INFO("  Floor height threshold: %.1f", floor_height_thresh_);
        ROS_INFO("  Table height range: [%.1f, %.1f]", table_height_min_, table_height_max_);
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

        ROS_INFO("Processing point cloud with %lu points", cloud->points.size());

        // 分析点云的Z坐标分布
        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
        std::vector<float> z_values;
        for (const auto& point : cloud->points) {
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
            z_values.push_back(point.z);
        }
        std::sort(z_values.begin(), z_values.end());
        float median_z = z_values[z_values.size() / 2];
        float q25_z = z_values[z_values.size() / 4];
        float q75_z = z_values[z_values.size() * 3 / 4];
        
        ROS_INFO("Z distribution: min=%.3f, q25=%.3f, median=%.3f, q75=%.3f, max=%.3f", 
                 min_z, q25_z, median_z, q75_z, max_z);
        ROS_INFO("Z range: %.3f, using distance threshold: %.3f", max_z - min_z, plane_distance_thresh_);

        // 重置ID计数器
        plane_id_counter_ = 0;
        object_id_counter_ = 0;
        
        float z_range = max_z - min_z;
        // 自适应调整距离阈值 - 如果Z范围很小，说明数据相对扁平
        double adaptive_distance_thresh = plane_distance_thresh_;
        if (z_range < 100.0) {  // 如果Z范围小于100（原始10cm）
            adaptive_distance_thresh = std::max(0.5, z_range * 0.02);  // 使用Z范围的2%作为阈值
            ROS_INFO("Data appears flat, using adaptive distance threshold: %.3f (original: %.3f)", 
                     adaptive_distance_thresh, plane_distance_thresh_);
        }

        // 步骤1: 检测平面
        std::vector<PlaneInfo> planes;
        pcl::PointCloud<pcl::PointNormal>::Ptr remaining_cloud = detectPlanes(cloud, planes, adaptive_distance_thresh);
        
        // 步骤2: 对剩余点进行聚类
        std::vector<ObjectCluster> objects = clusterRemainingPoints(remaining_cloud);
        
        // 步骤3: 分析和发布结果
        analyzeAndPublish(planes, objects, msg->header);
        
        ROS_INFO("Detected %lu planes and %lu objects", planes.size(), objects.size());
    }

private:
    // 主要的平面检测函数
    pcl::PointCloud<pcl::PointNormal>::Ptr detectPlanes(
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud, 
        std::vector<PlaneInfo>& planes,
        double distance_threshold)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr working_cloud(new pcl::PointCloud<pcl::PointNormal>);
        *working_cloud = *cloud;
        
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
            seg.setDistanceThreshold(distance_threshold);  // 使用自适应阈值
            
            seg.setInputCloud(working_cloud);
            seg.segment(*inliers, *coefficients);
            
            ROS_INFO("RANSAC found %lu inliers with distance threshold %.3f", 
                     inliers->indices.size(), distance_threshold);
            
            if (inliers->indices.size() < min_plane_points_)
            {
                ROS_INFO("No more significant planes found (inliers: %lu)", inliers->indices.size());
                break;
            }
            
            // 提取平面点云
            PlaneInfo plane_info;
            plane_info.points.reset(new pcl::PointCloud<pcl::PointNormal>);
            plane_info.coefficients = coefficients;
            plane_info.id = plane_id_counter_++;
            
            pcl::ExtractIndices<pcl::PointNormal> extract;
            extract.setInputCloud(working_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*plane_info.points);
            
            // 分析平面属性
            analyzePlane(plane_info);
            planes.push_back(plane_info);
            
            // 从工作点云中移除已检测的平面
            pcl::PointCloud<pcl::PointNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointNormal>);
            extract.setNegative(true);
            extract.filter(*temp_cloud);
            working_cloud = temp_cloud;
            
            ROS_INFO("Detected plane %d: %s, %lu points, area: %.2f", 
                     plane_info.id, plane_info.type.c_str(), 
                     plane_info.points->points.size(), plane_info.area);
        }
        
        return working_cloud;
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
        
        // 改进面积计算
        pcl::PointNormal min_pt, max_pt;
        pcl::getMinMax3D(*plane_info.points, min_pt, max_pt);
        
        float dx = max_pt.x - min_pt.x;
        float dy = max_pt.y - min_pt.y;
        float dz = max_pt.z - min_pt.z;
        
        // 根据平面法向量确定主要方向并计算面积
        Eigen::Vector3f abs_normal = plane_info.normal.cwiseAbs();
        
        if (abs_normal.z() > 0.8) {
            // 主要是水平平面 (法向量主要沿Z轴)
            plane_info.area = dx * dy;
        } else if (abs_normal.x() > 0.8) {
            // 主要是YZ平面 (法向量主要沿X轴)
            plane_info.area = dy * dz;
        } else if (abs_normal.y() > 0.8) {
            // 主要是XZ平面 (法向量主要沿Y轴)
            plane_info.area = dx * dz;
        } else {
            // 倾斜平面，使用最大的两个维度
            std::vector<float> dims = {dx, dy, dz};
            std::sort(dims.begin(), dims.end(), std::greater<float>());
            plane_info.area = dims[0] * dims[1];
        }
        
        // 分类平面类型
        classifyPlane(plane_info);
        
        ROS_INFO("Plane %d analysis: normal=(%.2f,%.2f,%.2f), centroid=(%.1f,%.1f,%.1f), area=%.1f, type=%s", 
                 plane_info.id, plane_info.normal[0], plane_info.normal[1], plane_info.normal[2],
                 plane_info.centroid[0], plane_info.centroid[1], plane_info.centroid[2],
                 plane_info.area, plane_info.type.c_str());
    }
    
    // 平面分类
    void classifyPlane(PlaneInfo& plane_info)
    {
        Eigen::Vector3f up_vector(0, 0, 1);      // Z轴向上
        
        // 计算与垂直方向的夹角
        float angle_with_up = std::acos(std::abs(plane_info.normal.dot(up_vector))) * 180.0 / M_PI;
        
        if (angle_with_up < horizontal_angle_thresh_)
        {
            // 水平平面 - 使用缩放调整后的阈值
            if (plane_info.centroid[2] < floor_height_thresh_)
                plane_info.type = "floor_surface";
            else if (plane_info.centroid[2] >= table_height_min_ && plane_info.centroid[2] <= table_height_max_)
                plane_info.type = "table_surface";
            else if (plane_info.centroid[2] > table_height_max_)
                plane_info.type = "high_surface";
            else
                plane_info.type = "low_surface";
        }
        else if (angle_with_up > (90.0 - vertical_angle_thresh_))
        {
            // 垂直平面
            if (plane_info.area > min_plane_area_)
                plane_info.type = "wall_surface";
            else
                plane_info.type = "cabinet_surface";
        }
        else
        {
            // 倾斜平面
            plane_info.type = "inclined_surface";
        }
    }
    
    // 对剩余点进行聚类
    std::vector<ObjectCluster> clusterRemainingPoints(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
    {
        std::vector<ObjectCluster> objects;
        
        if (cloud->points.empty())
        {
            ROS_WARN("No remaining points for clustering");
            return objects;
        }
        
        // 创建KD树
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
        tree->setInputCloud(cloud);
        
        // 欧几里得聚类
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        
        // 处理每个聚类
        for (const auto& indices : cluster_indices)
        {
            ObjectCluster obj;
            obj.points.reset(new pcl::PointCloud<pcl::PointNormal>);
            obj.id = object_id_counter_++;
            
            // 提取聚类点云
            pcl::ExtractIndices<pcl::PointNormal> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(boost::make_shared<pcl::PointIndices>(indices));
            extract.filter(*obj.points);
            
            // 分析物体属性
            analyzeObject(obj);
            objects.push_back(obj);
        }
        
        return objects;
    }
    
    // 分析物体属性
    void analyzeObject(ObjectCluster& obj)
    {
        // 计算质心
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*obj.points, centroid);
        obj.centroid = Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
        
        // 计算边界框
        pcl::PointNormal min_pt, max_pt;
        pcl::getMinMax3D(*obj.points, min_pt, max_pt);
        obj.min_pt = Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z);
        obj.max_pt = Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z);
        
        // 简单分类
        float width = max_pt.x - min_pt.x;
        float depth = max_pt.y - min_pt.y;
        float height = max_pt.z - min_pt.z;
        
        if (height > 1.5 && width < 0.3 && depth < 0.3)
            obj.type = "pole/pillar";
        else if (height < 0.5 && (width > 0.5 || depth > 0.5))
            obj.type = "low_object";
        else if (height > 0.8 && height < 1.2)
            obj.type = "human_height_object";
        else
            obj.type = "unknown_object";
    }
    
    // 分析和发布结果
    void analyzeAndPublish(const std::vector<PlaneInfo>& planes, 
                          const std::vector<ObjectCluster>& objects,
                          const std_msgs::Header& header)
    {
        // 合并所有平面点云
        pcl::PointCloud<pcl::PointNormal> all_planes;
        for (const auto& plane : planes)
        {
            all_planes += *plane.points;
        }
        
        // 合并所有物体点云
        pcl::PointCloud<pcl::PointNormal> all_objects;
        for (const auto& obj : objects)
        {
            all_objects += *obj.points;
        }
        
        // 发布平面点云
        if (!all_planes.points.empty())
        {
            sensor_msgs::PointCloud2 planes_msg;
            pcl::toROSMsg(all_planes, planes_msg);
            planes_msg.header = header;
            planes_pub_.publish(planes_msg);
        }
        
        // 发布物体点云
        if (!all_objects.points.empty())
        {
            sensor_msgs::PointCloud2 objects_msg;
            pcl::toROSMsg(all_objects, objects_msg);
            objects_msg.header = header;
            objects_pub_.publish(objects_msg);
        }
        
        // 发布可视化标记
        publishPlaneMarkers(planes, header);
        publishObjectMarkers(objects, header);
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
            
            // 根据平面类型设置颜色和大小
            marker.scale.x = 0.05; // 薄平面
            marker.scale.y = std::max(0.5, std::sqrt(plane.area));
            marker.scale.z = std::max(0.5, std::sqrt(plane.area));
            
            if (plane.type == "table_surface") {
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // 绿色
            } else if (plane.type == "wall_surface") {
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0; // 蓝色
            } else if (plane.type == "floor_surface") {
                marker.color.r = 0.5; marker.color.g = 0.3; marker.color.b = 0.0; // 棕色
            } else {
                marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; // 黄色
            }
            marker.color.a = 0.6;
            
            markers.markers.push_back(marker);
        }
        
        plane_markers_pub_.publish(markers);
    }
    
    // 发布物体可视化标记
    void publishObjectMarkers(const std::vector<ObjectCluster>& objects, const std_msgs::Header& header)
    {
        visualization_msgs::MarkerArray markers;
        
        for (const auto& obj : objects)
        {
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "objects";
            marker.id = obj.id;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = obj.centroid[0];
            marker.pose.position.y = obj.centroid[1];
            marker.pose.position.z = obj.centroid[2];
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = obj.max_pt[0] - obj.min_pt[0];
            marker.scale.y = obj.max_pt[1] - obj.min_pt[1];
            marker.scale.z = obj.max_pt[2] - obj.min_pt[2];
            
            // 根据物体类型设置颜色
            if (obj.type == "pole/pillar") {
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 1.0; // 紫色
            } else if (obj.type == "human_height_object") {
                marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0; // 橙色
            } else {
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // 红色
            }
            marker.color.a = 0.7;
            
            markers.markers.push_back(marker);
        }
        
        object_markers_pub_.publish(markers);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Publisher planes_pub_;
    ros::Publisher objects_pub_;
    ros::Publisher plane_markers_pub_;
    ros::Publisher object_markers_pub_;
    
    // RANSAC参数
    double plane_distance_thresh_;
    int plane_max_iterations_;
    int min_plane_points_;
    int max_planes_;
    
    // 聚类参数
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    
    // 滤波参数
    bool auto_detect_bounds_;
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double wall_thickness_;
    double floor_clearance_, ceiling_clearance_;
    
    // 缩放相关参数
    double point_cloud_scale_;
    double floor_height_thresh_;
    double table_height_min_;
    double table_height_max_;
    double min_plane_area_;
    double horizontal_angle_thresh_;
    double vertical_angle_thresh_;

    // ID计数器
    int plane_id_counter_;
    int object_id_counter_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_ransac_clusterer");
    PlaneRANSACClusterer node;
    ros::spin();
    return 0;
}