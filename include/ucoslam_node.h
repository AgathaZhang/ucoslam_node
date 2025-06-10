#ifndef UCOSLAM_NODE_H
#define UCOSLAM_NODE_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <string>
#include <sstream>

#include "ucoslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include "basictypes/cvversioning.h"

struct PointXYZRTLT {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint8_t tag;
    uint8_t line;
    double timestamp;
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRTLT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp, timestamp))

class UcoSlamNode {
public:
    UcoSlamNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~UcoSlamNode();

    void processBagFile(const std::string& bag_file);
    void processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void publishPose(const cv::Mat& camPose_c2g, const cv_bridge::CvImageConstPtr& cv_image);
    void publishPointCloud();
    void loadLidarExtrinsics(const std::string& file);
    void publishMergedPointCloud();
    void TransformToEnd(PointXYZRTLT const *const pi, PointXYZRTLT *const po, const double end_time, Eigen::Quaternionf q_last_curr, Eigen::Vector3f t_last_curr);

private:
    cv::Mat resize(cv::Mat& in, cv::Size size);
    Eigen::Matrix4f getTransformFromPose(const geometry_msgs::PoseStamped& pose);
    double fixNum(double num);

    ros::Publisher cam_pose_pub_;
    ros::Publisher pointcloud_pub_;
    ros::Publisher path_pub_;
    ros::Publisher processed_image_pub_;
    ros::Publisher merged_pointcloud_pub_; // Publisher for merged point cloud
    nav_msgs::Path path_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::string bag_file_;
    std::string camera_params_file_;
    std::string image_topic_;
    std::string lidar_topic_;
    std::string params_file_;
    std::string map_file_;
    std::string vocab_file_;
    std::string save_merged_pointcloud_file_;
    std::string save_map_file_;
    bool loc_only_;
    bool undistort_;
    int debug_level_;
    Eigen::Matrix4f last_lidar_to_world; // Last lidar to world transformation

    ucoslam::UcoSlam Slam;
    ucoslam::ImageParams image_params;
    ucoslam::Params params;
    std::shared_ptr<ucoslam::Map> TheMap;
    ucoslam::MapViewer TheViewer;

    std::vector<cv::Mat> undistMap;

    std::map<double, geometry_msgs::PoseStamped> pose_map_; // Map of poses
    pcl::PointCloud<PointXYZRTLT>::Ptr merged_cloud_{new pcl::PointCloud<PointXYZRTLT>()}; // Merged point cloud
    Eigen::Matrix4f lidar_to_camera_transform_;             // Lidar to camera extrinsics
    std::string lidar_params_file_;                         // Path to lidar extrinsics file
};

#endif // UCOSLAM_NODE_H