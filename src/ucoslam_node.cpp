#include <ros/ros.h>
#include <std_msgs/String.h>


#include "ucoslam_node.h"
// #define EIGEN_DONT_ALIGN_STATICALLY         // Eigen 不对齐静态分配


// 对时间辍进行处理只保留小数点后两位
double UcoSlamNode::fixNum(double num) {
    double factor = 1e2; // 放大1e2倍
    // num = std::trunc(num * factor) / factor; // 截断小数点后两位
    return std::trunc(num);
    return num;
}

// 加载激光雷达外参
void UcoSlamNode::loadLidarExtrinsics(const std::string& file) {
    // Load lidar to camera extrinsics from file
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("Failed to open lidar extrinsics file: %s", file.c_str());
        return;
    }

    cv::Mat extrinsics;
    fs["extrinsics"] >> extrinsics;

    lidar_to_camera_transform_ = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            lidar_to_camera_transform_(i, j) = extrinsics.at<float>(i, j);
        }
    }

    ROS_INFO("Lidar-to-camera transformation matrix:\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f",
        lidar_to_camera_transform_(0, 0), lidar_to_camera_transform_(0, 1), lidar_to_camera_transform_(0, 2), lidar_to_camera_transform_(0, 3),
        lidar_to_camera_transform_(1, 0), lidar_to_camera_transform_(1, 1), lidar_to_camera_transform_(1, 2), lidar_to_camera_transform_(1, 3),
        lidar_to_camera_transform_(2, 0), lidar_to_camera_transform_(2, 1), lidar_to_camera_transform_(2, 2), lidar_to_camera_transform_(2, 3),
        lidar_to_camera_transform_(3, 0), lidar_to_camera_transform_(3, 1), lidar_to_camera_transform_(3, 2), lidar_to_camera_transform_(3, 3));
}

// 点云去畸变处理
void UcoSlamNode::TransformToEnd(PointXYZRTLT const *const pi,  // 去畸变前
                                    PointXYZRTLT *const po,     // 去畸变后
                                    const double begin_time,    // cloud->back().timestamp 时间升序 小时间在前
                                    Eigen::Quaternionf q_last_curr, // 雷达帧在世界的Q
                                    Eigen::Vector3f t_last_curr){   // 雷达帧在世界的t
    
    // interpolation ratio
    double s;
    double SCAN_PERIOD = 0.1;
    bool DISTORTION = true;
    if (DISTORTION) {
        s = (begin_time / 1e9 - pi->timestamp / 1e9) / SCAN_PERIOD; // 计算时间插值比例
    } else {
        s = 1.0; // 如果不考虑畸变，直接设置 s=1.0
    }

    // 使用 SLERP 对旋转四元数进行插值
    Eigen::Quaternionf q_point_last = Eigen::Quaternionf::Identity().slerp(s, q_last_curr);
    // 对平移向量进行线性插值
    Eigen::Vector3f t_point_last = s * t_last_curr;

    // 将输入点的坐标存储到 Eigen 向量中
    Eigen::Vector3f point(pi->x, pi->y, pi->z);
    // 应用旋转和平移变换，将点从当前坐标系变换到初始坐标系
    Eigen::Vector3f un_point = q_point_last * point + t_point_last;

    // 将变换后的坐标赋值到输出点
    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// 对输入图像进行缩放处理
cv::Mat UcoSlamNode::resize(cv::Mat& in, cv::Size size) {
    if (size.area() <= 0) return in;
    cv::Mat ret;
    cv::resize(in, ret, size);
    return ret;
}

// 发布相机位姿
void UcoSlamNode::publishPose(const cv::Mat& camPose_c2g, const cv_bridge::CvImageConstPtr& cv_image) {
    try {
        // cv::Mat camPose_c2g = camPose_c2g_or;
        // 检查矩阵是否为空
        if (camPose_c2g.empty()) {
            ROS_WARN("SLAM cannot determine the camera pose for the current frame");
            // TODO 保留 R|t = 1|1
        // camPose_c2g = (cv::Mat_<float>(4,4) << 
        //                             1, 0, 0, 0,
        //                             0, 1, 0, 0,
        //                             0, 0, 1, 0,
        //                             0, 0, 0, 1);
            return;
        }

        // 创建并发布相机位姿消息
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = cv_image->header.stamp;
        pose_msg.header.frame_id = "map";

        // 提取旋转矩阵
        cv::Mat rotation = camPose_c2g(cv::Rect(0, 0, 3, 3)); // 左上角3x3子矩阵
        std::cout << "rotation: " << rotation << std::endl;
        // 将cv::Mat转换为Eigen::Matrix3d
        Eigen::Matrix3d R_WtoC;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_WtoC(i, j) = rotation.at<float>(i, j);
            }
        }

        Eigen::Vector3d T_WinC(camPose_c2g.at<float>(0, 3), camPose_c2g.at<float>(1, 3), camPose_c2g.at<float>(2, 3));

        Eigen::Matrix3d R_CtoW = R_WtoC.transpose();    // 旋转矩阵转置 R_CtoW 表示世界相对于当前相机帧的旋转 
        Eigen::Vector3d T_CinW = -R_CtoW * T_WinC;      // 世界相对于当前相机帧的平移 = 世界相对于当前相机的旋转 * -相机相对于世界的平移

        Eigen::Quaterniond quaternion(R_CtoW);

        pose_msg.pose.position.x = T_CinW(0);
        pose_msg.pose.position.y = T_CinW(1);
        pose_msg.pose.position.z = T_CinW(2);

        pose_msg.pose.orientation.x = quaternion.x();
        pose_msg.pose.orientation.y = quaternion.y();
        pose_msg.pose.orientation.z = quaternion.z();
        pose_msg.pose.orientation.w = quaternion.w();

        pose_msg_usefor_txt = pose_msg;         // 给到类成员中转给write txt
        cam_pose_pub_.publish(pose_msg);        // 世界相对于当前帧相机的位姿

        // 将位姿添加到路径并发布路径
        path_.header.stamp = pose_msg.header.stamp;
        path_.header.frame_id = "map";      // 初始化路径消息的坐标系为 "map"，与发布的 pose 保持一致，确保 RViz 中坐标系统一
        path_.poses.push_back(pose_msg);    // TODO 这里带宽限制要加滑动窗口 定期写文件 或者改用 Odometry
        path_pub_.publish(path_);

        pose_map_[fixNum(cv_image->header.stamp.toSec())] = pose_msg;
/** 写入txt*/
        // camera_full_trajectory(pose_msg_usefor_txt);        // 利用pose_msg_usefor_txt中转 放到集合块

    } catch (const cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error: %s", e.what());
    }
}

// 将ROS PoseStamped转换为Eigen::Matrix4f
Eigen::Matrix4f UcoSlamNode::getTransformFromPose(const geometry_msgs::PoseStamped& pose) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    Eigen::Quaternionf q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    Eigen::Matrix3f rotation = q.toRotationMatrix();

    transform.block<3, 3>(0, 0) = rotation;
    transform(0, 3) = pose.pose.position.x;
    transform(1, 3) = pose.pose.position.y;
    transform(2, 3) = pose.pose.position.z;

    return transform;
}

// 处理点云数据
void UcoSlamNode::processPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Find corresponding pose by timestamp // TODO 相机到雷达帧的位姿 外参变换
    auto it = pose_map_.find(fixNum(cloud_msg->header.stamp.toSec()));
    if (it == pose_map_.end()) {
        ROS_WARN("No matching pose found for point cloud at time: %f", cloud_msg->header.stamp.toSec());
        return;
    }

    ROS_INFO("matching pose found for point cloud at time: %f", cloud_msg->header.stamp.toSec());

    const geometry_msgs::PoseStamped& pose = it->second;

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<PointXYZRTLT>::Ptr cloud(new pcl::PointCloud<PointXYZRTLT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Transform point cloud using camera-to-lidar extrinsics and pose
    Eigen::Matrix4f transform = getTransformFromPose(pose) * lidar_to_camera_transform_;    // 世界位姿 * 雷达外参
    Eigen::Matrix4f last_to_cur;

    static bool is_first = true;

    if (is_first) {
        last_to_cur = Eigen::Matrix4f::Identity();
        is_first = false;
    } else {
        last_to_cur =  transform.inverse() * last_lidar_to_world;
    }

    Eigen::Quaternionf quaternion(last_to_cur.topLeftCorner<3, 3>());   // 转成Eigen Q|t
    Eigen::Vector3f translation = last_to_cur.topRightCorner<3, 1>();

    pcl::PointCloud<PointXYZRTLT>::Ptr undispoint_cloud(new pcl::PointCloud<PointXYZRTLT>);     // 定义去畸变点云
    undispoint_cloud->reserve(cloud->points.size());
    std::sort(cloud->begin(), cloud->end(), [](const PointXYZRTLT & a, const PointXYZRTLT & b)
    {
        return a.timestamp < b.timestamp;
    });

    // 点云去畸变
    for (const auto& point : cloud->points) {
        PointXYZRTLT undis_point;
        TransformToEnd(&point, &undis_point, cloud->back().timestamp, quaternion, translation);     
        undispoint_cloud->points.push_back(undis_point);
    }

    pcl::PointCloud<PointXYZRTLT>::Ptr transformed_cloud(new pcl::PointCloud<PointXYZRTLT>());
    pcl::transformPointCloud(*undispoint_cloud, *transformed_cloud, transform);

    // Add transformed cloud to merged cloud
    *merged_cloud_ += *transformed_cloud;
    last_lidar_to_world = transform;

    // Publish merged point cloud
    publishMergedPointCloud();
}

// 发布合并后的点云
void UcoSlamNode::publishMergedPointCloud() {
    // Convert merged cloud to ROS message
    sensor_msgs::PointCloud2 merged_msg;
    pcl::toROSMsg(*merged_cloud_, merged_msg);
    merged_msg.header.frame_id = "map";
    merged_msg.header.stamp = ros::Time::now();

    // Publish merged point cloud
    merged_pointcloud_pub_.publish(merged_msg);
}

// std_msgs/Header header         // 时间戳与参考坐标系
// uint32_t height                // 点云高度（行数）
// uint32_t width                 // 点云宽度（列数），height × width = 点数
// bool is_bigendian             // 数据是否为大端字节序（一般为 false）
// uint32_t point_step           // 单个点的字节数
// uint32_t row_step             // 一行数据的总字节数
// std::vector<sensor_msgs::PointField> fields  // 点字段定义，比如 x, y, z, rgb
// bool is_dense                 // 是否为密集点云（true：无非法点）
// std::vector<uint8_t> data     // 点数据字节流（按照 fields 描述的格式组织）

// 视觉特征点点云
void UcoSlamNode::publishPointCloud() {
    if (!TheMap) {
        ROS_WARN("Map is not initialized.");
        return;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "map";

    // 填充点云数据
    std::vector<sensor_msgs::PointField> fields(3);
    fields[0].name = "x";
    fields[0].offset = 0;       // 指定 "x" 坐标在每个点数据结构中的字节偏移量，这里从第 0 个字节开始
    fields[0].datatype = sensor_msgs::PointField::FLOAT32;// 设置 "x" 坐标的数据类型，这里使用 32 位浮点型（对应于 float）
    fields[0].count = 1;        // 设置 "x" 坐标在每个点里只占 1 个值（不是数组）

    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    fields[1].count = 1;

    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    fields[2].count = 1;

    cloud_msg.fields = fields;
    cloud_msg.point_step = 12;          // 每个点占用的字节数 (3 * 4 bytes)
    cloud_msg.is_bigendian = false;     // 数据是否为大端字节序
    cloud_msg.is_dense = true;          // 点字段定义，比如 x, y, z, rgb

    // 遍历 TheMap 中的点云
    const auto& map_points = TheMap->map_points;
    cloud_msg.width = map_points.size();
    cloud_msg.height = 1;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;    // 一行数据的总字节数
    cloud_msg.data.resize(cloud_msg.row_step);                      // 点数据字节流（按照 fields 描述的格式组织）

    uint8_t* ptr = cloud_msg.data.data();
    for (const auto& map_point : map_points) {
        if (map_point.isBad()) continue; // 跳过无效点
        const auto& pos = map_point.getCoordinates();
        float* data_ptr = reinterpret_cast<float*>(ptr);
        data_ptr[0] = pos.x;
        data_ptr[1] = pos.y;
        data_ptr[2] = pos.z;
        ptr += cloud_msg.point_step;
    }

    // 发布点云
    pointcloud_pub_.publish(cloud_msg);
}

void UcoSlamNode::publishMarkerArray() {
    if (!TheMap) return;

    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& m_pair : TheMap->map_markers) {        // for-range走的是基类的迭代器
        const auto& marker = m_pair.second;                 // ucoslam::marker类型
        if (!marker.pose_g2m.isValid()) continue;

        visualization_msgs::Marker marker_msg;
        marker_msg.header.stamp = ros::Time::now();
        marker_msg.header.frame_id = "map";
        marker_msg.ns = "aruco_markers";
        marker_msg.id = id++;
        marker_msg.type = visualization_msgs::Marker::CUBE;
        marker_msg.action = visualization_msgs::Marker::ADD;    // ADD新增或更新这个Marker | DELETE删除这个Marker | DELETEALL删除同一命名空间下所有 Marker（要单独发一个 MarkerArray）
        marker_msg.scale.x = 0.2;
        marker_msg.scale.y = 0.2;
        marker_msg.scale.z = 0.01;
        marker_msg.color.a = 1.0;
        // marker_msg.color.r = 1.0;
        // marker_msg.color.g = 0.6;
        // marker_msg.color.b = 0.3;
        marker_msg.color.r = 0.0;
        marker_msg.color.g = 0.0;
        marker_msg.color.b = 1.0;

        // 从pose_g2m提取位姿信息
        // cv::Mat Rt = marker.pose_g2m.inv(); // 因为 pose_g2m 是从全局到marker，rviz中要画marker相对于全局的位置
        cv::Mat Rt = marker.pose_g2m; // 因为 pose_g2m 是从全局到marker，rviz中要画marker相对于全局的位置
        cv::Mat R = Rt(cv::Range(0, 3), cv::Range(0, 3));
        cv::Mat t = Rt(cv::Range(0, 3), cv::Range(3, 4));

        // 转换为 geometry_msgs::Pose
        tf::Matrix3x3 tf3d(     // 把 OpenCV 的 3x3 旋转矩阵 R 转成 tf::Matrix3x3（Rviz 的旋转需要四元数）
            R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2),
            R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2),
            R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2)
        );
        tf::Quaternion q;       // 创建一个 tf::Quaternion，用于表示姿态的旋转
        tf3d.getRotation(q);    // 从 tf3d 中提取四元数表示（R → q）

        marker_msg.pose.position.x = t.at<float>(0);
        marker_msg.pose.position.y = t.at<float>(1);
        marker_msg.pose.position.z = t.at<float>(2);
        marker_msg.pose.orientation.x = q.x();
        marker_msg.pose.orientation.y = q.y();
        marker_msg.pose.orientation.z = q.z();
        marker_msg.pose.orientation.w = q.w();

        marker_array.markers.push_back(marker_msg);
    }

    marker_array_pub_.publish(marker_array);
}

bool UcoSlamNode::clearOutputFiles() {
    // 定义要删除的路径
    std::vector<std::string> files = {
        full_trajectory_total_path,
        full_trajectory_path,
        camera_trajectory_path,
        marker_path
    };

    bool all_ok = true;

    for (const auto& file : files) {
        if (std::remove(file.c_str()) == 0) {
            ROS_INFO("Deleted file: %s", file.c_str());
        } else {
            ROS_WARN("Failed to delete (may not exist): %s", file.c_str());
            all_ok = false;  // 只要有失败就标记
        }
    }

    return all_ok; // 全部成功 true，否则 false
}

bool UcoSlamNode::camera_full_trajectory(geometry_msgs::PoseStamped pose_msg){
// 格式: camera_full_trajectory.txt 图像时间 雷达时间 t^3 Q^4

// 3. 如果要写格式化内容，也可以直接写
// 在函数内部定义一个 Lambda，用于写一行位姿信息
auto write_pose_line = [](std::ofstream& ofs, const geometry_msgs::PoseStamped& pose) {
    ofs << std::fixed << std::setprecision(16)
        << pose.header.stamp.toSec() << " "    // 图像时间
        << std::setprecision(16) << " "
        << pose.header.stamp.toSec() << " "    // 示例：也可以放雷达时间
        << pose.pose.position.x << " "
        << pose.pose.position.y << " "
        << pose.pose.position.z << " "
        << pose.pose.orientation.x << " "
        << pose.pose.orientation.y << " "
        << pose.pose.orientation.z << " "
        << pose.pose.orientation.w << std::endl;
};


// 1. 使用 stat 检查文件是否存在
    struct stat buffer;
    if (stat(camera_trajectory_path.c_str(), &buffer) == 0) {
        // 文件存在 → 追加写
        std::ofstream outfile(camera_trajectory_path, std::ios::app);
        if (!outfile) {
            ROS_ERROR("Failed to open existing file for appending: %s", camera_trajectory_path.c_str());
            return false;
        }
        write_pose_line(outfile, pose_msg);
        outfile.close();
        return true;
    }

    // 2. 文件不存在，创建新文件
    std::ofstream outfile(camera_trajectory_path);
    if (!outfile) {
        ROS_ERROR("Failed to create file: %s", camera_trajectory_path.c_str());
        return false; // 创建失败
    }

    ROS_INFO("Created new file: %s", camera_trajectory_path.c_str());
    write_pose_line(outfile, pose_msg);
    outfile.close();

    return true;
}

bool UcoSlamNode::full_trajectory_total_key(bool is_KF, geometry_msgs::PoseStamped pose_msg){
// full_trajectory_total_key.txt    关键帧 ros时间 t^3 Q^4
// TODO 靠外参矩阵转回去

auto write_full_trajectory_total_key = [](std::ofstream& ofs, bool flag, const geometry_msgs::PoseStamped& pose) {
    ofs << std::fixed << std::setprecision(16)
        << (flag ? "1" : "0") << " "                 // 第1项：bool值（true=1, false=0）
        << pose.header.stamp.toSec() << " "      // TODO 这里需要再仔细检查一下格式 第2项：ROS时间  
        << pose.pose.position.x << " "           // 第3项：t.x
        << pose.pose.position.y << " "           // 第4项：t.y
        << pose.pose.position.z << " "           // 第5项：t.z
        << pose.pose.orientation.x << " "        // 第6项：q.x
        << pose.pose.orientation.y << " "        // 第7项：q.y
        << pose.pose.orientation.z << " "        // 第8项：q.z
        << pose.pose.orientation.w << std::endl; // 第9项：q.w
};

// 1. 使用 stat 检查文件是否存在
    struct stat buffer;
    if (stat(full_trajectory_total_path.c_str(), &buffer) == 0) {
        // 文件存在 → 追加写
        std::ofstream outfile(full_trajectory_total_path, std::ios::app);
        if (!outfile) {
            ROS_ERROR("Failed to open existing file for appending: %s", full_trajectory_total_path.c_str());
            return false;
        }
        write_full_trajectory_total_key(outfile, is_KF, pose_msg);
        outfile.close();
        return true;
    }

    // 2. 文件不存在，创建新文件
    std::ofstream outfile(full_trajectory_total_path);
    if (!outfile) {
        ROS_ERROR("Failed to create file: %s", full_trajectory_total_path.c_str());
        return false; // 创建失败
    }

    ROS_INFO("Created new file: %s", full_trajectory_total_path.c_str());
    write_full_trajectory_total_key(outfile, is_KF, pose_msg);
    outfile.close();

    return true;

    
}

bool UcoSlamNode::full_trajectory(geometry_msgs::PoseStamped pose_msg){
// full_trajectory.txt  关键帧 ros时间 t^3 Q^4
// TODO 靠外参矩阵转回去

auto write_full_trajectory = [](std::ofstream& ofs, const geometry_msgs::PoseStamped& pose) {
    ofs << std::fixed << std::setprecision(16)
        << "1" << " "                 // 第1项：bool值（true=1, false=0）
        << pose.header.stamp.toSec() << " "      // 第2项：ROS时间
        << pose.pose.position.x << " "           // 第3项：t.x
        << pose.pose.position.y << " "           // 第4项：t.y
        << pose.pose.position.z << " "           // 第5项：t.z
        << pose.pose.orientation.x << " "        // 第6项：q.x
        << pose.pose.orientation.y << " "        // 第7项：q.y
        << pose.pose.orientation.z << " "        // 第8项：q.z
        << pose.pose.orientation.w << std::endl; // 第9项：q.w
};

// 1. 使用 stat 检查文件是否存在
    struct stat buffer;
    if (stat(full_trajectory_path.c_str(), &buffer) == 0) {
        // 文件存在 → 追加写
        std::ofstream outfile(full_trajectory_path, std::ios::app);
        if (!outfile) {
            ROS_ERROR("Failed to open existing file for appending: %s", full_trajectory_path.c_str());
            return false;
        }
        write_full_trajectory(outfile, pose_msg);
        outfile.close();
        return true;
    }

    // 2. 文件不存在，创建新文件
    std::ofstream outfile(full_trajectory_path);
    if (!outfile) {
        ROS_ERROR("Failed to create file: %s", full_trajectory_path.c_str());
        return false; // 创建失败
    }

    ROS_INFO("Created new file: %s", full_trajectory_path.c_str());
    write_full_trajectory(outfile, pose_msg);
    outfile.close();

    return true;

}
/** 输出pcd文件*/
// void write_camera_and_lidar_trajectory(std::string filename){
//     std::fstream outfile;
//     outfile.open(filename.c_str(), std::istream::out);

//     if (!outfile) {
//         std::cout << "Error opening the file: " << filename<<std::endl;
//         return;
//     }

//     if (cloudKeyPoses3D->points.empty())
//     {
//         std::cout<<"没有任何关键帧，建图结束"<<std::endl;
//         return;
//     }

//     boost::filesystem::path meta_folder = "/home/kilox/wutong/data";
//     if (boost::filesystem::exists(meta_folder)) {
//         boost::filesystem::remove_all(meta_folder);
//     }
//     boost::filesystem::create_directory(meta_folder);
//     if (true)
//     {

//         int numPoses = isamCurrentEstimate.size();
//         for (int i = 0; i < numPoses; ++i)
//         {
//             cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
//             cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
//             cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

//             cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
//             cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
//             cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
//             cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
//             cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
//             cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
            
//             Eigen::Quaterniond q_lidar = EulerToQuat(cloudKeyPoses6D->points[i].roll, cloudKeyPoses6D->points[i].pitch, cloudKeyPoses6D->points[i].yaw);
//             outfile<<"1"<<" " <<std::setprecision (16)<< cloudKeyPoses6D->points[i].time << " " << cloudKeyPoses3D->points[i].x << " "<< cloudKeyPoses3D->points[i].y << " "<<cloudKeyPoses3D->points[i].z << " "
//             << q_lidar.x() << " " << q_lidar.y() << " "
//             << q_lidar.z() << " " << q_lidar.w() << '\n';
//             // updatePath(cloudKeyPoses6D->points[i]);
//             std::stringstream path_pcd;
//             path_pcd <<"/home/kilox/wutong/data/"<< std::setprecision (16)  << cloudKeyPoses6D->points[i].time<<".pcd";
            
//             if(surfCloudKeyFrames[i]->points.size()>0){
//                 pcl::io::savePCDFileBinary(path_pcd.str(), *transformPointCloud_loopdetect(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]));
//             }//TODO SAVE
//             // surfCloudKeyFrames[i]
            
//         }
//         ROS_INFO("====== finish save trajectory file =======");
//     }
// }

/** Old deal marker*/
// bool UcoSlamNode::export_marker_increment_txt(const cv_bridge::CvImageConstPtr& cv_image){

    // geometry_msgs::PoseStamped pose_msg;
    // pose_msg.header.stamp = cv_image->header.stamp;

    // std::stringstream write_loop_bag_stream;
    // write_loop_bag_stream << std::setprecision (16)  << image_time_stamp <<" " <<std::setprecision (16)<< current_lidar_time << " " << camera2lidar.t_(0) << " "<<camera2lidar.t_(1) << " "<<camera2lidar.t_(2) << " "
    //                         << camera2lidar.GetQ().x() << " " << camera2lidar.GetQ().y() << " "
    //                         << camera2lidar.GetQ().z() << " " << camera2lidar.GetQ().w() << '\n';

    // full_camera_trajectory_str += write_loop_bag_stream.str();
    // full_camera_trajectory_empty = false;

// }
/** refrence deal marker*/
// void UcoSlamNode::write_aruco_marker(){

//     std::cout<<"Begin to write aruco marker:" << marker_path <<std::endl;
//     std::string full_camera_trajectory_str = "";
//     std::fstream outfile;
//     outfile.open("/home/kilox/wutong/loop_detect/aruco_camera_trajectory.txt", std::istream::out);
//     for(auto item_aruco:arucoTags_map){
//         int markerID=item_aruco.first;
//         ArucoTag* aruco = item_aruco.second;
//         EigenPose marker_to_world(aruco->R_w_m,aruco->t_w_m);
//         EigenPose marker_to_camera(aruco->R_c_m,aruco->t_c_m);

//         std::stringstream write_loop_bag_stream;
//         //0是左目
//         write_loop_bag_stream<<aruco->id <<" " << std::setprecision (16)  << aruco->time_stamp <<" " <<std::setprecision (16)<< aruco->lidar_time_stamp << " " << marker_to_camera.t_(0) << " "<<marker_to_camera.t_(1) << " "<<marker_to_camera.t_(2) << " "
//                               << marker_to_camera.GetQ().x() << " " << marker_to_camera.GetQ().y() << " "
//                               << marker_to_camera.GetQ().z() << " " << marker_to_camera.GetQ().w() << '\n';
//         full_camera_trajectory_str += write_loop_bag_stream.str();
//     }
//     outfile << full_camera_trajectory_str;
//     outfile.flush();
//     outfile.close();
//     std::cout<<"end to write aruco marker"<<std::endl;
// }


// constructure function
UcoSlamNode::UcoSlamNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : 
    nh_(nh), private_nh_(private_nh),rviz_img_(nh){
    
    clearOutputFiles();
    std::string verify; 
    private_nh_.param("param_verify", verify, std::string("not_set"));
    ROS_INFO_STREAM("[DEBUG] Parameter param_verify = " << verify);  

    // 从ROS参数服务器获取参数
    // private_nh_.param("full_trajectory_total_path", full_trajectory_total_path, output_path); 
    // private_nh_.param("full_trajectory_path", full_trajectory_path, output_path);
    // private_nh_.param("camera_trajectory_path", camera_trajectory_path, output_path);
    // private_nh_.param("marker_path", marker_path, output_path);

    private_nh_.param("save_merged_pointcloud_file", save_merged_pointcloud_file_, std::string(""));    // 保存合并后的点云文件路径
    private_nh_.param("save_map_file", save_map_file_, std::string(""));                                // 最终视觉地图的保存路径
    private_nh_.param("bag_file", bag_file_, std::string(""));                                          // 需要处理的 ROS bag 文件路径
    private_nh_.param("camera_params_file", camera_params_file_, std::string(""));                      // 相机内参配置文件路径
    private_nh_.param("image_topic", image_topic_, std::string("/usb_cam_left/image_raw"));             // 图像话题名称，默认为 /usb_cam_left/image_raw
    private_nh_.param("lidar_topic", lidar_topic_, std::string("/lidar/point"));                        // 激光雷达点云话题名称，默认为 /lidar/point
    private_nh_.param("params_file", params_file_, std::string(""));                // UCOSLAM参数配置文件路径（例如 tracking threshold、BA 迭代次数等）
    private_nh_.param("map_file", map_file_, std::string(""));                                          // 已有地图文件路径（用于加载历史地图进行定位）
    private_nh_.param("vocab_file", vocab_file_, std::string(""));                                      // 词典文件路径（用于关键点重定位 BoW 词典用于回环）
    private_nh_.param("loc_only", loc_only_, false);    // 是否开启纯定位模式（跳过建图流程，仅进行跟踪）
    private_nh_.param("undistort", undistort_, false);  // 是否进行图像去畸变处理
    private_nh_.param("debug_level", debug_level_, 0);  // 调试级别（0: 无调试信息，1: 基本调试信息，2: 详细调试信息）

    private_nh_.param("lidar_params_file", lidar_params_file_, std::string("lidar_params_file.yml"));// 激光雷达外参文件路径

    loadLidarExtrinsics(lidar_params_file_);        // 加载激光雷达到相机的外参矩阵（从指定的 YAML 文件读取 4×4 变换矩阵）

    // 定义ROS发布者
    rviz_img_pub_ = rviz_img_.advertise("/ucoslam/image_debug", 10);           // 06.27 创建一个 publisher，发布话题“/ucoslam/image_debug”

    pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);    // 视觉地图中的特征点点云（用于可视化）
    cam_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("camera_pose", 10);   // 相机当前帧在世界坐标系下的位姿（PoseStamped 消息）位姿发布者
    path_pub_ = nh_.advertise<nav_msgs::Path>("camera_path", 10);                   // 相机路径（Path 消息）发布者
    processed_image_pub_ = nh_.advertise<sensor_msgs::Image>("processed_image", 10);// 处理后的图像（如已矫正图像，用于 debug 或可视化）
    merged_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("merged_point_cloud", 10);// 融合后的激光雷达点云（多帧累积之后的全局点云）合并后的点云（用于可视化）
    
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 10);    // 发布marker可视化
    // 初始化路径消息
    
}

UcoSlamNode::~UcoSlamNode() {}


void UcoSlamNode::imshow_in_Rviz(const cv::Mat& img){

    if (!img.empty()) {
        // 转换为 ROS 图像消息（BGR8格式）
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = ros::Time::now(); // 时间戳可以用于 RViz 显示同步

        // 发布消息
        rviz_img_pub_.publish(msg);
    }
}

// 处理bag文件
void UcoSlamNode::processBagFile() {
    try {
        // 初始化UCOSLAM
        Slam.setDebugLevel(debug_level_);
        Slam.showTimers(true);

        image_params.readFromXMLFile(camera_params_file_);      // 读取相机内参配置文件

        if (!params_file_.empty()) {
            params.readFromYMLFile(params_file_);               // 读取UCOSLAM参数配置文件
        }

        TheMap = std::make_shared<ucoslam::Map>();              // 创建地图对象,对象位于node空间中
        if (!map_file_.empty()) {
            TheMap->readFromFile(map_file_);                    // 如果指定了地图文件，则加载已有地图     
        }

        Slam.setParams(TheMap, params, vocab_file_);            // TODO 06.12 Finish 06.16 实际上 call map_initializer 设置UCOSLAM参数，包括地图、参数和词典文件

        if (vocab_file_.empty() && map_file_.empty()) {
            ROS_WARN("Warning!! No VOCABULARY INDICATED. KEYPOINT RELOCALIZATION IMPOSSIBLE WITHOUT A VOCABULARY FILE!!!!!");
        }

        if (loc_only_) {
            Slam.setMode(ucoslam::MODE_LOCALIZATION);           // 如果开启了纯定位模式，则设置UCOSLAM为定位模式
        }

        // 初始化图像处理
        cv::Mat in_image, auxImage;
        size_t prev_keyframe_number;    // 当前关键帧数量
        bool whether_KF;          // 本轮是否为关键帧
        if (undistort_) {
            if (undistMap.empty()) {
                undistMap.resize(2);
                cv::fisheye::initUndistortRectifyMap(
                    image_params.CameraMatrix,
                    image_params.Distorsion,
                    cv::Mat::eye(3, 3, CV_64F),
                    image_params.CameraMatrix,
                    image_params.CamSize,
                    CV_32FC1,
                    undistMap[0],
                    undistMap[1]
                );
            }
            image_params.Distorsion.setTo(cv::Scalar::all(0));
        }   // 如果需要去畸变处理，则初始化去畸变映射

        // 打开bag文件
        rosbag::Bag bag;
        bag.open(bag_file_, rosbag::bagmode::Read);

        // 查询图像话题
        std::vector<std::string> topics = {image_topic_, lidar_topic_};
        rosbag::View view(bag, rosbag::TopicQuery(topics));     // 表示只关心 topics 中列出的两个话题（图像与点云）忽略 .bag 中的其他内容

        // 处理bag文件中的图像消息
        int frameIndex = 0;
        for (const rosbag::MessageInstance &msg : view) {
            if (msg.getTopic() == image_topic_) {       // TODO msg.getTopic() == image_topic_这个行为是做了挑选和舍弃吗
                try {
                    sensor_msgs::ImageConstPtr image_msg = msg.instantiate<sensor_msgs::Image>();
                    if (image_msg) {
                        // 将ROS图像消息转换为OpenCV图像
                        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
                        in_image = cv_image->image;     // 取出了图像Mat
                        imshow_in_Rviz(in_image);
                        // 图像畸变矫正
                        if (undistort_) {
                            cv::remap(in_image, auxImage, undistMap[0], undistMap[1], cv::INTER_LINEAR);
                            // cv::remap(in_image, auxImage, undistMap[0], undistMap[1], cv::INTER_CUBIC);
                            // cv::remap(in_image, auxImage, undistMap[0], undistMap[1], cv::INTER_LANCZOS4);
                            in_image = auxImage;    // 浅拷贝 内存共享
                        }

                        // 处理图像并获取相机姿态 相对于全局的位姿/每帧  
                        cv::Mat camPose_c2g = Slam.process(in_image/* newframe& */, image_params/*K*/, frameIndex);                        

                        // 07.03 Aruco码相对 全局系 /雷达帧？/的位姿
                        // TODO 用于 Netvlad 的图片信息


                        // 发布位姿
                        publishPose(camPose_c2g, cv_image);

                        if (generate_txt)
                        {
                            camera_full_trajectory(pose_msg_usefor_txt);
                            // 要放在camera_full_trajectory(pose_msg)后面
                            if(TheMap->keyframes.size() > prev_keyframe_number){    // TODO 这个判断是否鲁棒还需研判
                                prev_keyframe_number = TheMap->keyframes.size();
                                whether_KF = full_trajectory(pose_msg_usefor_txt);
                            }
                            else whether_KF = 0;
                            full_trajectory_total_key(whether_KF, pose_msg_usefor_txt);
                        }

                        // publishPointCloud();               // TODO 发布点云

                        // publishMarkerArray();            // 发布地图标记

                        // publishPath();              // 发布相机路径

                        // TODO 添加下标的图像可能是从这里发出去的 发布处理后的图像
                        sensor_msgs::ImagePtr processed_msg = cv_bridge::CvImage(cv_image->header, "bgr8", in_image).toImageMsg();
                        processed_image_pub_.publish(processed_msg);

                        frameIndex++;
                    }
                } catch (cv_bridge::Exception &e) {
                    ROS_ERROR("cv_bridge error: %s", e.what());
                }
            }
            if (msg.getTopic() == lidar_topic_) {   // 处理Lidar数据
                
                // TODO 雷达全部帧位姿(要从r3live拿)
                // TODO 雷达关键帧位姿
                sensor_msgs::PointCloud2ConstPtr cloud_msg = msg.instantiate<sensor_msgs::PointCloud2>();
                if (cloud_msg) {
                    processPointCloud(cloud_msg);}

            } 
        }

    } catch (const std::exception &ex) {
        ROS_ERROR("error: %s", ex.what());
    }
}


int main(int argc, char **argv) {
    
    try {
        printf("ucoslam_node initialized\n");
        ros::init(argc, argv, "ucoslam_node");  // 创建ROS节点句柄
        ros::NodeHandle nh;                     // 创建公共节点句柄 
        ros::NodeHandle private_nh("~");        // 创建私有节点句柄

        UcoSlamNode ucoSlamNode(nh, private_nh);

        // // 从ROS参数服务器获取bag文件路径
        // std::string bag_file;
        // private_nh.param("bag_file", bag_file, std::string(""));

        ucoSlamNode.processBagFile();

    } catch (const std::exception &ex) {
        ROS_ERROR("error: %s", ex.what());
    }
    return 0;
}
