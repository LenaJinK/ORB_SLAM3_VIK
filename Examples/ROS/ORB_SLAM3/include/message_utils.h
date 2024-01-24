//
// Created by yonghui on 19-10-30.
//

#ifndef ORB_SLAM3_MESSAGE_UTILS_H
#define ORB_SLAM3_MESSAGE_UTILS_H

// ORB_SLAM3
#include "System.h"
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "pointcloudmapping.h"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <string>
#include <memory>

namespace ORB_SLAM3
{
    class System;
    class Tracking;
    class FrameDrawer;
    class MapDrawer;
}
class PointCloudMapping;


namespace ORB_SLAM3_DENSE
{
    class MessageUtils
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MessageUtils(tf::TransformListener &listener, ORB_SLAM3::System *pSystem);

        void publishOdometry();

        void publishFrame();

        void publishPointCloud();

    protected:
        bool getTransformedPose(tf::Stamped<tf::Transform> &output_pose, const string &target_frame, const double &timeout=1.0);

        bool getTransformedPose(Eigen::Matrix4d &output_mat, const string &target_frame, const string &source_frame, const double &timeout=1.0);

        // node handler
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // tf
        tf::TransformListener &listener_;
        tf::TransformBroadcaster broadcaster_;

        // pub and sub
        ros::Publisher odom_pub_;
        ros::Publisher frame_pub_;
        ros::Publisher pcl2_pub_;

        // parameters bool类型
        bool use_odom_pub_;
        bool use_frame_pub_;
        bool use_pcl_pub_;

        // frame id  名称 str类型
        std::string map_frame_;
        std::string camera_frame_;    // 赋值Tcw
        std::string body_frame_;       // 赋值Twcom

        // ORB_SLAM3 pointer
        ORB_SLAM3::System *mpSystem_;
        ORB_SLAM3::Tracking *mpTracker_;
        ORB_SLAM3::FrameDrawer *mpFrameDrawer_;
        ORB_SLAM3::MapDrawer *mpMapDrawer_;
        std::shared_ptr<PointCloudMapping> mpPclMapper_;

        // point cloud map
        PointCloudMapping::PointCloud::Ptr pcl_map_;     // map point cloud
        PointCloudMapping::PointCloud::Ptr pcl_plane_;   // plane point cloud (ground)
        Eigen::Vector4d plane_coeffs_;             // 平面系数
        Eigen::Vector4d last_plane_coeffs_;        // 上一帧平面系数
    };
}

#endif //ORB_SLAM3_MESSAGE_UTILS_H
