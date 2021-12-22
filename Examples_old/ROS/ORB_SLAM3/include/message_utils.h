//
// Created by lenajin on 23-11-28.
//

#ifndef ORB_SLAM3_MESSAGE_UTILS_H
#define ORB_SLAM3_MESSAGE_UTILS_H

// ORBSLAM3
#include "System.h"
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "pointcloudmapping.h"
#include "PointCloude.h"

// PCL
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


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

namespace ORB_SLAM3_DENSE{

    class MessageUtils{
    public:
        MessageUtils(ORB_SLAM3::System *pSystem);

        void PublishOdometry();

        void PublishFrame();

        void PublishPointCloud();

    protected:
        // pub and sub
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        tf::TransformBroadcaster transform_;
        ros::Publisher odom_pub_;
        ros::Publisher frame_pub_;
        ros::Publisher pcl_pub_;
        ros::Publisher pcl2_pub_;

        // paramters
        bool use_odom_pub_;
        bool use_tf_;
        bool use_frame_pub_;
        bool use_pcl_pub_;
        std::string global_frame_;
        std::string base_frame_;

        // ORB_SLAM3 pointer
        ORB_SLAM3::System *mpSystem_;
        ORB_SLAM3::Tracking *mpTracker_;
        ORB_SLAM3::FrameDrawer *mpFrameDrawer_;
        ORB_SLAM3::MapDrawer *mpMapDrawer_;
        std::shared_ptr<PointCloudMapping> mpPclMapper_;

        // point cloud map
        PointCloudMapping::PointCloud pcl_map_;
    };
}

#endif //ORB_SLAM3_MESSAGE_UTILS_H
