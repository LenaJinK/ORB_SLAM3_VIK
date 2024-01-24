//
// Created by yonghui on 19-10-30.
//

// ROS
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include "tf_conversions/tf_eigen.h"

#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <sophus/se3.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message_utils.h"

namespace ORB_SLAM3_DENSE {
    MessageUtils::MessageUtils(tf::TransformListener &listener, ORB_SLAM3::System *pSystem) :
            private_nh_("~"), listener_(listener), mpSystem_(pSystem), pcl_map_(new PointCloudMapping::PointCloud()),
            pcl_plane_(new PointCloudMapping::PointCloud()) {
        // initial plane coefficients ( xy plane) 初始化平面系数，xy平面
        plane_coeffs_ << 0.0, 0.0, 1.0, 0.0;
        last_plane_coeffs_ << 0.0, 0.0, 1.0, 0.0;

        // initalize publisher and subscriber 初始化发布者和订阅者 确认topic name 和 queue size
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);        // ROS 导航用的nav_msg
        frame_pub_ = nh_.advertise<sensor_msgs::Image>("slam_frame", 10);  // ROS 图片消息
        pcl2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);  // ROS 点云消息

        // get parameters ros param 维护的是一个字典，里面存储的是key、value，每一个key对应一个value  默认参数是bool类型
        private_nh_.param("use_odom_pub", use_odom_pub_, true);
        private_nh_.param("use_frame_pub", use_frame_pub_, true);
        private_nh_.param("use_pcl_pub", use_pcl_pub_, true);
//
        //get frame id  坐标系的id ：名称 - 默认参数str类型 - 默认值
        private_nh_.param("map_frame", map_frame_, std::string("map"));
        private_nh_.param("body_frame", body_frame_, std::string("body"));  // todo new 改成 body frame
        private_nh_.param("camera_frame", camera_frame_, std::string("camera"));

        // get slam thread pointer
        mpTracker_ = mpSystem_->GetTracker();
        mpFrameDrawer_ = mpSystem_->GetFrameDrawer();
        mpMapDrawer_ = mpSystem_->GetMapDrawer();
        mpPclMapper_ = mpSystem_->GetPointCloudMapper();
    }

    // todo 发布里程计消息 odom_frame<--optical_frame
    void MessageUtils::publishOdometry() {
        // step 1 判断状态
        if (mpTracker_->mState == ORB_SLAM3::Tracking::LOST) {
            ROS_WARN("ORB_SLAM3 has lost tracking.");
            return;
        }
        if (mpTracker_->mState != ORB_SLAM3::Tracking::OK)     // 不是OK就返回
            return;

        // step 2 获取需要的变换矩阵
        Sophus::SE3f Tcw_SE3f = mpTracker_->mCurrentFrame.GetPose();
        Sophus::SE3f Twc_SE3f = Tcw_SE3f.inverse();
        Sophus::SE3f Twb_SE3f = mpTracker_->mCurrentFrame.GetCoMPose();
        Sophus::SE3f Tbc_SE3f = Twb_SE3f.inverse() * Twc_SE3f;

        // step 3 转换成tf类型数据
        tf::Transform Twb, Tbc;
        Twb.setOrigin(tf::Vector3(Twb_SE3f.translation().x(), Twb_SE3f.translation().y(), Twb_SE3f.translation().z() + 0.313)); // 313是机器人地面到body的大概距离
        Eigen::Quaternionf q(Twb_SE3f.rotationMatrix());
        q.normalize();   // 归一化
        Twb.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));

        Tbc.setOrigin(tf::Vector3(Tbc_SE3f.translation().x(), Tbc_SE3f.translation().y(), Tbc_SE3f.translation().z()));
        Eigen::Quaternionf q2(Tbc_SE3f.rotationMatrix());
        q2.normalize();
        Tbc.setRotation(tf::Quaternion(q2.x(), q2.y(), q2.z(), q2.w()));

        // step 4 发布tf
        broadcaster_.sendTransform(tf::StampedTransform(Twb, ros::Time::now(), "world", body_frame_));
        broadcaster_.sendTransform(tf::StampedTransform(Tbc, ros::Time::now(), body_frame_, camera_frame_));

        // publish odom message
        /*    nav_msgs::Odometry odom_msgs;
            odom_msgs.header.stamp = ros::Time::now();
            odom_msgs.header.frame_id = odom_frame_;     // 父坐标系
            odom_msgs.child_frame_id = optical_frame_;   // 子坐标系
            tf::poseTFToMsg(Twb, odom_msgs.pose.pose);  // 发布的是Twcom topic
            odom_pub_.publish(odom_msgs);*/

        // update SLAM PointCloudMapping thread extrinsic matrix
        // step 5 更新点云地图  -》因为源代码直接赋值单位矩阵，这里保持一致也用单位矩阵
        Eigen::Matrix3d matRbc;
        matRbc.setIdentity();
        Eigen::Vector3d mattbc(0, 0, 0);
        mpPclMapper_->updateTbc(matRbc, mattbc);
    }

    // todo 发布图像
    void MessageUtils::publishFrame() {
        if (!use_frame_pub_)
            return;
        // draw current frame
        cv::Mat frame_mat = mpFrameDrawer_->DrawFrame();

        // publish
        std_msgs::Header h;
        h.stamp = ros::Time::now();
        h.frame_id = camera_frame_;
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage(h, sensor_msgs::image_encodings::BGR8, frame_mat));
        frame_pub_.publish(cv_ptr->toImageMsg());
    }

    // todo 发布点云地图
    void MessageUtils::publishPointCloud() {
        if (!use_pcl_pub_ || !mpPclMapper_->getPointCloudMapUpdatedFlag())
            return;

        // update point cloud
        if (mpPclMapper_->getUsePlanSegmentationFlag()) {
            mpPclMapper_->getObstacleCloud(pcl_map_);
            mpPclMapper_->getPlaneCloud(pcl_plane_);
            mpPclMapper_->getPlaneCoeffs(plane_coeffs_);
        } else {
            mpPclMapper_->getGlobalCloud(pcl_map_);
        }

        // transform point cloud: footprint_base<--optical_base
        pcl_map_->width = pcl_map_->size();  //! Occassionally it will conflict, force to equal

        // set map frame
        tf::Transform Tbm;
        Tbm.setOrigin( tf::Vector3(0,0,0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        Tbm.setRotation(q);
        tf::StampedTransform trans(Tbm, ros::Time::now(), "world" ,map_frame_);
        broadcaster_.sendTransform(trans);

        // publish: sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 pcl2_msgs;
        pcl::toROSMsg(*pcl_map_, pcl2_msgs);
        pcl2_msgs.header.stamp = ros::Time::now();
        pcl2_msgs.header.frame_id = map_frame_;
        pcl2_pub_.publish(pcl2_msgs);
        mpPclMapper_->setPointCloudMapUpdatedFlag(false);

    }
}