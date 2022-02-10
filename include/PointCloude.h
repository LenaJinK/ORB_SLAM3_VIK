//
// Created by lenajin on 23-11-28.
//

#ifndef ORB_SLAM3_POINTCLOUDE_H
#define ORB_SLAM3_POINTCLOUDE_H


#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <mutex>

// todo 取消了原本的命名空间  namespace
class PointCloude
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
public:
    PointCloud::Ptr pcE;
public:
    Eigen::Isometry3d T;
    int pcID;
};



#endif //ORB_SLAM3_POINTCLOUDE_H
