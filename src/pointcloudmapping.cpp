/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "Converter.h"
#include "PointCloude.h"
#include "System.h"


int currentloopcount = 0;

/**
 * @param resolution_ :体素大小分辨率，分辨率越小，单个体素越小
 * @param meank_ ： meank_ 为在进行统计时考虑查询邻近点个数
 * @param thresh_：设置距离阈值，其公式是 mean + global stddev_mult * global stddev，即mean+1.0*stddev
 * @return ：无
 */
PointCloudMapping::PointCloudMapping(double resolution_,double meank_,double thresh_)
{
    this->resolution = resolution_;      //分辨率
    this->meank = thresh_;
    this->thresh = thresh_;
    statistical_filter.setMeanK(meank);    //统计估计滤波参数
    statistical_filter.setStddevMulThresh(thresh);
    voxel.setLeafSize( resolution, resolution, resolution);   //设置每个体素子叶分辨率
    globalMap = boost::make_shared< PointCloud >( );

    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

/**
 * @brief 关闭建图线程
 */
void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    //等待PointCloudMapping_viewer 本线程执行结束再执行系统主线程
    viewerThread->join();
}


/**
 * @brief 插入关键帧
 * @param kf    关键帧
 * @param color 关键帧彩色图
 * @param depth 关键帧深度图
 * @param idk   第idk个关键帧
 * @param vpKFs 获取全部关键帧
 * @function    在点云地图里插入关键帧
 */
void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk,vector<KeyFrame*> vpKFs)
{
    cout<<"receive a keyframe, id = "<<idk<<" 第"<<kf->mnId<<"个"<<endl;
    //cout<<"vpKFs数量"<<vpKFs.size()<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    currentvpKFs = vpKFs;
    //colorImgs.push_back( color.clone() );
    //depthImgs.push_back( depth.clone() );
    PointCloude pointcloude;
    pointcloude.pcID = idk;
    pointcloude.T = ORB_SLAM3::Converter::toSE3Quat( kf->GetPose() );  //获取关键帧位姿
    pointcloude.pcE = generatePointCloud(kf,color,depth);  //迭代关键帧点云
    pointcloud.push_back(pointcloude);
    keyFrameUpdated.notify_one();   //通知线程开锁
}

/**
 * @param kf    关键帧
 * @param color 彩色图
 * @param depth 深度图
 * @return 关键帧点云
 */
pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)//,Eigen::Isometry3d T
{
    //新建一个点云
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    //对点云进行
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];  //获取（m,n）处的深度值
            if (d < 0.01 || d>5)            //滤除设备可靠深度范围之外点
                continue;
            PointT p;

            //相机模型，只计算关键帧的点云
            //座标系与pcl座标系相反，所以可以p.z=-d
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

            //彩色图计算点云颜色
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    //Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    //PointCloud::Ptr cloud(new PointCloud);
    //pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    //cloud->is_dense = false;

//    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return tmp;
}

/*
 * @brief 显示点云线程
 */
void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {

        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(loopbusy || bStop)
        {
            //cout<<"loopbusy || bStop"<<endl;
            continue;
        }
        //cout<<lastKeyframeSize<<"    "<<N<<endl;
        if(lastKeyframeSize == N)
            cloudbusy = false;
        //cout<<"待处理点云个数 = "<<N<<endl;
        cloudbusy = true;
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {


            PointCloud::Ptr p (new PointCloud);
            //将点云数据转换成ascii码形式存储在pcd文件中
            //1、源点云   2、转变后的点云   3、位姿变换矩阵
            pcl::transformPointCloud( *(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
            cout<<"处理好第i个点云"<<i<<endl;
            //  转换后的点云叠加存储在globalMap中
            *globalMap += *p;
            //PointCloud::Ptr tmp(new PointCloud());
            //voxel.setInputCloud( globalMap );
            // voxel.filter( *tmp );
            //globalMap->swap( *tmp );


        }

        // depth filter and statistical removal
        //这里的滤波只是显示上的滤波，不会改变globalMap的值
        PointCloud::Ptr tmp1 ( new PointCloud );

        statistical_filter.setInputCloud(globalMap);   //对globalMap进行统计学去噪
        statistical_filter.filter( *tmp1 );         // 执行去噪计算并保存点到 tmp1

        //体素滤波器voxel filter进行降采样
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *globalMap );
        //globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        cout<<"show global map, size="<<N<<"   "<<globalMap->points.size()<<endl;
        lastKeyframeSize = N;
        cloudbusy = false;
        //*globalMap = *tmp1;

        //if()
        //{

        //}
    }
}

/**
 * @brief 保存pcd地图
 */
void PointCloudMapping::save()
{
    pcl::io::savePCDFile( "result.pcd", *globalMap );
    cout<<"globalMap save finished"<<endl;
}

/**
 * @brief 更新点云*/
void PointCloudMapping::updatecloud()
{
    if(!cloudbusy)
    {
        loopbusy = true;
        cout<<"startloopmappoint"<<endl;
        PointCloud::Ptr tmp1(new PointCloud);
        for (int i=0;i<currentvpKFs.size();i++)
        {
            for (int j=0;j<pointcloud.size();j++)
            {
                if(pointcloud[j].pcID==currentvpKFs[i]->mnFrameId)
                {
                    Eigen::Isometry3d T = ORB_SLAM3::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
                    PointCloud::Ptr cloud(new PointCloud);
                    pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
                    *tmp1 +=*cloud;

                    cout<<"第pointcloud"<<j<<"与第vpKFs"<<i<<"匹配"<<endl;
                    continue;
                }
            }
        }
        cout<<"finishloopmap"<<endl;
        PointCloud::Ptr tmp2(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *tmp2 );
        globalMap->swap( *tmp2 );
        //viewer.showCloud( globalMap );
        loopbusy = false;
        //cloudbusy = true;
        loopcount++;

        //*globalMap = *tmp1;
    }
}

//获取全局点云地图点，智能指针，return 回来
pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::getGlobalMap() {

    return this->globalMap;
}

// add
bool PointCloudMapping::isPointCloudMapUpdated()
{
    unique_lock<mutex> lock(mMtxPointCloudUpdated);
    return mbPointCloudMapUpdated;
}


void PointCloudMapping::setPointCloudMapUpdatedFlag(bool bFlag)
{
    unique_lock<mutex> lock(mMtxPointCloudUpdated);
    mbPointCloudMapUpdated = bFlag;
}


PointCloudMapping::PointCloud PointCloudMapping::GetPointCloudMap()
{
    unique_lock<mutex> lock(mMtxPointCloudUpdated);
    return *globalMap;
}