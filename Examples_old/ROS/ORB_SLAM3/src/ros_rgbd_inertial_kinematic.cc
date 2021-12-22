#include <iostream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>   // todo new
#include <cv_bridge/cv_bridge.h>   // 提供ros和opencv之间的接口
#include <tf/transform_listener.h>   // todo new
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>   // todo new

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>   // 处理图像
#include <pcl_conversions/pcl_conversions.h>
#include "../include/ImuTypes.h"   // 处理IMU数据
#include "../include/forward_kinematics.h"
#include "../include/System.h"  // 链接System
#include "message_utils.h" // todo new



using namespace std;

float shift = 0;
float joint_shift = 0;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;  //存储IMU数据的队列，GrabImu函数会将imu数据放入这个队列
    std::mutex mBufMutex;
};


class JointStateGrabber
{
public:
    JointStateGrabber(){};

    void GrabJointState(const sensor_msgs::JointState::ConstPtr &msg);   // 将topic发布的msg存到JointStateBuf队列,回调函数
//    void ReadPositionData();        // 1.提取msg中所需的数据,2.完成运动学数据计算

    std::queue<sensor_msgs::JointState::ConstPtr> JointStateBuf;   // 存储队列
    std::mutex mBufMutexJointState;

    Joint::ForwardKinematics* mpFK;  //指针
};


class ImageGrabber
{
public:
    // 构造函数
    ImageGrabber(ORB_SLAM3::System* pSLAM,
                 ImuGrabber *pImuGb,
                 JointStateGrabber *pJointGb):
                 mpSLAM(pSLAM), mpImuGb(pImuGb), mpJointGb(pJointGb){} // 初始化量

    void GrabImageRgb(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageDepth(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu_Joint();

private:
    queue<sensor_msgs::ImageConstPtr> imgRgbBuf, imgDepthBuf;
    std::mutex mBufMutexRgb,mBufMutexDepth;

    ORB_SLAM3::System* mpSLAM;   // slam系统指针
    ImuGrabber *mpImuGb;         // imugrabber指针
    JointStateGrabber *mpJointGb;   //jointstategrabber指针


    cv::Mat M1l,M2l,M1r,M2r;
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial_Kinematic");   // 初始化ros节点,""中间的是节点名称
    ros::start();
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);//输出ros INFO ,ros的消息日志级别，默认有Debug,Info,Warn,Error,Fatal(首字母大写)


    if(argc < 2 )
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD_Inertial_Kinematic path_to_vocabulary path_to_settings " << endl;
        ros::shutdown();
        return 1;
    }    // 如果不符合传入的文件量,就退出

    ros::NodeHandle n;    // 句柄

    tf::TransformListener listener;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD_KINEMATIC,true);  // 初始化SLAM系统


    // 定义imu\joint\rgbd类的对象
    ImuGrabber imugb;
    JointStateGrabber jointstategb;
    ImageGrabber igb(&SLAM,&imugb,&jointstategb);

    // 读取参数设置文件
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    shift = fsSettings["IMU.shift"];  // 我的yaml文件中没有这个参数
    joint_shift = fsSettings["Joint.shift"];

    ROS_INFO(" waiting for data ... ");

    // Maximum delay, 5 seconds
    // 订阅节点数据, 调用之前两个类中定义的回调函数
    ros::Subscriber sub_imu = n.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_rgb = n.subscribe("/camera/color/image_raw", 100, &ImageGrabber::GrabImageRgb,&igb);
    ros::Subscriber sub_img_depth = n.subscribe("/camera/aligned_depth_to_color/image_raw", 100, &ImageGrabber::GrabImageDepth,&igb);
    ros::Subscriber sub_joint = n.subscribe("/robotis/present_joint_states", 1000, &JointStateGrabber::GrabJointState , &jointstategb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu_Joint,&igb);  //添加关节处理代码 notice 数据同步线程与SLAM指针

    //TODO OCTOMAP添加
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_map(new pcl::PointCloud<pcl::PointXYZRGBA>);
    global_map = SLAM.mpPointCloudMapping->getGlobalMap();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
    //数据格式转换
    cout<<"-----------------------------------------------------------"<<endl;
    cout <<"ros is running "<<endl;
    while (ros::ok())
    {

        pcl::copyPointCloud(*global_map, *global_map_copy);

        ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2> ("/orbslam3/output", 10);

        sensor_msgs::PointCloud2 output;

        pcl::toROSMsg(*global_map_copy,output);// 转换成ROS下的数据类型 最终通过topic发布

        output.header.stamp=ros::Time::now();
        output.header.frame_id  ="camera_rgb_frame";
        //output.header.frame_id  ="map";

        ros::Rate loop_rate(10);

        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    //TODO 结束


    ros::spin();   // 自循环。ros::ok()返回false，ros::spin()就会退出，

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_rosRGBDInertial.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_rosRGBDInertial.txt");

    SLAM.save();  // todo new

    ros::shutdown();

    return 0;
}



void ImageGrabber::GrabImageRgb(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexRgb.lock();
    if (!imgRgbBuf.empty())
        imgRgbBuf.pop();
    imgRgbBuf.push(img_msg);
    mBufMutexRgb.unlock();
}

void ImageGrabber::GrabImageDepth(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexDepth.lock();
    if (!imgDepthBuf.empty())
        imgDepthBuf.pop();
    imgDepthBuf.push(img_msg);
    mBufMutexDepth.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    return cv_ptr->image.clone();

}
/**
 * 1. 同步RGB\Depth时间戳,
 * 2. 用一个vector存储当前帧图像之前所有的IMU数据,
 * 3. 将 处理好的图像\深度图\IMU数据向量传递给 TrackRGBD_kinematic 函数
 * */
void ImageGrabber::SyncWithImu_Joint()   // notice 同步函数,传递数据
{
    std::cout << "SyncWithImuAndJoint start ! " << std::endl;
    const double maxTimeDiff = 0.01;   // 所允许的最大时间误差
    while(1)
    {
        // 定义两个图像
        cv::Mat imRgb, imDepth;
        double tImRgb = 0, tImDepth = 0;
        if (!imgRgbBuf.empty()&&!imgDepthBuf.empty()&&!mpImuGb->imuBuf.empty()&&!mpJointGb->JointStateBuf.empty())
        {
            // notice 图像数据
            // 首先是对齐深度图和RGB的时间戳
            //提取队列中第一个RGBD的stamp时间戳
            tImRgb = imgRgbBuf.front()->header.stamp.toSec();
            tImDepth = imgDepthBuf.front()->header.stamp.toSec();

            // 如果深度图时间大于rgb图的时间戳
            this->mBufMutexDepth.lock();
            while((tImRgb-tImDepth)>maxTimeDiff && imgDepthBuf.size()>1)
            {
                imgDepthBuf.pop();   // 删除队列第一个元素
                tImDepth = imgDepthBuf.front()->header.stamp.toSec();  //重新赋值新的时间戳
            }
            this->mBufMutexDepth.unlock();

            // 如果RGB时间戳大于深度图的时间戳
            this->mBufMutexRgb.lock();
            while((tImDepth-tImRgb)>maxTimeDiff && imgRgbBuf.size()>1)
            {
                imgRgbBuf.pop();
                tImRgb = imgRgbBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRgb.unlock();

            // 如果深度图和RGB图的时间戳差别太大，退出
            if((tImRgb-tImDepth)>maxTimeDiff || (tImDepth-tImRgb)>maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            // 图像时间戳 大于IMU队列最后一个元素的时间戳，退出
            if(tImRgb>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;
            // 以上完成了深度图与RGB的时间戳处理

            // 将队列中的第一帧图像去出来给 imRgb, 并从队列中删除
            this->mBufMutexRgb.lock();
            imRgb = GetImage(imgRgbBuf.front());
            imgRgbBuf.pop();
            this->mBufMutexRgb.unlock();
            // 取出队列中第一个深度图给 imDepth, 并从队列中删除
            this->mBufMutexDepth.lock();
            imDepth = GetImage(imgDepthBuf.front());
            imgDepthBuf.pop();
            this->mBufMutexDepth.unlock();

            // notice IMU数据
            vector<ORB_SLAM3::IMU::Point> vImuMeas;  // 存储IMU测量值, 每一个元素为 IMU中定义的 Point对象
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                // 当IMU队列不为空, 并且imgb类指向IMU对象中,队列的首个数据的时间戳小于等于 RGB的 "由前面代码可知shift为0"
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImRgb+shift)
                {   // 取出imu第一帧的时间戳
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    mpImuGb->imuBuf.pop();   // 添加进vIMUMeas后删除
                }
            }
            mpImuGb->mBufMutex.unlock();

            // notice 关节数据
//            std::cout << "ReadPositionData start ! " << std::endl;
            std::vector<Joint::Data> vJointMeas;  // 存储关节数据
            mpJointGb->mBufMutexJointState.lock();
            if (!mpJointGb->JointStateBuf.empty()){
                vJointMeas.clear();
                while(!mpJointGb->JointStateBuf.empty() && mpJointGb->JointStateBuf.front()->header.stamp.toSec() <= tImRgb + joint_shift)
                {
                    Eigen::VectorXf legr(6),legl(6);
                    double t = mpJointGb->JointStateBuf.front()->header.stamp.toSec();
                    float waist;
                    legl(0)= mpJointGb->JointStateBuf.front()->position[2];
                    legl(1)= mpJointGb->JointStateBuf.front()->position[3];
                    legl(2)= mpJointGb->JointStateBuf.front()->position[5];
                    legl(3)= mpJointGb->JointStateBuf.front()->position[6];
                    legl(4)= mpJointGb->JointStateBuf.front()->position[7];
                    legl(5)= mpJointGb->JointStateBuf.front()->position[8];
                    legr(0)= mpJointGb->JointStateBuf.front()->position[11];
                    legr(1)= mpJointGb->JointStateBuf.front()->position[12];
                    legr(2)= mpJointGb->JointStateBuf.front()->position[14];
                    legr(3)= mpJointGb->JointStateBuf.front()->position[15];
                    legr(4)= mpJointGb->JointStateBuf.front()->position[16];
                    legr(5)= mpJointGb->JointStateBuf.front()->position[17];
                    waist = mpJointGb->JointStateBuf.front()->position[20];
                    vJointMeas.push_back(Joint::Data(legr,legl,waist,t));
                    mpJointGb->JointStateBuf.pop();
                }
            }
            mpJointGb->mBufMutexJointState.unlock();

            /*// notice test
            cout << " testing data ======================================================="<<endl;
            cout << tImRgb <<" ";    //todo
            cout << "IMU size:"<<vImuMeas.size()<< " ";
            cout << "Joint size:"<<vJointMeas.size()<< endl;*/


            mpSLAM->TrackRGBD_kinematic(imRgb,imDepth,tImRgb,vImuMeas,vJointMeas);    // notice 将数据喂给slam系统, 这个函数在System.cc中
//            cout<<"ros cpp: imRgb size: "<< imRgb.cols<<"*"<<imRgb.rows<<endl;
//            cout<<"ros cpp: imDepth size: "<< imDepth.cols<<"*"<<imDepth.rows<<endl;

            ROS_DEBUG("TIME NOW: %f", ros::Time::now().toSec());

            std::chrono::milliseconds tSleep(1);   // 数据处理频率 : 1 ms 1000Hz
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}

void JointStateGrabber::GrabJointState(const sensor_msgs::JointState::ConstPtr &msg){
//    std::cout << "----------" << std::endl;
//    std::cout << msg->name.size() << "\t" << msg->position.size() << "\t" ;
//    std::cout << msg->velocity.size() << "\t" << msg->effort.size() << std::endl;
    mBufMutexJointState.lock();

    JointStateBuf.push(msg);

//    int qSize = JointStateBuf.size();
//    std::cout << qSize << std::endl;
//    std::cout << JointStateBuf.back()->header.stamp << std::endl;


    mBufMutexJointState.unlock();
    return;
}

