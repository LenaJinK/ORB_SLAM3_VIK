#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>
#include <vector>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include "eigen_kdl.h"
#include <mutex>

#ifndef FK_TEST_FORWARD_KINEMATICS_H
#define FK_TEST_FORWARD_KINEMATICS_H


using namespace std;
namespace Joint {
    typedef Eigen::Matrix<float,6,1> Vector6f;

    class Data{
    public:

        Data(const Eigen::VectorXf legr, const Eigen::VectorXf legl, const float waist,const double &timestamp):
                 LegR(legr),LegL(legl),Waist(waist),t(timestamp){}
    public:
        Eigen::Matrix<float,6,1>  LegR;
        Eigen::Matrix<float,6,1>  LegL;
        float Waist;
        double t;
    };

    class ForwardKinematics {
    public:
        ForwardKinematics(const string &urdf_path);
        ForwardKinematics(ForwardKinematics* pJointPre);  // 由之前的数据
        ForwardKinematics(){}
        ~ForwardKinematics(){}

        // 运动链求解
        void ComputeLegData();
        void CalculateNewMeasurement(const Eigen::VectorXf &right, const Eigen::VectorXf &left, float &w, double &t);

        // 支撑腿
        int support_leg( const Sophus::SE3f &T_r, const Sophus::SE3f &T_l);
        void ComputeTij();

    public:
        // 转存的数据
        std::vector<float> r_joint_position;
        std::vector<float> l_joint_position;
        float waist;
        double timestamp,timestamp_pre;
        // 运动链
        KDL::Chain leftleg_chain;
        KDL::Chain rightleg_chain;
        // 结果
        KDL::Frame left_result,right_result;                              // 运动链计算结果
        Sophus::SE3f T_r;            // 将kdl格式转换为 eigen
        Sophus::SE3f T_l;
        Sophus::SE3f T_r_pre;
        Sophus::SE3f T_l_pre;
        // 支撑状态
        int support_state, support_state_pre;
        Sophus::SE3f Tij;


    private:
        struct jointable
        {
            jointable(){}
            jointable(const Eigen::VectorXf &lr_, const Eigen::VectorXf &ll_, const float &w_,const double &t_):
                    LegR(lr_),LegL(ll_),Waist(w_),t(t_){}

            Vector6f LegR;
            Vector6f LegL;
            float Waist;
            double t;
        };

        std::vector<jointable> mvMeasurements;
        std::mutex mMutex;
        Eigen::Isometry3d Trtemp = Eigen::Isometry3d::Identity(),Tltemp= Eigen::Isometry3d::Identity();

    };

    class Calib
    {

    public:

        Calib(const Sophus::SE3<float> &Tcomc){
            Set(Tcomc);
        }

        Calib(const Calib &calib);
        Calib(){mbIsSet = false;}

        //void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const float &ngw, const float &naw);
        void Set(const Sophus::SE3<float> &sophTcomc);

    public:
        // Sophus/Eigen implementation
        Sophus::SE3<float> mTcomc;
        Sophus::SE3<float> mTccom;
        bool mbIsSet;
        KDL::Chain camera_chain;
        KDL::Frame camera_result;
        Eigen::Isometry3d Tcomctemp = Eigen::Isometry3d::Identity();
    };
}


#endif //FK_TEST_FORWARD_KINEMATICS_H


