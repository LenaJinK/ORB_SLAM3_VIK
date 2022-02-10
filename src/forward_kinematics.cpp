//
// Created by lenajin on 23-8-22.
//
#include "forward_kinematics.h"
#include <iostream>
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/kdl.hpp"
#include <kdl/chain.hpp>
#include "eigen_kdl.h"


using namespace Joint;
using namespace std;

ForwardKinematics::ForwardKinematics(const string &urdf_path){
    // 构造kdl tree
    KDL::Tree my_tree;
    kdl_parser::treeFromFile(urdf_path,my_tree);

    // 创建运动链接
    bool exit_value;
    exit_value = my_tree.getChain("BODY","L_ANK_ROLL",leftleg_chain);
    exit_value = my_tree.getChain("BODY","R_ANK_ROLL",rightleg_chain);
}
void ForwardKinematics::CalculateNewMeasurement(const Eigen::VectorXf &right, const Eigen::VectorXf &left, float &w, double &t){

    mvMeasurements.push_back(jointable(right,left,w,t));

    for(int i=0;i<6;i++){
        r_joint_position.push_back(right[i]);
        l_joint_position.push_back(left[i]);
    }
    waist = w;

    ComputeLegData();

    if( mvMeasurements.size() == 1){
        T_r_pre.setRotationMatrix(Trtemp.rotation().cast<float>());
        T_r_pre.translation()=Trtemp.translation().cast<float>();
        T_l_pre.setRotationMatrix(Tltemp.rotation().cast<float>());
        T_l_pre.translation()=Tltemp.translation().cast<float>();
        support_state_pre = support_leg(T_r_pre,T_l_pre);
        timestamp_pre = t;
    }else{
        T_r.setRotationMatrix(Trtemp.rotation().cast<float>());
        T_r.translation()=Trtemp.translation().cast<float>();
        T_l.setRotationMatrix(Tltemp.rotation().cast<float>());
        T_l.translation()=Tltemp.translation().cast<float>();
        support_state = support_leg( T_r, T_l );
        timestamp = t;
    }

//    cout << " finish Calculate New Joint Measurement" <<endl;

}
void ForwardKinematics::ComputeLegData(){
    // creat fk solver
    KDL::ChainFkSolverPos_recursive fksolver_l = KDL::ChainFkSolverPos_recursive(leftleg_chain);
    KDL::ChainFkSolverPos_recursive fksolver_r = KDL::ChainFkSolverPos_recursive(rightleg_chain);
    // array
    unsigned int nl = leftleg_chain.getNrOfJoints();
    KDL::JntArray jointpositions_left = KDL::JntArray(nl);
    {
        jointpositions_left(0)=waist;
        jointpositions_left(1)=l_joint_position[4];
        jointpositions_left(2)=l_joint_position[3];
        jointpositions_left(3)=l_joint_position[2];
        jointpositions_left(4)=l_joint_position[5];
        jointpositions_left(5)=l_joint_position[0];
        jointpositions_left(6)=l_joint_position[1];

    }
    unsigned int nr = rightleg_chain.getNrOfJoints();
    KDL::JntArray jointpositions_right = KDL::JntArray(nr);
    {
        jointpositions_right(0)=waist;
        jointpositions_right(1)=r_joint_position[4];
        jointpositions_right(2)=r_joint_position[3];
        jointpositions_right(3)=r_joint_position[2];
        jointpositions_right(4)=r_joint_position[5];
        jointpositions_right(5)=r_joint_position[0];
        jointpositions_right(6)=r_joint_position[1];
    }

    bool kinematics_status;
    kinematics_status = fksolver_l.JntToCart(jointpositions_left,left_result);
    if(kinematics_status>=0){
//        std::cout << left_result <<std::endl;
//        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
    kinematics_status = fksolver_r.JntToCart(jointpositions_right,right_result);
    if(kinematics_status>=0){
//        std::cout << right_result <<std::endl;
//        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }

    tf::transformKDLToEigen(right_result,Trtemp);
    tf::transformKDLToEigen(left_result,Tltemp);

    return;
}


// 右支撑-0,左支撑-1,双支撑-2
int ForwardKinematics::support_leg(const Sophus::SE3f &T_r, const Sophus::SE3f &T_l){
    float e_rl = abs(T_r.translation().z()) - abs(T_l.translation().z());  //z axis
    if(fabs(e_rl) <= 0.01 ) {
        return 2;
    }else if( e_rl > 0){
        return 0;
    }else if( e_rl < 0 ) {
        return 1;
    }
}
void ForwardKinematics::ComputeTij() {
    if(support_state == support_state_pre){
        if(support_state==0 || support_state==2){
            Tij = T_r_pre*T_r.inverse();
        }
        if(support_state==1){
            Tij = T_l_pre*T_l.inverse();
        }
    }else{
        if(support_state_pre == 0){
            Eigen::Vector3f tij;
            tij = ((T_r_pre.inverse()*T_l_pre).translation() + (T_r.inverse()*T_l).translation())*0.5f;
            tij.z()=0;
            Sophus::SE3f Tsisj(Eigen::Matrix3f::Identity(),tij);
            Tij=T_r_pre*Tsisj*T_l.inverse();
        }
        if(support_state_pre==1){
            Eigen::Vector3f tij;
            tij = ((T_l_pre.inverse()*T_r_pre).translation() + (T_l.inverse()*T_r).translation())*0.5f;
            tij.z()=0;
            Sophus::SE3f Tsisj(Eigen::Matrix3f::Identity(),tij);
            Tij=T_l_pre*Tsisj*T_r.inverse();
        }
        if(support_state_pre == 2){
            if(support_state==0)Tij = T_r_pre*T_r.inverse();
            if(support_state==1)Tij = T_l_pre*T_l.inverse();
        }
    }
}

/*Calib::Calib(const string &urdf_path) {
    // 构造kdl tree
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromFile(urdf_path,my_tree));

    // 创建运动链接
    bool exit_value;
    exit_value = my_tree.getChain("BODY","camera",camera_chain);
    KDL::ChainFkSolverPos_recursive fksolver_l = KDL::ChainFkSolverPos_recursive(camera_chain);
    unsigned int nl = camera_chain.getNrOfJoints();
    KDL::JntArray jointpositions_head = KDL::JntArray(nl);
    {
        jointpositions_head(0)=0.0;
        jointpositions_head(1)=0.0;
        jointpositions_head(2)=0.0;
        jointpositions_head(3)=0.0;
    }
    bool kinematics_status;
    kinematics_status = fksolver_l.JntToCart(jointpositions_head,camera_result);
     *//*if(kinematics_status>=0){
                std::cout <<camera_result <<std::endl;
                printf("%s \n","Succes, thanks KDL!");
     }else{
         printf("%s \n","Error: could not calculate forward kinematics :(");
     }*//*
    tf::transformKDLToEigen(camera_result,Tcomctemp);

    Sophus::SE3f T(Tcomctemp.rotation().cast<float>(),Tcomctemp.translation().cast<float>());
    Set(T);
}*/
void Calib::Set(const Sophus::SE3<float> &sophTcomc) {
    mbIsSet = true;
    // Sophus/Eigen
    mTcomc = sophTcomc;
    mTccom = mTcomc.inverse();
}

Calib::Calib(const Calib &calib)
{
    mbIsSet = calib.mbIsSet;
    // Sophus/Eigen parameters
    mTcomc = calib.mTcomc;
    mTccom = calib.mTccom;
}