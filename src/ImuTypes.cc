/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "ImuTypes.h"
#include "Converter.h"

#include "GeometricTools.h"

#include<iostream>

namespace ORB_SLAM3
{

namespace IMU
{

const float eps = 1e-4;

Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R){
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z)
{
    Eigen::Matrix3f I;
    I.setIdentity();
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);
    if(d<eps) {
        return I;
    }
    else {
        return I - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v)
{
    return RightJacobianSO3(v(0),v(1),v(2));
}

Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z)
{
    Eigen::Matrix3f I;
    I.setIdentity();
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);

    if(d<eps) {
        return I;
    }
    else {
        return I + W/2 + W*W*(1.0f/d2 - (1.0f+cos(d))/(2.0f*d*sin(d)));
    }
}

Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v)
{
    return InverseRightJacobianSO3(v(0),v(1),v(2));
}

IntegratedRotation::IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time) {
    const float x = (angVel(0)-imuBias.bwx)*time;
    const float y = (angVel(1)-imuBias.bwy)*time;
    const float z = (angVel(2)-imuBias.bwz)*time;

    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);

    Eigen::Vector3f v;
    v << x, y, z;
    Eigen::Matrix3f W = Sophus::SO3f::hat(v);
    if(d<eps)
    {
        deltaR = Eigen::Matrix3f::Identity() + W;
        rightJ = Eigen::Matrix3f::Identity();
    }
    else
    {
        deltaR = Eigen::Matrix3f::Identity() + W*sin(d)/d + W*W*(1.0f-cos(d))/d2;
        rightJ = Eigen::Matrix3f::Identity() - W*(1.0f-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Preintegrated::Preintegrated(const Bias &b_, const Calib &calib)
{
    Nga = calib.Cov;
    NgaWalk = calib.CovWalk;
    Initialize(b_);
}

// Copy constructor
Preintegrated::Preintegrated(Preintegrated* pImuPre): dT(pImuPre->dT),C(pImuPre->C), Info(pImuPre->Info),
     Nga(pImuPre->Nga), NgaWalk(pImuPre->NgaWalk), b(pImuPre->b), dR(pImuPre->dR), dV(pImuPre->dV),
    dP(pImuPre->dP), JRg(pImuPre->JRg), JVg(pImuPre->JVg), JVa(pImuPre->JVa), JPg(pImuPre->JPg), JPa(pImuPre->JPa),
    avgA(pImuPre->avgA), avgW(pImuPre->avgW), bu(pImuPre->bu), db(pImuPre->db), mvMeasurements(pImuPre->mvMeasurements)
{

}

void Preintegrated::CopyFrom(Preintegrated* pImuPre)
{
    dT = pImuPre->dT;
    C = pImuPre->C;
    Info = pImuPre->Info;
    Nga = pImuPre->Nga;
    NgaWalk = pImuPre->NgaWalk;
    b.CopyFrom(pImuPre->b);
    dR = pImuPre->dR;
    dV = pImuPre->dV;
    dP = pImuPre->dP;
    JRg = pImuPre->JRg;
    JVg = pImuPre->JVg;
    JVa = pImuPre->JVa;
    JPg = pImuPre->JPg;
    JPa = pImuPre->JPa;
    avgA = pImuPre->avgA;
    avgW = pImuPre->avgW;
    bu.CopyFrom(pImuPre->bu);
    db = pImuPre->db;
    mvMeasurements = pImuPre->mvMeasurements;
}


void Preintegrated::Initialize(const Bias &b_)
{
    dR.setIdentity();
    dV.setZero();
    dP.setZero();
    JRg.setZero();
    JVg.setZero();
    JVa.setZero();
    JPg.setZero();
    JPa.setZero();
    C.setZero();
    Info.setZero();
    db.setZero();
    b=b_;
    bu=b_;
    avgA.setZero();
    avgW.setZero();
    dT=0.0f;
    mvMeasurements.clear();
}

void Preintegrated::Reintegrate()
{
    std::unique_lock<std::mutex> lock(mMutex);
    const std::vector<integrable> aux = mvMeasurements;
    Initialize(bu);
    for(size_t i=0;i<aux.size();i++)
        IntegrateNewMeasurement(aux[i].a,aux[i].w,aux[i].t);
}
/**
// Notice 预积分计算 核心函数
 * @brief 预积分计算，更新noise
 *
 * @param[in] acceleration  加速度计数据
 * @param[in] angVel        陀螺仪数据
 * @param[in] dt            两图像 帧之间时间差
 */
void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt)
{
    // 保存imu数据，利用中值积分的结果构造一个预积分类保存在mvMeasurements中
    mvMeasurements.push_back(integrable(acceleration,angVel,dt));

    // Position is updated firstly, as it depends on previously computed velocity and rotation.
    // Velocity is updated secondly, as it depends on previously computed rotation.
    // Rotation is the last to be updated.

    //Matrices to compute covariance
    // 1. 构造协方差矩阵
    Eigen::Matrix<float,9,9> A;  // 噪声矩阵的传递矩阵，这部分用于计算i到j-1历史噪声或者协方差
    A.setIdentity();
    Eigen::Matrix<float,9,6> B;  // 噪声矩阵的传递矩阵，这部分用于计算j-1新的噪声或协方差，这两个矩阵里面的数都是当前时刻的，计算主要是为了下一时刻使用
    B.setZero();

    // 考虑偏置后的加速度、角速度
    Eigen::Vector3f acc, accW;
    acc << acceleration(0)-b.bax, acceleration(1)-b.bay, acceleration(2)-b.baz;
    accW << angVel(0)-b.bwx, angVel(1)-b.bwy, angVel(2)-b.bwz;
    // 记录平均加速度和角速度
    avgA = (dT*avgA + dR*acc*dt)/(dT+dt);
    avgW = (dT*avgW + accW*dt)/(dT+dt);

    // Update delta position dP and velocity dV (rely on no-updated delta rotation)
    // 根据没有更新的dR来更新dP与dV  eq.(38) 更新顺序：dp-dv-dR
    dP = dP + dV*dt + 0.5f*dR*acc*dt*dt;
    dV = dV + dR*acc*dt;

    // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
    // 根据η_ij = A * η_i,j-1 + B_j-1 * η_j-1中的Ａ矩阵和Ｂ矩阵对速度和位移进行更新
    Eigen::Matrix<float,3,3> Wacc = Sophus::SO3f::hat(acc);

    A.block<3,3>(3,0) = -dR*dt*Wacc;
    A.block<3,3>(6,0) = -0.5f*dR*dt*dt*Wacc;
    A.block<3,3>(6,3) = Eigen::DiagonalMatrix<float,3>(dt, dt, dt);
    B.block<3,3>(3,3) = dR*dt;
    B.block<3,3>(6,3) = 0.5f*dR*dt*dt;


    // Update position and velocity jacobians wrt bias correction

    // 因为随着时间推移，不可能每次都重新计算雅克比矩阵，所以需要做J(k+1) = j(k) + (~)这类事，分解方式与AB矩阵相同
    // 论文作者对forster论文公式的基础上做了变形，然后递归更新，参见 https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/212
    JPa = JPa + JVa*dt -0.5f*dR*dt*dt;
    JPg = JPg + JVg*dt -0.5f*dR*dt*dt*Wacc*JRg;
    JVa = JVa - dR*dt;
    JVg = JVg - dR*dt*Wacc*JRg;

    // Update delta rotation
    // Step 2. 构造函数，会根据更新后的bias进行角度积分
    IntegratedRotation dRi(angVel,b,dt);
    dR = NormalizeRotation(dR*dRi.deltaR);// 强行归一化使其符合旋转矩阵的格式

    // Compute rotation parts of matrices A and B
    // 补充AB矩阵    因为AB矩阵会用到dR，所以需要更新
    A.block<3,3>(0,0) = dRi.deltaR.transpose();
    B.block<3,3>(0,0) = dRi.rightJ*dt;

    // Update covariance
    // 小量delta初始为0，更新后通常也为0，故省略了小量的更新
    // Step 3.更新协方差，frost经典预积分论文的第63个公式，推导了噪声（ηa, ηg）对dR dV dP 的影响
    C.block<9,9>(0,0) = A * C.block<9,9>(0,0) * A.transpose() + B*Nga*B.transpose();// B矩阵为9*6矩阵 Nga 6*6对角矩阵，3个陀螺仪噪声的平方，
    // 这一部分最开始是0矩阵，随着积分次数增加，每次都加上随机游走，偏置的信息矩阵
    C.block<6,6>(9,9) += NgaWalk;

    // Update rotation jacobian wrt bias correction
    // 计算偏置的雅克比矩阵，r对bg的导数，∂ΔRij/∂bg = (ΔRjj-1) * ∂ΔRij-1/∂bg - Jr(j-1)*t
    // 论文作者对forster论文公式的基础上做了变形，然后递归更新，参见 https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/212
    // ? 为什么先更新JPa、JPg、JVa、JVg最后更新JRg? 答：这里必须先更新dRi才能更新到这个值，但是为什么JPg和JVg依赖的上一个JRg值进行更新的？
    JRg = dRi.deltaR.transpose()*JRg - dRi.rightJ*dt;

    // Total integrated time
    dT += dt;  // 更新总时间
}

void Preintegrated::MergePrevious(Preintegrated* pPrev)
{
    if (pPrev==this)
        return;

    std::unique_lock<std::mutex> lock1(mMutex);
    std::unique_lock<std::mutex> lock2(pPrev->mMutex);
    Bias bav;
    bav.bwx = bu.bwx;
    bav.bwy = bu.bwy;
    bav.bwz = bu.bwz;
    bav.bax = bu.bax;
    bav.bay = bu.bay;
    bav.baz = bu.baz;

    const std::vector<integrable > aux1 = pPrev->mvMeasurements;
    const std::vector<integrable> aux2 = mvMeasurements;

    Initialize(bav);
    for(size_t i=0;i<aux1.size();i++)
        IntegrateNewMeasurement(aux1[i].a,aux1[i].w,aux1[i].t);
    for(size_t i=0;i<aux2.size();i++)
        IntegrateNewMeasurement(aux2[i].a,aux2[i].w,aux2[i].t);

}

void Preintegrated::SetNewBias(const Bias &bu_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    bu = bu_;

    db(0) = bu_.bwx-b.bwx;
    db(1) = bu_.bwy-b.bwy;
    db(2) = bu_.bwz-b.bwz;
    db(3) = bu_.bax-b.bax;
    db(4) = bu_.bay-b.bay;
    db(5) = bu_.baz-b.baz;
}
// notice 根据新的零偏计算新的bias\旋转\速度\位置的预积分
IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    return IMU::Bias(b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz,b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz);
}


Eigen::Matrix3f Preintegrated::GetDeltaRotation(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    Eigen::Vector3f dbg;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
}

Eigen::Vector3f Preintegrated::GetDeltaVelocity(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    dba << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz;
    return dV + JVg * dbg + JVa * dba;
}

Eigen::Vector3f Preintegrated::GetDeltaPosition(const Bias &b_)
{
    std::unique_lock<std::mutex> lock(mMutex);
    Eigen::Vector3f dbg, dba;
    dbg << b_.bwx-b.bwx,b_.bwy-b.bwy,b_.bwz-b.bwz;
    dba << b_.bax-b.bax,b_.bay-b.bay,b_.baz-b.baz;
    return dP + JPg * dbg + JPa * dba;
}

Eigen::Matrix3f Preintegrated::GetUpdatedDeltaRotation()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return NormalizeRotation(dR * Sophus::SO3f::exp(JRg*db.head(3)).matrix());
}

Eigen::Vector3f Preintegrated::GetUpdatedDeltaVelocity()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dV + JVg * db.head(3) + JVa * db.tail(3);
}

Eigen::Vector3f Preintegrated::GetUpdatedDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP + JPg*db.head(3) + JPa*db.tail(3);
}

Eigen::Matrix3f Preintegrated::GetOriginalDeltaRotation() {
    std::unique_lock<std::mutex> lock(mMutex);
    return dR;
}

Eigen::Vector3f Preintegrated::GetOriginalDeltaVelocity() {
    std::unique_lock<std::mutex> lock(mMutex);
    return dV;
}

Eigen::Vector3f Preintegrated::GetOriginalDeltaPosition()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return dP;
}

Bias Preintegrated::GetOriginalBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return b;
}

Bias Preintegrated::GetUpdatedBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return bu;
}

Eigen::Matrix<float,6,1> Preintegrated::GetDeltaBias()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return db;
}

void Bias::CopyFrom(Bias &b)
{
    bax = b.bax;
    bay = b.bay;
    baz = b.baz;
    bwx = b.bwx;
    bwy = b.bwy;
    bwz = b.bwz;
}

std::ostream& operator<< (std::ostream &out, const Bias &b)
{
    if(b.bwx>0)
        out << " ";
    out << b.bwx << ",";
    if(b.bwy>0)
        out << " ";
    out << b.bwy << ",";
    if(b.bwz>0)
        out << " ";
    out << b.bwz << ",";
    if(b.bax>0)
        out << " ";
    out << b.bax << ",";
    if(b.bay>0)
        out << " ";
    out << b.bay << ",";
    if(b.baz>0)
        out << " ";
    out << b.baz;

    return out;
}

void Calib::Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw) {
    mbIsSet = true;
    const float ng2 = ng*ng;
    const float na2 = na*na;
    const float ngw2 = ngw*ngw;
    const float naw2 = naw*naw;

    // Sophus/Eigen
    mTbc = sophTbc;
    mTcb = mTbc.inverse();
    Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
    CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
}

Calib::Calib(const Calib &calib)
{
    mbIsSet = calib.mbIsSet;
    // Sophus/Eigen parameters
    mTbc = calib.mTbc;
    mTcb = calib.mTcb;
    Cov = calib.Cov;
    CovWalk = calib.CovWalk;
}

} //namespace IMU

} //namespace ORB_SLAM2
