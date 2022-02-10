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


#include "Optimizer.h"


#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "G2oTypes.h"
#include "Converter.h"

#include<mutex>

#include "OptimizableTypes.h"


namespace ORB_SLAM3
{
bool sortByVal(const pair<MapPoint*, int> &a, const pair<MapPoint*, int> &b)
{
    return (a.second < b.second);
}

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs,vpMP,nIterations,pbStopFlag, nLoopKF, bRobust);
}


void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                 int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    Map* pMap = vpKFs[0]->GetMap();

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    const int nExpectedSize = (vpKFs.size())*vpMP.size();

    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);


    // Set KeyFrame vertices

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKF->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId==pMap->GetInitKFid());
        optimizer.addVertex(vSE3);
        if(pKF->mnId>maxKFid)
            maxKFid=pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        MapPoint* pMP = vpMP[i];
        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        const int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

       const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid)
                continue;
            if(optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
                continue;
            nEdges++;

            const int leftIndex = get<0>(mit->second);

            if(leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)]<0)
            {
                const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->pCamera = pKF->mpCamera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMP);
            }
            else if(leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0) //Stereo observation
            {
                const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMP);
            }

            if(pKF->mpCamera2){
                int rightIndex = get<1>(mit->second);

                if(rightIndex != -1 && rightIndex < pKF->mvKeysRight.size()){
                    rightIndex -= pKF->NLeft;

                    Eigen::Matrix<double,2,1> obs;
                    cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
                    obs << kp.pt.x, kp.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);

                    Sophus::SE3f Trl = pKF-> GetRelativePoseTrl();
                    e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                    e->pCamera = pKF->mpCamera2;

                    optimizer.addEdge(e);
                    vpEdgesBody.push_back(e);
                    vpEdgeKFBody.push_back(pKF);
                    vpMapPointEdgeBody.push_back(pMP);
                }
            }
        }



        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

    // Recover optimized data
    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));

        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==pMap->GetOriginKF()->mnId)
        {
            pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>()));
        }
        else
        {
            pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(),SE3quat.translation()).cast<float>();
            pKF->mnBAGlobalForKF = nLoopKF;

            Sophus::SE3f mTwc = pKF->GetPoseInverse();
            Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
            Eigen::Vector3f vector_dist =  mTcGBA_c.translation();
            double dist = vector_dist.norm();
            if(dist > 1)
            {
                int numMonoBadPoints = 0, numMonoOptPoints = 0;
                int numStereoBadPoints = 0, numStereoOptPoints = 0;
                vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;

                for(size_t i2=0, iend=vpEdgesMono.size(); i2<iend;i2++)
                {
                    ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i2];
                    MapPoint* pMP = vpMapPointEdgeMono[i2];
                    KeyFrame* pKFedge = vpEdgeKFMono[i2];

                    if(pKF != pKFedge)
                    {
                        continue;
                    }

                    if(pMP->isBad())
                        continue;

                    if(e->chi2()>5.991 || !e->isDepthPositive())
                    {
                        numMonoBadPoints++;

                    }
                    else
                    {
                        numMonoOptPoints++;
                        vpMonoMPsOpt.push_back(pMP);
                    }

                }

                for(size_t i2=0, iend=vpEdgesStereo.size(); i2<iend;i2++)
                {
                    g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i2];
                    MapPoint* pMP = vpMapPointEdgeStereo[i2];
                    KeyFrame* pKFedge = vpEdgeKFMono[i2];

                    if(pKF != pKFedge)
                    {
                        continue;
                    }

                    if(pMP->isBad())
                        continue;

                    if(e->chi2()>7.815 || !e->isDepthPositive())
                    {
                        numStereoBadPoints++;
                    }
                    else
                    {
                        numStereoOptPoints++;
                        vpStereoMPsOpt.push_back(pMP);
                    }
                }
            }
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));

        if(nLoopKF==pMap->GetOriginKF()->mnId)
        {
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA = vPoint->estimate().cast<float>();
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }
}

void Optimizer::FullInertialBA(Map *pMap, int its, const bool bFixLocal, const long unsigned int nLoopId, bool *pbStopFlag, bool bInit, float priorG, float priorA, Eigen::VectorXd *vSingVal, bool *bHess)
{
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int nNonFixed = 0;

    // Set KeyFrame vertices
    KeyFrame* pIncKF;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        pIncKF=pKFi;
        bool bFixed = false;
        if(bFixLocal)
        {
            bFixed = (pKFi->mnBALocalForKF>=(maxKFid-1)) || (pKFi->mnBAFixedForKF>=(maxKFid-1));
            if(!bFixed)
                nNonFixed++;
            VP->setFixed(bFixed);
        }
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(bFixed);
            optimizer.addVertex(VV);
            if (!bInit)
            {
                VertexGyroBias* VG = new VertexGyroBias(pKFi);
                VG->setId(maxKFid+3*(pKFi->mnId)+2);
                VG->setFixed(bFixed);
                optimizer.addVertex(VG);
                VertexAccBias* VA = new VertexAccBias(pKFi);
                VA->setId(maxKFid+3*(pKFi->mnId)+3);
                VA->setFixed(bFixed);
                optimizer.addVertex(VA);
            }
        }
    }

    if (bInit)
    {
        VertexGyroBias* VG = new VertexGyroBias(pIncKF);
        VG->setId(4*maxKFid+2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pIncKF);
        VA->setId(4*maxKFid+3);
        VA->setFixed(false);
        optimizer.addVertex(VA);
    }

    if(bFixLocal)
    {
        if(nNonFixed<3)
            return;
    }

    // IMU links
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(!pKFi->mPrevKF)
        {
            Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!", Verbose::VERBOSITY_NORMAL);
            continue;
        }

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
            if(pKFi->bImu && pKFi->mPrevKF->bImu)
            {
                pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
                g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);

                g2o::HyperGraph::Vertex* VG1;
                g2o::HyperGraph::Vertex* VA1;
                g2o::HyperGraph::Vertex* VG2;
                g2o::HyperGraph::Vertex* VA2;
                if (!bInit)
                {
                    VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
                    VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
                    VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
                    VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);
                }
                else
                {
                    VG1 = optimizer.vertex(4*maxKFid+2);
                    VA1 = optimizer.vertex(4*maxKFid+3);
                }

                g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);

                if (!bInit)
                {
                    if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
                    {
                        cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;
                        continue;
                    }
                }
                else
                {
                    if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2)
                    {
                        cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<endl;
                        continue;
                    }
                }

                EdgeInertial* ei = new EdgeInertial(pKFi->mpImuPreintegrated);
                ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
                ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
                ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
                ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
                ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
                ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rki);
                rki->setDelta(sqrt(16.92));

                optimizer.addEdge(ei);

                if (!bInit)
                {
                    EdgeGyroRW* egr= new EdgeGyroRW();
                    egr->setVertex(0,VG1);
                    egr->setVertex(1,VG2);
                    Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
                    egr->setInformation(InfoG);
                    egr->computeError();
                    optimizer.addEdge(egr);

                    EdgeAccRW* ear = new EdgeAccRW();
                    ear->setVertex(0,VA1);
                    ear->setVertex(1,VA2);
                    Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
                    ear->setInformation(InfoA);
                    ear->computeError();
                    optimizer.addEdge(ear);
                }
            }
            else
                cout << pKFi->mnId << " or " << pKFi->mPrevKF->mnId << " no imu" << endl;
        }
    }

    if (bInit)
    {
        g2o::HyperGraph::Vertex* VG = optimizer.vertex(4*maxKFid+2);
        g2o::HyperGraph::Vertex* VA = optimizer.vertex(4*maxKFid+3);

        // Add prior to comon biases
        Eigen::Vector3f bprior;
        bprior.setZero();

        EdgePriorAcc* epa = new EdgePriorAcc(bprior);
        epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
        double infoPriorA = priorA; //
        epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
        optimizer.addEdge(epa);

        EdgePriorGyro* epg = new EdgePriorGyro(bprior);
        epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
        double infoPriorG = priorG; //
        epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
        optimizer.addEdge(epg);
    }

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    const unsigned long iniMPid = maxKFid*5;

    vector<bool> vbNotIncludedMP(vpMPs.size(),false);

    for(size_t i=0; i<vpMPs.size(); i++)
    {
        MapPoint* pMP = vpMPs[i];
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();


        bool bAllFixed = true;

        //Set edges
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnId>maxKFid)
                continue;

            if(!pKFi->isBad())
            {
                const int leftIndex = get<0>(mit->second);
                cv::KeyPoint kpUn;

                if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]<0) // Monocular observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if(bAllFixed)
                        if(!VP->fixed())
                            bAllFixed=false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                }
                else if(leftIndex != -1 && pKFi->mvuRight[leftIndex] >= 0) // stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    if(bAllFixed)
                        if(!VP->fixed())
                            bAllFixed=false;

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, VP);
                    e->setMeasurement(obs);
                    const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                }

                if(pKFi->mpCamera2){ // Monocular right observation
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size()){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        kpUn = pKFi->mvKeysRight[rightIndex];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMono *e = new EdgeMono(1);

                        g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                        if(bAllFixed)
                            if(!VP->fixed())
                                bAllFixed=false;

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, VP);
                        e->setMeasurement(obs);
                        const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                    }
                }
            }
        }

        if(bAllFixed)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;


    optimizer.initializeOptimization();
    optimizer.optimize(its);


    // Recover optimized data
    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        if(nLoopId==0)
        {
            Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
            pKFi->SetPose(Tcw);
        }
        else
        {
            pKFi->mTcwGBA = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(),VP->estimate().tcw[0].cast<float>());
            pKFi->mnBAGlobalForKF = nLoopId;

        }
        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            if(nLoopId==0)
            {
                pKFi->SetVelocity(VV->estimate().cast<float>());
            }
            else
            {
                pKFi->mVwbGBA = VV->estimate().cast<float>();
            }

            VertexGyroBias* VG;
            VertexAccBias* VA;
            if (!bInit)
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            }
            else
            {
                VG = static_cast<VertexGyroBias*>(optimizer.vertex(4*maxKFid+2));
                VA = static_cast<VertexAccBias*>(optimizer.vertex(4*maxKFid+3));
            }

            Vector6d vb;
            vb << VG->estimate(), VA->estimate();
            IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);
            if(nLoopId==0)
            {
                pKFi->SetNewBias(b);
            }
            else
            {
                pKFi->mBiasGBA = b;
            }
        }
    }

    //Points
    for(size_t i=0; i<vpMPs.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        MapPoint* pMP = vpMPs[i];
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));

        if(nLoopId==0)
        {
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA = vPoint->estimate().cast<float>();
            pMP->mnBAGlobalForKF = nLoopId;
        }

    }

    pMap->IncreaseChangeIndex();
}

/**
 * @brief 位姿优化，纯视觉\恒速模型\参考关键帧\重定位跟踪 使用。优化目标：单帧的位姿
 * @param pFrame 待优化的帧
 * */
int Optimizer::PoseOptimization(Frame *pFrame)
{

    // step 1 创建线性求解器 linearSolver
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    // step 2 创建块求解器 BlockSolver
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    // step 3 创建总求解器 Solver
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    // step 4 设置稀疏优化器 SparseOptimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);  //用前面定义的求解器作为求解方法

    int nInitialCorrespondences=0;

    // step 5 定义图的顶点
    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pFrame->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *> vpEdgesMono_FHR;
    vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
    vpEdgesMono.reserve(N);
    vpEdgesMono_FHR.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeRight.reserve(N);

    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815);

    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    // step 6 定义图的边
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = pFrame->mvpMapPoints[i];
        if(pMP)
        {
            //Conventional SLAM
            if(!pFrame->mpCamera2){
                // Monocular observation
                if(pFrame->mvuRight[i]<0)
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->pCamera = pFrame->mpCamera;
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                else  // Stereo observation
                {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,3,1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    const float &kp_ur = pFrame->mvuRight[i];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaStereo);

                    e->fx = pFrame->fx;
                    e->fy = pFrame->fy;
                    e->cx = pFrame->cx;
                    e->cy = pFrame->cy;
                    e->bf = pFrame->mbf;
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }
            }
            //SLAM with respect a rigid body
            else{
                nInitialCorrespondences++;

                cv::KeyPoint kpUn;

                if (i < pFrame->Nleft) {    //Left camera observation
                    kpUn = pFrame->mvKeys[i];

                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->pCamera = pFrame->mpCamera;
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                else {
                    kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    pFrame->mvbOutlier[i] = false;

                    ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->pCamera = pFrame->mpCamera2;
                    e->Xw = pMP->GetWorldPos().cast<double>();

                    e->mTrl = g2o::SE3Quat(pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(), pFrame->GetRelativePoseTrl().translation().cast<double>());

                    optimizer.addEdge(e);

                    vpEdgesMono_FHR.push_back(e);
                    vnIndexEdgeRight.push_back(i);
                }
            }
        }
    }
    }

    if(nInitialCorrespondences<3)
        return 0;

    // step 7 设置优化参数和迭代次数
    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4]={10,10,10,10};    

    int nBad=0;
    for(size_t it=0; it<4; it++)
    {
        Tcw = pFrame->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        // 优化结束,开始遍历参与优化的每一条边
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {                
                pFrame->mvbOutlier[idx]=true;  // 大于设定的值, 设置为外点
                e->setLevel(1);  // 表示不优化
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);  //前两次优化使用核函数,之后不使用
        }

        for(size_t i=0, iend=vpEdgesMono_FHR.size(); i<iend; i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

            const size_t idx = vnIndexEdgeRight[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {                
                e->setLevel(0);
                pFrame->mvbOutlier[idx]=false;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        if(optimizer.edges().size()<10)
            break;
    }    

    // Recover optimized pose and return number of inliers
    // 得到优化后当前帧的位姿
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
            SE3quat_recov.translation().cast<float>());
    pFrame->SetPose(pose);

    return nInitialCorrespondences-nBad;   // 返回内点数目
}
// 局部BA
void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    Map* pCurrentMap = pKF->GetMap();

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    num_fixedKF = 0;
    list<MapPoint*> lLocalMapPoints;
    set<MapPoint*> sNumObsMP;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(pKFi->mnId==pMap->GetInitKFid())
        {
            num_fixedKF = 1;
        }
        vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                {

                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
                }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId )
            {                
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    num_fixedKF = lFixedCameras.size() + num_fixedKF;


    if(num_fixedKF == 0)
    {
        Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
        return;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    if (pMap->IsInertial())
        solver->setUserLambdaInit(100.0);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // DEBUG LBA
    pCurrentMap->msOptKFs.clear();
    pCurrentMap->msFixedKFs.clear();

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId==pMap->GetInitKFid());
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
        // DEBUG LBA
        pCurrentMap->msOptKFs.insert(pKFi->mnId);
    }
    num_OptKF = lLocalKeyFrames.size();

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
        // DEBUG LBA
        pCurrentMap->msFixedKFs.insert(pKFi->mnId);
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    int nPoints = 0;

    int nEdges = 0;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        nPoints++;

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int leftIndex = get<0>(mit->second);

                // Monocular observation
                if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]<0)
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->pCamera = pKFi->mpCamera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    nEdges++;
                }
                else if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]>=0)// Stereo observation
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    nEdges++;
                }

                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        Sophus::SE3f Trl = pKFi-> GetRelativePoseTrl();
                        e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                        e->pCamera = pKFi->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKFi);
                        vpMapPointEdgeBody.push_back(pMP);

                        nEdges++;
                    }
                }
            }
        }
    }
    num_edges = nEdges;

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesBody.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesBody.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
        MapPoint* pMP = vpMapPointEdgeBody[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFBody[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }


    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data
    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
        pKFi->SetPose(Tiw);
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}


void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
{   
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    vector<Eigen::Vector3d> vZvectors(nMaxKFid+1); // For debugging
    Eigen::Vector3d z_vec;
    z_vec << 0.0, 0.0, 1.0;

    const int minFeat = 100;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF->mnId==pMap->GetInitKFid())
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);
        vZvectors[nIDi]=vScw[nIDi].rotation()*z_vec; // For debugging

        vpVertices[nIDi]=VSim3;
    }


    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    int count_loop = 0;
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);
            count_loop++;
            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Swi;

        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Sjw;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Slw;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[pLKF->mnId];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) /*&& !sLoopEdges.count(pKFn)*/)
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Snw;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[pKFn->mnId];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }

        // Inertial edges if inertial
        if(pKF->bImu && pKF->mPrevKF)
        {
            g2o::Sim3 Spw;
            LoopClosing::KeyFrameAndPose::const_iterator itp = NonCorrectedSim3.find(pKF->mPrevKF);
            if(itp!=NonCorrectedSim3.end())
                Spw = itp->second;
            else
                Spw = vScw[pKF->mPrevKF->mnId];

            g2o::Sim3 Spi = Spw * Swi;
            g2o::EdgeSim3* ep = new g2o::EdgeSim3();
            ep->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mPrevKF->mnId)));
            ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            ep->setMeasurement(Spi);
            ep->information() = matLambda;
            optimizer.addEdge(ep);
        }
    }


    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);
    optimizer.computeActiveErrors();
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        double s = CorrectedSiw.scale();

        Sophus::SE3f Tiw(CorrectedSiw.rotation().cast<float>(), CorrectedSiw.translation().cast<float>() / s);
        pKFi->SetPose(Tiw);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mnCorrectedByKF==pCurKF->mnId)
        {
            nIDr = pMP->mnCorrectedReference;
        }
        else
        {
            KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mnId;
        }


        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double,3,1> eigP3Dw = pMP->GetWorldPos().cast<double>();
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
        pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

        pMP->UpdateNormalAndDepth();
    }

    // TODO Check this changeindex
    pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
                                       vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs)
{
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpFixedKFs.size()) + " KFs fixed in the merged map", Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpFixedCorrectedKFs.size()) + " KFs fixed in the old map", Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpNonFixedKFs.size()) + " KFs non-fixed in the merged map", Verbose::VERBOSITY_DEBUG);
    Verbose::PrintMess("Opt_Essential: There are " + to_string(vpNonCorrectedMPs.size()) + " MPs non-corrected in the merged map", Verbose::VERBOSITY_DEBUG);

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    Map* pMap = pCurKF->GetMap();
    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    vector<bool> vpGoodPose(nMaxKFid+1);
    vector<bool> vpBadPose(nMaxKFid+1);

    const int minFeat = 100;

    for(KeyFrame* pKFi : vpFixedKFs)
    {
        if(pKFi->isBad())
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);

        vCorrectedSwc[nIDi]=Siw.inverse();
        VSim3->setEstimate(Siw);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = true;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;

        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi] = false;
    }
    Verbose::PrintMess("Opt_Essential: vpFixedKFs loaded", Verbose::VERBOSITY_DEBUG);

    set<unsigned long> sIdKF;
    for(KeyFrame* pKFi : vpFixedCorrectedKFs)
    {
        if(pKFi->isBad())
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const int nIDi = pKFi->mnId;

        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);

        vCorrectedSwc[nIDi]=Siw.inverse();
        VSim3->setEstimate(Siw);

        Sophus::SE3d Tcw_bef = pKFi->mTcwBefMerge.cast<double>();
        vScw[nIDi] = g2o::Sim3(Tcw_bef.unit_quaternion(),Tcw_bef.translation(),1.0);

        VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;

        sIdKF.insert(nIDi);

        vpGoodPose[nIDi] = true;
        vpBadPose[nIDi] = true;
    }

    for(KeyFrame* pKFi : vpNonFixedKFs)
    {
        if(pKFi->isBad())
            continue;

        const int nIDi = pKFi->mnId;

        if(sIdKF.count(nIDi)) // It has already added in the corrected merge KFs
            continue;

        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        Sophus::SE3d Tcw = pKFi->GetPose().cast<double>();
        g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);

        vScw[nIDi] = Siw;
        VSim3->setEstimate(Siw);

        VSim3->setFixed(false);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;

        sIdKF.insert(nIDi);

        vpGoodPose[nIDi] = false;
        vpBadPose[nIDi] = true;
    }

    vector<KeyFrame*> vpKFs;
    vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() + vpNonFixedKFs.size());
    vpKFs.insert(vpKFs.end(),vpFixedKFs.begin(),vpFixedKFs.end());
    vpKFs.insert(vpKFs.end(),vpFixedCorrectedKFs.begin(),vpFixedCorrectedKFs.end());
    vpKFs.insert(vpKFs.end(),vpNonFixedKFs.begin(),vpNonFixedKFs.end());
    set<KeyFrame*> spKFs(vpKFs.begin(), vpKFs.end());

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    for(KeyFrame* pKFi : vpKFs)
    {
        int num_connections = 0;
        const int nIDi = pKFi->mnId;

        g2o::Sim3 correctedSwi;
        g2o::Sim3 Swi;

        if(vpGoodPose[nIDi])
            correctedSwi = vCorrectedSwc[nIDi];
        if(vpBadPose[nIDi])
            Swi = vScw[nIDi].inverse();

        KeyFrame* pParentKFi = pKFi->GetParent();

        // Spanning tree edge
        if(pParentKFi && spKFs.find(pParentKFi) != spKFs.end())
        {
            int nIDj = pParentKFi->mnId;

            g2o::Sim3 Sjw;
            bool bHasRelation = false;

            if(vpGoodPose[nIDi] && vpGoodPose[nIDj])
            {
                Sjw = vCorrectedSwc[nIDj].inverse();
                bHasRelation = true;
            }
            else if(vpBadPose[nIDi] && vpBadPose[nIDj])
            {
                Sjw = vScw[nIDj];
                bHasRelation = true;
            }

            if(bHasRelation)
            {
                g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3* e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                e->setMeasurement(Sji);

                e->information() = matLambda;
                optimizer.addEdge(e);
                num_connections++;
            }

        }

        // Loop edges
        const set<KeyFrame*> sLoopEdges = pKFi->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(spKFs.find(pLKF) != spKFs.end() && pLKF->mnId<pKFi->mnId)
            {
                g2o::Sim3 Slw;
                bool bHasRelation = false;

                if(vpGoodPose[nIDi] && vpGoodPose[pLKF->mnId])
                {
                    Slw = vCorrectedSwc[pLKF->mnId].inverse();
                    bHasRelation = true;
                }
                else if(vpBadPose[nIDi] && vpBadPose[pLKF->mnId])
                {
                    Slw = vScw[pLKF->mnId];
                    bHasRelation = true;
                }


                if(bHasRelation)
                {
                    g2o::Sim3 Sli = Slw * Swi;
                    g2o::EdgeSim3* el = new g2o::EdgeSim3();
                    el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                    num_connections++;
                }
            }
        }

        // Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKFi->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKFi && !pKFi->hasChild(pKFn) && !sLoopEdges.count(pKFn) && spKFs.find(pKFn) != spKFs.end())
            {
                if(!pKFn->isBad() && pKFn->mnId<pKFi->mnId)
                {

                    g2o::Sim3 Snw =  vScw[pKFn->mnId];
                    bool bHasRelation = false;

                    if(vpGoodPose[nIDi] && vpGoodPose[pKFn->mnId])
                    {
                        Snw = vCorrectedSwc[pKFn->mnId].inverse();
                        bHasRelation = true;
                    }
                    else if(vpBadPose[nIDi] && vpBadPose[pKFn->mnId])
                    {
                        Snw = vScw[pKFn->mnId];
                        bHasRelation = true;
                    }

                    if(bHasRelation)
                    {
                        g2o::Sim3 Sni = Snw * Swi;

                        g2o::EdgeSim3* en = new g2o::EdgeSim3();
                        en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                        en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                        num_connections++;
                    }
                }
            }
        }

        if(num_connections == 0 )
        {
            Verbose::PrintMess("Opt_Essential: KF " + to_string(pKFi->mnId) + " has 0 connections", Verbose::VERBOSITY_DEBUG);
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(KeyFrame* pKFi : vpNonFixedKFs)
    {
        if(pKFi->isBad())
            continue;

        const int nIDi = pKFi->mnId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        double s = CorrectedSiw.scale();
        Sophus::SE3d Tiw(CorrectedSiw.rotation(),CorrectedSiw.translation() / s);

        pKFi->mTcwBefMerge = pKFi->GetPose();
        pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
        pKFi->SetPose(Tiw.cast<float>());
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(MapPoint* pMPi : vpNonCorrectedMPs)
    {
        if(pMPi->isBad())
            continue;

        KeyFrame* pRefKF = pMPi->GetReferenceKeyFrame();
        while(pRefKF->isBad())
        {
            if(!pRefKF)
            {
                Verbose::PrintMess("MP " + to_string(pMPi->mnId) + " without a valid reference KF", Verbose::VERBOSITY_DEBUG);
                break;
            }

            pMPi->EraseObservation(pRefKF);
            pRefKF = pMPi->GetReferenceKeyFrame();
        }

        if(vpBadPose[pRefKF->mnId])
        {
            Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
            Sophus::SE3f Twr = pRefKF->GetPoseInverse();

            Eigen::Vector3f eigCorrectedP3Dw = Twr * TNonCorrectedwr.inverse() * pMPi->GetWorldPos();
            pMPi->SetWorldPos(eigCorrectedP3Dw);

            pMPi->UpdateNormalAndDepth();
        }
        else
        {
            cout << "ERROR: MapPoint has a reference KF from another map" << endl;
        }

    }
}

int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2,
                            const bool bFixScale, Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Camera poses
    const Eigen::Matrix3f R1w = pKF1->GetRotation();
    const Eigen::Vector3f t1w = pKF1->GetTranslation();
    const Eigen::Matrix3f R2w = pKF2->GetRotation();
    const Eigen::Vector3f t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    ORB_SLAM3::VertexSim3Expmap * vSim3 = new ORB_SLAM3::VertexSim3Expmap();
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->pCamera1 = pKF1->mpCamera;
    vSim3->pCamera2 = pKF2->mpCamera;
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<ORB_SLAM3::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;
    vector<bool> vbIsInKF2;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);
    vbIsInKF2.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;
    int nBadMPs = 0;
    int nInKF2 = 0;
    int nOutKF2 = 0;
    int nMatchWithoutMP = 0;

    vector<int> vIdsOnlyInKF2;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        MapPoint* pMP1 = vpMapPoints1[i];
        MapPoint* pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));

        Eigen::Vector3f P3D1c;
        Eigen::Vector3f P3D2c;

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad())
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
                P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(P3D1c.cast<double>());
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast<double>());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
            {
                nBadMPs++;
                continue;
            }
        }
        else
        {
            nMatchWithoutMP++;

            //TODO The 3D position in KF1 doesn't exist
            if(!pMP2->isBad())
            {
                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
                P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(P3D2c.cast<double>());
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);

                vIdsOnlyInKF2.push_back(id2);
            }
            continue;
        }

        if(i2<0 && !bAllPoints)
        {
            Verbose::PrintMess("    Remove point -> i2: " + to_string(i2) + "; bAllPoints: " + to_string(bAllPoints), Verbose::VERBOSITY_DEBUG);
            continue;
        }

        if(P3D2c(2) < 0)
        {
            Verbose::PrintMess("Sim3: Z coordinate is negative", Verbose::VERBOSITY_DEBUG);
            continue;
        }

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();

        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        cv::KeyPoint kpUn2;
        bool inKF2;
        if(i2 >= 0)
        {
            kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;
            inKF2 = true;

            nInKF2++;
        }
        else
        {
            float invz = 1/P3D2c(2);
            float x = P3D2c(0)*invz;
            float y = P3D2c(1)*invz;

            obs2 << x, y;
            kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

            inKF2 = false;
            nOutKF2++;
        }

        ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);

        vbIsInKF2.push_back(inKF2);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    int nBadOutKF2 = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2 || e21->chi2()>th2)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;

            if(!vbIsInKF2[i])
            {
                nBadOutKF2++;
            }
            continue;
        }

        //Check if remove the robust adjustment improve the result
        e12->setRobustKernel(0);
        e21->setRobustKernel(0);
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers
    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        e12->computeError();
        e21->computeError();

        if(e12->chi2()>th2 || e21->chi2()>th2){
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<MapPoint*>(NULL);
        }
        else{
            nIn++;
        }
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}

void Optimizer::LocalInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit)
{
    Map* pCurrentMap = pKF->GetMap();

    int maxOpt=10;
    int opt_it=10;
    if(bLarge)
    {
        maxOpt=25;
        opt_it=4;
    }
    const int Nd = std::min((int)pCurrentMap->KeyFramesInMap()-2,maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    vector<KeyFrame*> vpOptimizableKFs;
    const vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
    list<KeyFrame*> lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;
    for(int i=1; i<Nd; i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by temporal optimizable keyframes
    list<MapPoint*> lLocalMapPoints;
    for(int i=0; i<N; i++)
    {
        vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    // Step 3. 固定一帧，为vpOptimizableKFs中最早的那一关键帧的上一关键帧，如果没有上一关键帧了就用最早的那一帧，毕竟目前得到的地图虽然有尺度但并不是绝对的位置
    list<KeyFrame*> lFixedKeyFrames;
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF=pKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->mnBALocalForKF=0;
        vpOptimizableKFs.back()->mnBAFixedForKF=pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    // 4. 做了一系列操作发现最后lpOptVisKFs为空。这段应该是调试遗留代码，如果实现的话其实就是把共视图中在前面没有加过的关键帧们加进来，
    // 但作者可能发现之前就把共视图的全部帧加进来了，也有可能发现优化的效果不好浪费时间
    // 获得与当前关键帧有共视关系的一些关键帧，大于15个点，排序为从小到大
    const int maxCovKF = 0;
    for(int i=0, iend=vpNeighsKFs.size(); i<iend; i++)
    {
        if(lpOptVisKFs.size() >= maxCovKF)
            break;

        KeyFrame* pKFi = vpNeighsKFs[i];
        if(pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
            continue;
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        {
            lpOptVisKFs.push_back(pKFi);

            vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
            for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
            {
                MapPoint* pMP = *vit;
                if(pMP)
                    if(!pMP->isBad())
                        if(pMP->mnBALocalForKF!=pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF=pKF->mnId;
                        }
            }
        }
    }

    // Fixed KFs which are not covisible optimizable
    // 5. 将所有mp点对应的关键帧（除了前面加过的）放入到固定组里面，后面优化时不改变
    const int maxFixKF = 200;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,tuple<int,int>> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if(lFixedKeyFrames.size()>=maxFixKF)
            break;
    }

    bool bNonFixed = (lFixedKeyFrames.size() == 0);

    // Setup optimizer
    // 6. 构造优化器，正式开始优化
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    if(bLarge)
    {
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e-2); // to avoid iterating for finding optimal lambda
        optimizer.setAlgorithm(solver);
    }
    else
    {
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        solver->setUserLambdaInit(1e0);
        optimizer.setAlgorithm(solver);
    }


    // Set Local temporal KeyFrame vertices
    // 7. 建立关于关键帧的节点，其中包括，位姿，速度，以及两个偏置
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Local visual KeyFrame vertices
    // 8. 建立关于共视关键帧的节点，但这里为空
    for(list<KeyFrame*>::iterator it=lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it!=itEnd; it++)
    {
        KeyFrame* pKFi = *it;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
    }

    // Set Fixed KeyFrame vertices
    // 9. 建立关于固定关键帧的节点，其中包括，位姿，速度，以及两个偏置
    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if(pKFi->bImu) // This should be done only for keyframe just before temporal window
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
    }

    // Create intertial constraints
    // 10. 建立惯性边，没有imu跳过
    vector<EdgeInertial*> vei(N,(EdgeInertial*)NULL);
    vector<EdgeGyroRW*> vegr(N,(EdgeGyroRW*)NULL);
    vector<EdgeAccRW*> vear(N,(EdgeAccRW*)NULL);

    for(int i=0;i<N;i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        if(!pKFi->mPrevKF)
        {
            cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << endl;
            continue;
        }
        if(pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
        {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
            g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
            g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);

            if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
            {
                cerr << "Error " << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            // 到最早的一个可优化的关键帧或者地图没有ba2时添加鲁棒核函数
            if(i==N-1 || bRecInit)
            {
                // All inertial residuals are included without robust cost function, but not that one linking the
                // last optimizable keyframe inside of the local window and the first fixed keyframe out. The
                // information matrix for this measurement is also downweighted. This is done to avoid accumulating
                // error due to fixing variables.
                // 所有惯性残差都没有鲁棒核，但不包括窗口内最早一个可优化关键帧与第一个固定关键帧链接起来的惯性残差。
                // 该度量的信息矩阵也被降权。这样做是为了避免由于固定变量而累积错误。
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                if(i==N-1)
                    vei[i]->setInformation(vei[i]->information()*1e-2);
                rki->setDelta(sqrt(16.92));
            }
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0,VG1);
            vegr[i]->setVertex(1,VG2);
            Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0,VA1);
            vear[i]->setVertex(1,VA2);
            Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
            vear[i]->setInformation(InfoA);           

            optimizer.addEdge(vear[i]);
        }
        else
            cout << "ERROR building inertial edge" << endl;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (N+lFixedKeyFrames.size())*lLocalMapPoints.size();

    // Mono
    vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);


    // 11. 建立视觉边
    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const unsigned long iniMPid = maxKFid*5;

    map<int,int> mVisEdges;
    for(int i=0;i<N;i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];
        mVisEdges[pKFi->mnId] = 0;
    }
    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        mVisEdges[(*lit)->mnId] = 0;
    }
    // 遍历地图点
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        // Create visual constraints
        // bi
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
                continue;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int leftIndex = get<0>(mit->second);

                cv::KeyPoint kpUn;

                // Monocular left observation
                if(leftIndex != -1 && pKFi->mvuRight[leftIndex]<0)
                {
                    mVisEdges[pKFi->mnId]++;  // notice KF id, numbers of observations

                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));  // map point id
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId))); // keyframe id
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                // Stereo-observation
                else if(leftIndex != -1)// Stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    mVisEdges[pKFi->mnId]++;

                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                // Monocular right observation
                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;
                        mVisEdges[pKFi->mnId]++;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        EdgeMono* e = new EdgeMono(1);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                }
            }
        }
    }

    //cout << "Total map points: " << lLocalMapPoints.size() << endl;
    // todo 报错点  debug会报错先注释掉
    /*for(map<int,int>::iterator mit=mVisEdges.begin(), mend=mVisEdges.end(); mit!=mend; mit++)
    {
        assert(mit->second>=3);  // each id , observations >= 3
    }*/

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float err = optimizer.activeRobustChi2();
    optimizer.optimize(opt_it); // Originally to 2
    float err_end = optimizer.activeRobustChi2();
    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        EdgeMono* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];
        bool bClose = pMP->mTrackDepth<10.f;

        if(pMP->isBad())
            continue;

        if((e->chi2()>chi2Mono2 && !bClose) || (e->chi2()>1.5f*chi2Mono2 && bClose) || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }


    // Stereo
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>chi2Stereo2)
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);


    // TODO: Some convergence problems have been detected here
    if((2*err < err_end || isnan(err) || isnan(err_end)) && !bLarge) //bGN)
    {
        cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
        return;
    }



    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
        (*lit)->mnBAFixedForKF = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF=0;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));

        }
    }

    // Local visual KeyFrame
    for(list<KeyFrame*>::iterator it=lpOptVisKFs.begin(), itEnd = lpOptVisKFs.end(); it!=itEnd; it++)
    {
        KeyFrame* pKFi = *it;
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);
        pKFi->mnBALocalForKF=0;
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}
/**
 * @brief PoseInertialOptimizationLastFrame 中使用 Marginalize(H, 0, 14);
 * 使用舒尔补的方式边缘化海森矩阵，边缘化。
 * 列数 6            3                    3                            3                         6           3             3              3
 * --------------------------------------------------------------------------------------------------------------------------------------------------- 行数
 * |  Jp1.t * Jp1  Jp1.t * Jv1         Jp1.t * Jg1                 Jp1.t * Ja1            |  Jp1.t * Jp2  Jp1.t * Jv2        0              0        |  6
 * |  Jv1.t * Jp1  Jv1.t * Jv1         Jv1.t * Jg1                 Jv1.t * Ja1            |  Jv1.t * Jp2  Jv1.t * Jv2        0              0        |  3
 * |  Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1 + Jgr1.t * Jgr1        Jg1.t * Ja1            |  Jg1.t * Jp2  Jg1.t * Jv2  Jgr1.t * Jgr2        0        |  3
 * |  Ja1.t * Jp1  Ja1.t * Jv1         Ja1.t * Jg1           Ja1.t * Ja1 + Jar1.t * Jar1  |  Ja1.t * Jp2  Ja1.t * Jv2  Jar1.t * Jar2        0        |  3
 * |--------------------------------------------------------------------------------------------------------------------------------------------------
 * |  Jp2.t * Jp1  Jp2.t * Jv1         Jp2.t * Jg1                 Jp2.t * Ja1            |  Jp2.t * Jp2  Jp2.t * Jv2        0              0        |  6
 * |  Jv2.t * Jp1  Jv2.t * Jv1         Jv2.t * Jg1                 Jv2.t * Ja1            |  Jv2.t * Jp2  Jv2.t * Jv2        0              0        |  3
 * |      0            0              Jgr2.t * Jgr1                      0                |        0           0       Jgr2.t * Jgr2        0        |  3
 * |      0            0                    0                     Jar2.t * Jar1           |        0           0             0        Jar2.t * Jar2  |  3
 * ---------------------------------------------------------------------------------------------------------------------------------------------------
 * @param H 30*30的海森矩阵
 * @param start 开始位置
 * @param end 结束位置
 */
Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end)
{
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    const int a = start;
    // Size of block to marginalize
    const int b = end-start+1;
    // Size of block after block to marginalize
    const int c = H.cols() - (end+1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(),H.cols());
    if(a>0)
    {
        Hn.block(0,0,a,a) = H.block(0,0,a,a);
        Hn.block(0,a+c,a,b) = H.block(0,a,a,b);
        Hn.block(a+c,0,b,a) = H.block(a,0,b,a);
    }
    if(a>0 && c>0)
    {
        Hn.block(0,a,a,c) = H.block(0,a+b,a,c);
        Hn.block(a,0,c,a) = H.block(a+b,0,c,a);
    }
    if(c>0)
    {
        Hn.block(a,a,c,c) = H.block(a+b,a+b,c,c);
        Hn.block(a,a+c,c,b) = H.block(a+b,a,c,b);
        Hn.block(a+c,a,b,c) = H.block(a,a+b,b,c);
    }
    Hn.block(a+c,a+c,b,b) = H.block(a,a,b,b);

    // Perform marginalization (Schur complement)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hn.block(a+c,a+c,b,b),Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv=svd.singularValues();
    for (int i=0; i<b; ++i)
    {
        if (singularValues_inv(i)>1e-6)
            singularValues_inv(i)=1.0/singularValues_inv(i);
        else singularValues_inv(i)=0;
    }
    Eigen::MatrixXd invHb = svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose();
    Hn.block(0,0,a+c,a+c) = Hn.block(0,0,a+c,a+c) - Hn.block(0,a+c,a+c,b)*invHb*Hn.block(a+c,0,b,a+c);
    Hn.block(a+c,a+c,b,b) = Eigen::MatrixXd::Zero(b,b);
    Hn.block(0,a+c,a+c,b) = Eigen::MatrixXd::Zero(a+c,b);
    Hn.block(a+c,0,b,a+c) = Eigen::MatrixXd::Zero(b,a+c);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*
    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(),H.cols());
    if(a>0)
    {
        res.block(0,0,a,a) = Hn.block(0,0,a,a);
        res.block(0,a,a,b) = Hn.block(0,a+c,a,b);
        res.block(a,0,b,a) = Hn.block(a+c,0,b,a);
    }
    if(a>0 && c>0)
    {
        res.block(0,a+b,a,c) = Hn.block(0,a,a,c);
        res.block(a+b,0,c,a) = Hn.block(a,0,c,a);
    }
    if(c>0)
    {
        res.block(a+b,a+b,c,c) = Hn.block(a,a,c,c);
        res.block(a+b,a,c,b) = Hn.block(a,a+c,c,b);
        res.block(a,a+b,b,c) = Hn.block(a+c,a,b,c);
    }

    res.block(a,a,b,b) = Hn.block(a+c,a+c,b,b);

    return res;
}

void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel, bool bGauss, float priorG, float priorA)
{
    Verbose::PrintMess("inertial optimization", Verbose::VERBOSITY_NORMAL);
    int its = 200;
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    if (priorG!=0.f)
        solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+(pKFi->mnId)+1);
        if (bFixedVel)
            VV->setFixed(true);
        else
            VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid*2+2);
    if (bFixedVel)
        VG->setFixed(true);
    else
        VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid*2+3);
    if (bFixedVel)
        VA->setFixed(true);
    else
        VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Rwg);
    VGDir->setId(maxKFid*2+4);
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(scale);
    VS->setId(maxKFid*2+5);
    VS->setFixed(!bMono); // Fixed for stereo case
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    vector<pair<KeyFrame*,KeyFrame*> > vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());
    //std::cout << "build optimization graph" << std::endl;

    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
            if(!pKFi->mpImuPreintegrated)
                std::cout << "Not preintegrated measurement" << std::endl;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid*2+2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid*2+3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid*2+4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid*2+5);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG << ", "<< VA << ", " << VP2 << ", " << VV2 <<  ", "<< VGDir << ", "<< VS <<endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(make_pair(pKFi->mPrevKF,pKFi));
            optimizer.addEdge(ei);

        }
    }

    // Compute error for different scales
    std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);

    scale = VS->estimate();

    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid*2+2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid*2+3));
    Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();
    scale = VS->estimate();


    IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);
    Rwg = VGDir->estimate().Rwg;

    //Keyframes velocities and biases
    const int N = vpKFs.size();
    for(size_t i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+(pKFi->mnId)+1));
        Eigen::Vector3d Vw = VV->estimate(); // Velocity is scaled after
        pKFi->SetVelocity(Vw.cast<float>());

        if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01)
        {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        }
        else
            pKFi->SetNewBias(b);


    }
}


void Optimizer::InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG, float priorA)
{
    int its = 200; // Check number of iterations
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (fixed poses and optimizable velocities)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+(pKFi->mnId)+1);
        VV->setFixed(false);

        optimizer.addVertex(VV);
    }

    // Biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(maxKFid*2+2);
    VG->setFixed(false);
    optimizer.addVertex(VG);

    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(maxKFid*2+3);
    VA->setFixed(false);

    optimizer.addVertex(VA);
    // prior acc bias
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA;
    epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);
    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG;
    epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
    VGDir->setId(maxKFid*2+4);
    VGDir->setFixed(true);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(1.0);
    VS->setId(maxKFid*2+5);
    VS->setFixed(true); // Fixed since scale is obtained from already well initialized map
    optimizer.addVertex(VS);

    // Graph edges
    // IMU links with gravity and scale
    vector<EdgeInertialGS*> vpei;
    vpei.reserve(vpKFs.size());
    vector<pair<KeyFrame*,KeyFrame*> > vppUsedKF;
    vppUsedKF.reserve(vpKFs.size());

    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;

            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(maxKFid*2+2);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(maxKFid*2+3);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid*2+4);
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(maxKFid*2+5);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                cout << "Error" << VP1 << ", "<< VV1 << ", "<< VG << ", "<< VA << ", " << VP2 << ", " << VV2 <<  ", "<< VGDir << ", "<< VS <<endl;

                continue;
            }
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

            vpei.push_back(ei);

            vppUsedKF.push_back(make_pair(pKFi->mPrevKF,pKFi));
            optimizer.addEdge(ei);

        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(its);


    // Recover optimized data
    // Biases
    VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid*2+2));
    VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid*2+3));
    Vector6d vb;
    vb << VG->estimate(), VA->estimate();
    bg << VG->estimate();
    ba << VA->estimate();

    IMU::Bias b (vb[3],vb[4],vb[5],vb[0],vb[1],vb[2]);

    //Keyframes velocities and biases
    const int N = vpKFs.size();
    for(size_t i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;

        VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+(pKFi->mnId)+1));
        Eigen::Vector3d Vw = VV->estimate();
        pKFi->SetVelocity(Vw.cast<float>());

        if ((pKFi->GetGyroBias() - bg.cast<float>()).norm() > 0.01)
        {
            pKFi->SetNewBias(b);
            if (pKFi->mpImuPreintegrated)
                pKFi->mpImuPreintegrated->Reintegrate();
        }
        else
            pKFi->SetNewBias(b);
    }
}

void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale)
{
    int its = 10;
    long unsigned int maxKFid = pMap->GetMaxKFid();
    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Set KeyFrame vertices (all variables are fixed)
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKFi = vpKFs[i];
        if(pKFi->mnId>maxKFid)
            continue;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+1+(pKFi->mnId));
        VV->setFixed(true);
        optimizer.addVertex(VV);

        // Vertex of fixed biases
        VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
        VG->setId(2*(maxKFid+1)+(pKFi->mnId));
        VG->setFixed(true);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(vpKFs.front());
        VA->setId(3*(maxKFid+1)+(pKFi->mnId));
        VA->setFixed(true);
        optimizer.addVertex(VA);
    }

    // Gravity and scale
    VertexGDir* VGDir = new VertexGDir(Rwg);
    VGDir->setId(4*(maxKFid+1));
    VGDir->setFixed(false);
    optimizer.addVertex(VGDir);
    VertexScale* VS = new VertexScale(scale);
    VS->setId(4*(maxKFid+1)+1);
    VS->setFixed(false);
    optimizer.addVertex(VS);

    // Graph edges
    int count_edges = 0;
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        if(pKFi->mPrevKF && pKFi->mnId<=maxKFid)
        {
            if(pKFi->isBad() || pKFi->mPrevKF->mnId>maxKFid)
                continue;
                
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex((maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex((maxKFid+1)+pKFi->mnId);
            g2o::HyperGraph::Vertex* VG = optimizer.vertex(2*(maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VA = optimizer.vertex(3*(maxKFid+1)+pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(4*(maxKFid+1));
            g2o::HyperGraph::Vertex* VS = optimizer.vertex(4*(maxKFid+1)+1);
            if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
            {
                Verbose::PrintMess("Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) + ", " + to_string(VG->id()) + ", " + to_string(VA->id()) + ", " + to_string(VP2->id()) + ", " + to_string(VV2->id()) +  ", " + to_string(VGDir->id()) + ", " + to_string(VS->id()), Verbose::VERBOSITY_NORMAL);

                continue;
            }
            count_edges++;
            EdgeInertialGS* ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
            ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
            ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
            ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            ei->setRobustKernel(rk);
            rk->setDelta(1.f);
            optimizer.addEdge(ei);
        }
    }

    // Compute error for different scales
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float err = optimizer.activeRobustChi2();
    optimizer.optimize(its);
    optimizer.computeActiveErrors();
    float err_end = optimizer.activeRobustChi2();
    // Recover optimized data
    scale = VS->estimate();
    Rwg = VGDir->estimate().Rwg;
}

void Optimizer::LocalBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag)
{
    bool bShowImages = false;

    vector<MapPoint*> vpMPs;

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    optimizer.setVerbose(false);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;
    set<KeyFrame*> spKeyFrameBA;

    Map* pCurrentMap = pMainKF->GetMap();

    // Set fixed KeyFrame vertices
    int numInsertedPoints = 0;
    for(KeyFrame* pKFi : vpFixedKF)
    {
        if(pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map", Verbose::VERBOSITY_NORMAL);
            continue;
        }

        pKFi->mnBALocalForMerge = pMainKF->mnId;

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
        for(MapPoint* pMPi : spViewMPs)
        {
            if(pMPi)
                if(!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)

                    if(pMPi->mnBALocalForMerge!=pMainKF->mnId)
                    {
                        vpMPs.push_back(pMPi);
                        pMPi->mnBALocalForMerge=pMainKF->mnId;
                        numInsertedPoints++;
                    }
        }

        spKeyFrameBA.insert(pKFi);
    }

    // Set non fixed Keyframe vertices
    set<KeyFrame*> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
    numInsertedPoints = 0;
    for(KeyFrame* pKFi : vpAdjustKF)
    {
        if(pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            continue;

        pKFi->mnBALocalForMerge = pMainKF->mnId;

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        Sophus::SE3<float> Tcw = pKFi->GetPose();
        vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));
        vSE3->setId(pKFi->mnId);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        set<MapPoint*> spViewMPs = pKFi->GetMapPoints();
        for(MapPoint* pMPi : spViewMPs)
        {
            if(pMPi)
            {
                if(!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)
                {
                    if(pMPi->mnBALocalForMerge != pMainKF->mnId)
                    {
                        vpMPs.push_back(pMPi);
                        pMPi->mnBALocalForMerge = pMainKF->mnId;
                        numInsertedPoints++;
                    }
                }
            }
        }

        spKeyFrameBA.insert(pKFi);
    }

    const int nExpectedSize = (vpAdjustKF.size()+vpFixedKF.size())*vpMPs.size();

    vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // Set MapPoint vertices
    map<KeyFrame*, int> mpObsKFs;
    map<KeyFrame*, int> mpObsFinalKFs;
    map<MapPoint*, int> mpObsMPs;
    for(unsigned int i=0; i < vpMPs.size(); ++i)
    {
        MapPoint* pMPi = vpMPs[i];
        if(pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMPi->GetWorldPos().cast<double>());
        const int id = pMPi->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);


        const map<KeyFrame*,tuple<int,int>> observations = pMPi->GetObservations();
        int nEdges = 0;
        //SET EDGES
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid || pKF->mnBALocalForMerge != pMainKF->mnId || !pKF->GetMapPoint(get<0>(mit->second)))
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[get<0>(mit->second)];

            if(pKF->mvuRight[get<0>(mit->second)]<0) //Monocular
            {
                mpObsMPs[pMPi]++;
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber2D);

                e->pCamera = pKF->mpCamera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMPi);

                mpObsKFs[pKF]++;
            }
            else // RGBD or Stereo
            {
                mpObsMPs[pMPi]+=2;
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuber3D);

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMPi);

                mpObsKFs[pKF]++;
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    map<unsigned long int, int> mWrongObsKF;
    if(bDoMore)
    {
        // Check inlier observations
        int badMonoMP = 0, badStereoMP = 0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badMonoMP++;
            }
            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badStereoMP++;
            }

            e->setRobustKernel(0);
        }
        Verbose::PrintMess("[BA]: First optimization(Huber), there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP) + " stereo bad edges", Verbose::VERBOSITY_DEBUG);

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
    }

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());
    set<MapPoint*> spErasedMPs;
    set<KeyFrame*> spErasedKFs;

    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
            mWrongObsKF[pKFi->mnId]++;
            badMonoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
            mWrongObsKF[pKFi->mnId]++;
            badStereoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    Verbose::PrintMess("[BA]: Second optimization, there are " + to_string(badMonoMP) + " monocular and " + to_string(badStereoMP) + " sterero bad edges", Verbose::VERBOSITY_DEBUG);

    // Get Map Mutex
    unique_lock<mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }
    for(unsigned int i=0; i < vpMPs.size(); ++i)
    {
        MapPoint* pMPi = vpMPs[i];
        if(pMPi->isBad())
            continue;

        const map<KeyFrame*,tuple<int,int>> observations = pMPi->GetObservations();
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFrame* pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid || pKF->mnBALocalForKF != pMainKF->mnId || !pKF->GetMapPoint(get<0>(mit->second)))
                continue;

            if(pKF->mvuRight[get<0>(mit->second)]<0) //Monocular
            {
                mpObsFinalKFs[pKF]++;
            }
            else // RGBD or Stereo
            {
                mpObsFinalKFs[pKF]++;
            }
        }
    }

    // Recover optimized data
    // Keyframes
    for(KeyFrame* pKFi : vpAdjustKF)
    {
        if(pKFi->isBad())
            continue;

        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());

        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;
        vector<MapPoint*> vpMonoMPsBad, vpStereoMPsBad;

        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];
            KeyFrame* pKFedge = vpEdgeKFMono[i];

            if(pKFi != pKFedge)
            {
                continue;
            }

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                numMonoBadPoints++;
                vpMonoMPsBad.push_back(pMP);

            }
            else
            {
                numMonoOptPoints++;
                vpMonoMPsOpt.push_back(pMP);
            }

        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];
            KeyFrame* pKFedge = vpEdgeKFMono[i];

            if(pKFi != pKFedge)
            {
                continue;
            }

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                numStereoBadPoints++;
                vpStereoMPsBad.push_back(pMP);
            }
            else
            {
                numStereoOptPoints++;
                vpStereoMPsOpt.push_back(pMP);
            }
        }

        pKFi->SetPose(Tiw);
    }

    //Points
    for(MapPoint* pMPi : vpMPs)
    {
        if(pMPi->isBad())
            continue;

        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMPi->mnId+maxKFid+1));
        pMPi->SetWorldPos(vPoint->estimate().cast<float>());
        pMPi->UpdateNormalAndDepth();

    }
}


void Optimizer::MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses)
{
    const int Nd = 6;
    const unsigned long maxKFid = pCurrKF->mnId;

    vector<KeyFrame*> vpOptimizableKFs;
    vpOptimizableKFs.reserve(2*Nd);

    // For cov KFS, inertial parameters are not optimized
    const int maxCovKF = 30;
    vector<KeyFrame*> vpOptimizableCovKFs;
    vpOptimizableCovKFs.reserve(maxCovKF);

    // Add sliding window for current KF
    vpOptimizableKFs.push_back(pCurrKF);
    pCurrKF->mnBALocalForKF = pCurrKF->mnId;
    for(int i=1; i<Nd; i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
            break;
    }

    list<KeyFrame*> lFixedKeyFrames;
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF=pCurrKF->mnId;
    }
    else
    {
        vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Add temporal neighbours to merge KF (previous and next KFs)
    vpOptimizableKFs.push_back(pMergeKF);
    pMergeKF->mnBALocalForKF = pCurrKF->mnId;

    // Previous KFs
    for(int i=1; i<(Nd/2); i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
            break;
    }

    // We fix just once the old map
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF=pCurrKF->mnId;
    }
    else
    {
        vpOptimizableKFs.back()->mnBALocalForKF=0;
        vpOptimizableKFs.back()->mnBAFixedForKF=pCurrKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Next KFs
    if(pMergeKF->mNextKF)
    {
        vpOptimizableKFs.push_back(pMergeKF->mNextKF);
        vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    }

    while(vpOptimizableKFs.size()<(2*Nd))
    {
        if(vpOptimizableKFs.back()->mNextKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
            vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by optimizable keyframes
    list<MapPoint*> lLocalMapPoints;
    map<MapPoint*,int> mLocalObs;
    for(int i=0; i<N; i++)
    {
        vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            // Using mnBALocalForKF we avoid redundance here, one MP can not be added several times to lLocalMapPoints
            MapPoint* pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mnBALocalForKF!=pCurrKF->mnId)
                    {
                        mLocalObs[pMP]=1;
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF=pCurrKF->mnId;
                    }
                    else {
                        mLocalObs[pMP]++;
                    }
        }
    }

    std::vector<std::pair<MapPoint*, int>> pairs;
    pairs.reserve(mLocalObs.size());
    for (auto itr = mLocalObs.begin(); itr != mLocalObs.end(); ++itr)
        pairs.push_back(*itr);
    sort(pairs.begin(), pairs.end(),sortByVal);

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    int i=0;
    for(vector<pair<MapPoint*,int>>::iterator lit=pairs.begin(), lend=pairs.end(); lit!=lend; lit++, i++)
    {
        map<KeyFrame*,tuple<int,int>> observations = lit->first->GetObservations();
        if(i>=maxCovKF)
            break;
        for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pCurrKF->mnId && pKFi->mnBAFixedForKF!=pCurrKF->mnId) // If optimizable or already included...
            {
                pKFi->mnBALocalForKF=pCurrKF->mnId;
                if(!pKFi->isBad())
                {
                    vpOptimizableCovKFs.push_back(pKFi);
                    break;
                }
            }
        }
    }

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e3);

    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Set Local KeyFrame vertices
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Local cov keyframes vertices
    int Ncov=vpOptimizableCovKFs.size();
    for(int i=0; i<Ncov; i++)
    {
        KeyFrame* pKFi = vpOptimizableCovKFs[i];

        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(false);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(false);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(false);
            optimizer.addVertex(VA);
        }
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(true);
        optimizer.addVertex(VP);

        if(pKFi->bImu)
        {
            VertexVelocity* VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid+3*(pKFi->mnId)+1);
            VV->setFixed(true);
            optimizer.addVertex(VV);
            VertexGyroBias* VG = new VertexGyroBias(pKFi);
            VG->setId(maxKFid+3*(pKFi->mnId)+2);
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias* VA = new VertexAccBias(pKFi);
            VA->setId(maxKFid+3*(pKFi->mnId)+3);
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }
    }

    // Create intertial constraints
    vector<EdgeInertial*> vei(N,(EdgeInertial*)NULL);
    vector<EdgeGyroRW*> vegr(N,(EdgeGyroRW*)NULL);
    vector<EdgeAccRW*> vear(N,(EdgeAccRW*)NULL);
    for(int i=0;i<N;i++)
    {
        //cout << "inserting inertial edge " << i << endl;
        KeyFrame* pKFi = vpOptimizableKFs[i];

        if(!pKFi->mPrevKF)
        {
            Verbose::PrintMess("NOT INERTIAL LINK TO PREVIOUS FRAME!!!!", Verbose::VERBOSITY_NORMAL);
            continue;
        }
        if(pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
        {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
            g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
            g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
            g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);

            if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
            {
                cerr << "Error " << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            // TODO Uncomment
            g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
            vei[i]->setRobustKernel(rki);
            rki->setDelta(sqrt(16.92));
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0,VG1);
            vegr[i]->setVertex(1,VG2);
            Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0,VA1);
            vear[i]->setVertex(1,VA2);
            Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
            vear[i]->setInformation(InfoA);
            optimizer.addEdge(vear[i]);
        }
        else
            Verbose::PrintMess("ERROR building inertial edge", Verbose::VERBOSITY_NORMAL);
    }

    Verbose::PrintMess("end inserting inertial edges", Verbose::VERBOSITY_NORMAL);


    // Set MapPoint vertices
    const int nExpectedSize = (N+Ncov+lFixedKeyFrames.size())*lLocalMapPoints.size();

    // Mono
    vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const unsigned long iniMPid = maxKFid*5;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        if (!pMP)
            continue;

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(pMP->GetWorldPos().cast<double>());

        unsigned long id = pMP->mnId+iniMPid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();

        // Create visual constraints
        for(map<KeyFrame*,tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if (!pKFi)
                continue;

            if ((pKFi->mnBALocalForKF!=pCurrKF->mnId) && (pKFi->mnBAFixedForKF!=pCurrKF->mnId))
                continue;

            if (pKFi->mnId>maxKFid){
                continue;
            }


            if(optimizer.vertex(id)==NULL || optimizer.vertex(pKFi->mnId)==NULL)
                continue;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[get<0>(mit->second)];

                if(pKFi->mvuRight[get<0>(mit->second)]<0) // Monocular observation
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);
                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // stereo observation
                {
                    const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(8);

    vector<pair<KeyFrame*,MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        EdgeMono* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>chi2Mono2)
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Stereo
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>chi2Stereo2)
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }

    // Get Map Mutex and erase outliers
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);
    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFrame* pKFi = vToErase[i].first;
            MapPoint* pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }


    // Recover optimized data
    //Keyframes
    for(int i=0; i<N; i++)
    {
        KeyFrame* pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);

        Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
        corrPoses[pKFi] = g2oSiw;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));
        }
    }

    for(int i=0; i<Ncov; i++)
    {
        KeyFrame* pKFi = vpOptimizableCovKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);

        Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
        g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
        corrPoses[pKFi] = g2oSiw;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));
        }
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}

/**
 * @brief 使用上一关键帧+当前帧的视觉信息和IMU信息，优化当前帧位姿
 *
 * 可分为以下几个步骤：
 * // Step 1：创建g2o优化器，初始化顶点和边
 * // Step 2：启动多轮优化，剔除外点
 * // Step 3：更新当前帧位姿、速度、IMU偏置
 * // Step 4：记录当前帧的优化状态，包括参数信息和对应的海森矩阵
 *
 * @param[in] pFrame 当前帧，也是待优化的帧
 * @param[in] bRecInit 调用这个函数的位置并没有传这个参数，因此它的值默认为false
 * @return int 返回优化后的内点数
 */
 int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
    {
        // 1. 创建g2o优化器，初始化顶点和边
        // 构建一个稀疏求解器
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        // 使用dense的求解器，（常见非dense求解器有cholmod线性求解器和shur补线性求解器）
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
        // 使用高斯牛顿求解器
        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        optimizer.setVerbose(false);
        optimizer.setAlgorithm(solver);

        // 当前帧单（左）目地图点数目
        int nInitialMonoCorrespondences = 0;
        int nInitialStereoCorrespondences = 0;
        //上面两项的和，即参与优化的地图点总数目
        int nInitialCorrespondences = 0;

        // Set Frame vertex
        // 2. 确定节点
        // 当前帧的位姿，旋转+平移，6-dim
        VertexPose *VP = new VertexPose(pFrame);
        VP->setId(0);
        VP->setFixed(false);
        optimizer.addVertex(VP);
        // 当前帧的速度，3-dim
        VertexVelocity *VV = new VertexVelocity(pFrame);
        VV->setId(1);
        VV->setFixed(false);
        optimizer.addVertex(VV);
        // 当前帧的陀螺仪偏置，3-dim
        VertexGyroBias *VG = new VertexGyroBias(pFrame);
        VG->setId(2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        // 当前帧的加速度偏置，3-dim
        VertexAccBias *VA = new VertexAccBias(pFrame);
        VA->setId(3);
        VA->setFixed(false);
        optimizer.addVertex(VA);
        // setFixed(false)这个设置使以上四个顶点（15个参数）在优化时更新

        // Set MapPoint vertices
        // 当前帧的特征点总数
        const int N = pFrame->N;
        // 当前帧左目的特征点总数
        const int Nleft = pFrame->Nleft;
        // 当前帧是否存在右目（即是否为双目）
        const bool bRight = (Nleft != -1);

        vector<EdgeMonoOnlyPose *> vpEdgesMono;       //存储边
        vector<EdgeStereoOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeMono;               // 存储边顶点的特征点索引
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesMono.reserve(N);
        vpEdgesStereo.reserve(N);
        vnIndexEdgeMono.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值5.991
        const float thHuberMono = sqrt(5.991);
        // 自由度为3的卡方分布，显著性水平为0.05，对应的临界阈值7.815
        const float thHuberStereo = sqrt(7.815);

        {
            // 锁定地图点。由于需要使用地图点来构造顶点和边,因此不希望在构造的过程中部分地图点被改写造成不一致甚至是段错误
            // 3. 投影边
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP)
                {
                    cv::KeyPoint kpUn;

                    // Left monocular observation
                    // 这里说的Left monocular包含两种情况：1.单目情况 2.双目情况下的左目
                    if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft)
                    {
                        // 如果是双目情况下的左目
                        if (i < Nleft) // pair left-right
                            //使用未畸变校正的特征点
                            kpUn = pFrame->mvKeys[i];
                            // 如果是单目
                        else
                            // 使用畸变校正过的特征点
                            kpUn = pFrame->mvKeysUn[i];

                        // 单目地图点计数增加
                        nInitialMonoCorrespondences++;
                        // 当前地图点默认设置为不是外点
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        // 第一种边(视觉重投影约束)：地图点投影到该帧图像的坐标与特征点坐标偏差尽可能小
                        EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                        // 将位姿作为第一个顶点
                        e->setVertex(0, VP);

                        // 设置观测值，即去畸变后的像素坐标
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        // 获取不确定度，这里调用uncertainty2返回固定值1.0
                        // ?这里的1.0是作为缺省值的意思吗？是否可以根据对视觉信息的信任度动态修改这个值，比如标定的误差？
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        // invSigma2 = (Inverse(协方差矩阵))^2，表明该约束在各个维度上的可信度
                        //  图像金字塔层数越高，可信度越差
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        // 设置该约束的信息矩阵
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        // 设置鲁棒核函数，避免其误差的平方项出现数值过大的增长 注：后续在优化2次后会用e->setRobustKernel(0)禁掉鲁棒核函数
                        e->setRobustKernel(rk);

                        // 重投影误差的自由度为2，设置对应的卡方阈值
                        rk->setDelta(thHuberMono);

                        // 将第一种边加入优化器
                        optimizer.addEdge(e);

                        // 将第一种边加入vpEdgesMono
                        vpEdgesMono.push_back(e);
                        // 将对应的特征点索引加入vnIndexEdgeMono
                        vnIndexEdgeMono.push_back(i);
                    }
                        // Stereo observation
                    else if (!bRight)
                    {
                        nInitialStereoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysUn[i];
                        const float kp_ur = pFrame->mvuRight[i];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        EdgeStereoOnlyPose *e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                        const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }

                    // Right monocular observation
                    if (bRight && i >= Nleft)
                    {
                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysRight[i - Nleft];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                }
            }
        }
        // 统计参与优化的地图点总数目
        nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

        // 4. 上一个关键帧节点
        // pKF为上一关键帧
        KeyFrame *pKF = pFrame->mpLastKeyFrame;

        // 上一关键帧的位姿，旋转+平移，6-dim
        VertexPose *VPk = new VertexPose(pKF);
        VPk->setId(4);
        VPk->setFixed(true);
        optimizer.addVertex(VPk);
        // 上一关键帧的速度，3-dim
        VertexVelocity *VVk = new VertexVelocity(pKF);
        VVk->setId(5);
        VVk->setFixed(true);
        optimizer.addVertex(VVk);
        // 上一关键帧的陀螺仪偏置，3-dim
        VertexGyroBias *VGk = new VertexGyroBias(pKF);
        VGk->setId(6);
        VGk->setFixed(true);
        optimizer.addVertex(VGk);
        // 上一关键帧的加速度偏置，3-dim
        VertexAccBias *VAk = new VertexAccBias(pKF);
        VAk->setId(7);
        VAk->setFixed(true);
        optimizer.addVertex(VAk);
        // setFixed(true)这个设置使以上四个顶点（15个参数）的值在优化时保持固定
        // 既然被选为关键帧，就不能太善变

        // 5. 第二种边（IMU预积分约束）：两帧之间位姿的变化量与IMU预积分的值偏差尽可能小
        EdgeInertial *ei = new EdgeInertial(pFrame->mpImuPreintegrated);

        // 将上一关键帧四个顶点（P、V、BG、BA）和当前帧两个顶点（P、V）加入第二种边
        ei->setVertex(0, VPk);
        ei->setVertex(1, VVk);
        ei->setVertex(2, VGk);
        ei->setVertex(3, VAk);
        ei->setVertex(4, VP);
        ei->setVertex(5, VV);
        // 把第二种边加入优化器
        optimizer.addEdge(ei);

        // 6. 第三种边（陀螺仪随机游走约束）：陀螺仪的随机游走值在相近帧间不会相差太多  residual=VG-VGk
        // 用大白话来讲就是用固定的VGK拽住VG，防止VG在优化中放飞自我
        EdgeGyroRW *egr = new EdgeGyroRW();
        // 将上一关键帧的BG加入第三种边
        egr->setVertex(0, VGk);
        // 将当前帧的BG加入第三种边
        egr->setVertex(1, VG);
        // C值在预积分阶段更新，range(9,12)对应陀螺仪偏置的协方差，最终cvInfoG值为inv(∑(GyroRW^2/freq))
        Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        egr->setInformation(InfoG);
        // 把第三种边加入优化器
        optimizer.addEdge(egr);

        // 7. 第四种边（加速度随机游走约束）：加速度的随机游走值在相近帧间不会相差太多  residual=VA-VAk
        // 用大白话来讲就是用固定的VAK拽住VA，防止VA在优化中放飞自我
        EdgeAccRW *ear = new EdgeAccRW();
        // 将上一关键帧的BA加入第四种边
        ear->setVertex(0, VAk);
        // 将当前帧的BA加入第四种边
        ear->setVertex(1, VA);
        // C值在预积分阶段更新，range(12,15)对应加速度偏置的协方差，最终cvInfoG值为inv(∑(AccRW^2/freq))
        Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        ear->setInformation(InfoA);
        // 把第四种边加入优化器
        optimizer.addEdge(ear);

        // 8. 启动多轮优化，剔除外点

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        // 卡方检验值呈递减趋势，目的是让检验越来越苛刻
        float chi2Mono[4] = {12, 7.5, 5.991, 5.991};
        float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};
        // 4次优化的迭代次数都为10
        int its[4] = {10, 10, 10, 10};

        // 坏点数
        int nBad = 0;
        // 单目坏点数
        int nBadMono = 0;
        // 双目坏点数
        int nBadStereo = 0;
        // 单目内点数
        int nInliersMono = 0;
        // 双目内点数
        int nInliersStereo = 0;
        // 内点数
        int nInliers = 0;
        bool bOut = false;

        // 进行4次优化
        for (size_t it = 0; it < 4; it++)
        {
            // 初始化优化器,这里的参数0代表只对level为0的边进行优化（不传参数默认也是0）
            optimizer.initializeOptimization(0);
            // 每次优化迭代十次
            optimizer.optimize(its[it]);

            // 每次优化都重新统计各类点的数目
            nBad = 0;
            nBadMono = 0;
            nBadStereo = 0;
            nInliers = 0;
            nInliersMono = 0;
            nInliersStereo = 0;

            // 使用1.5倍的chi2Mono作为“近点”的卡方检验值，意味着地图点越近，检验越宽松
            // 地图点如何定义为“近点”在下面的代码中有解释
            float chi2close = 1.5 * chi2Mono[it];

            // For monocular observations
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                EdgeMonoOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];

                // 如果这条误差边是来自于outlier
                if (pFrame->mvbOutlier[idx])
                {
                    // 计算这条边上次优化后的误差
                    e->computeError();
                }

                // 就是error*\Omega*error，表示了这条边考虑置信度以后的误差大小
                const float chi2 = e->chi2();

                // 当地图点在当前帧的深度值小于10时，该地图点属于close（近点）
                // mTrackDepth是在Frame.cc的isInFrustum函数中计算出来的
                bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

                // 判断某地图点为外点的条件有以下三种：
                // 1.该地图点不是近点并且误差大于卡方检验值chi2Mono[it]
                // 2.该地图点是近点并且误差大于卡方检验值chi2close
                // 3.深度不为正
                // 每次优化后，用更小的卡方检验值，原因是随着优化的进行，对划分为内点的信任程度越来越低
                if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive())
                {
                    // 将该点设置为外点
                    pFrame->mvbOutlier[idx] = true;
                    // 外点不参与下一轮优化
                    e->setLevel(1);
                    // 单目坏点数+1
                    nBadMono++;
                }
                else
                {
                    // 将该点设置为内点（暂时）
                    pFrame->mvbOutlier[idx] = false;
                    // 内点继续参与下一轮优化
                    e->setLevel(0);
                    // 单目内点数+1
                    nInliersMono++;
                }

                // 从第三次优化开始就不设置鲁棒核函数了，原因是经过两轮优化已经趋向准确值，不会有太大误差
                if (it == 2)
                    e->setRobustKernel(0);
            }

            // For stereo observations
            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                EdgeStereoOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1); // not included in next optimization
                    nBadStereo++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersStereo++;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            // 内点总数=单目内点数+双目内点数
            nInliers = nInliersMono + nInliersStereo;
            // 坏点数=单目坏点数+双目坏点数
            nBad = nBadMono + nBadStereo;

            if (optimizer.edges().size() < 10)
            {
                cout << "PIOLKF: NOT ENOUGH EDGES" << endl;
                break;
            }
        }

        // If not too much tracks, recover not too bad points
        // 9. 若4次优化后内点数小于30，尝试恢复一部分不那么糟糕的坏点
        if ((nInliers < 30) && !bRecInit)
        {
            // 重新从0开始统计坏点数
            nBad = 0;
            // 单目可容忍的卡方检验最大值（如果误差比这还大就不要挣扎了...）
            const float chi2MonoOut = 18.f;
            const float chi2StereoOut = 24.f;
            EdgeMonoOnlyPose *e1;
            EdgeStereoOnlyPose *e2;
            // 遍历所有单目特征点
            for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++)
            {
                const size_t idx = vnIndexEdgeMono[i];
                // 获取这些特征点对应的边
                e1 = vpEdgesMono[i];
                e1->computeError();
                // 判断误差值是否超过单目可容忍的卡方检验最大值，是的话就把这个点保下来
                if (e1->chi2() < chi2MonoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
            for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++)
            {
                const size_t idx = vnIndexEdgeStereo[i];
                e2 = vpEdgesStereo[i];
                e2->computeError();
                if (e2->chi2() < chi2StereoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
        }

        // 10. 更新当前帧位姿、速度、IMU偏置

        // Recover optimized pose, velocity and biases
        // 给当前帧设置优化后的旋转、位移、速度，用来更新位姿
        pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
        Vector6d b;
        b << VG->estimate(), VA->estimate();
        // 给当前帧设置优化后的bg，ba
        pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

        // 11. 记录当前帧的优化状态，包括参数信息和对应的海森矩阵
        // Recover Hessian, marginalize keyFframe states and generate new prior for frame
        Eigen::Matrix<double, 15, 15> H;
        H.setZero();

        // H(x)=J(x).t()*info*J(x)

        // J(x)取的是EdgeInertial中的_jacobianOplus[4]和_jacobianOplus[5]，即EdgeInertial::computeError计算出来的er,ev,ep对当前帧Pose和Velocity的偏导
        //因此ei->GetHessian2的结果为：
        // H(∂er/∂r) H(∂er/∂t) H(∂er/∂v)
        // H(∂ev/∂r) H(∂ev/∂t) H(∂ev/∂v)
        // H(∂ep/∂r) H(∂ep/∂t) H(∂ep/∂v)
        //每项H都是3x3，故GetHessian2的结果是9x9
        H.block<9, 9>(0, 0) += ei->GetHessian2();

        // J(x)取的是EdgeGyroRW中的_jacobianOplusXj，即EdgeGyroRW::computeError计算出来的_error(ebg)对当前帧bg的偏导
        //因此egr->GetHessian2的结果为：
        // H(∂ebg/∂bg) ，3x3
        H.block<3, 3>(9, 9) += egr->GetHessian2();

        // J(x)取的是EdgeAccRW中的_jacobianOplusXj，即EdgeAccRW::computeError计算出来的_error(ebg)对当前帧ba的偏导
        //因此ear->GetHessian2的结果为：
        // H(∂eba/∂ba)  ，3x3
        H.block<3, 3>(12, 12) += ear->GetHessian2();

        //经过前面几步，Hessian Matrix长这个样子（注：省略了->GetHessian2()）
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // ei ei ei ei ei ei ei ei ei   0      0     0    0     0    0
        // 0  0  0  0  0  0  0   0  0  egr egr egr  0     0     0
        // 0  0  0  0  0  0  0   0  0  egr egr egr  0     0     0
        // 0  0  0  0  0  0  0   0  0  egr egr egr  0     0     0
        // 0  0  0  0  0  0  0   0  0    0     0      0  ear ear ear
        // 0  0  0  0  0  0  0   0  0    0     0      0  ear ear ear
        // 0  0  0  0  0  0  0   0  0    0     0      0  ear ear ear

        int tot_in = 0, tot_out = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeMonoOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (!pFrame->mvbOutlier[idx])
            {
                H.block<6, 6>(0, 0) += e->GetHessian();
                tot_in++;
            }
            else
                tot_out++;
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            EdgeStereoOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (!pFrame->mvbOutlier[idx])
            {
                H.block<6, 6>(0, 0) += e->GetHessian();
                tot_in++;
            }
            else
                tot_out++;
        }

        // 设eie = ei->GetHessian2()+∑(e->GetHessian())
        // 则最终Hessian Matrix长这样
        // eie eie eie eie eie eie         ei ei ei   0      0     0    0     0    0
        // eie eie eie eie eie eie         ei ei ei   0      0     0    0     0    0
        // eie eie eie eie eie eie         ei ei ei   0      0     0    0     0    0
        // eie eie eie eie eie eie         ei ei ei   0      0     0    0     0    0
        // eie eie eie eie eie eie         ei ei ei   0      0     0    0     0    0
        // eie eie eie eie eie eie         ei ei ei   0      0     0    0     0    0
        // ei    ei    ei    ei   ei   ei  ei ei ei   0      0     0    0     0    0
        // ei    ei    ei    ei   ei   ei  ei ei ei   0      0     0    0     0    0
        // ei    ei    ei    ei   ei   ei  ei ei ei   0      0     0    0     0    0
        //  0     0     0     0     0    0   0   0  0  egr egr egr  0     0     0
        //  0     0     0     0     0    0   0   0  0  egr egr egr  0     0     0
        //  0     0     0     0     0    0   0   0  0  egr egr egr  0     0     0
        //  0     0     0     0     0    0   0   0  0    0     0      0  ear ear ear
        //  0     0     0     0     0    0   0   0  0    0     0      0  ear ear ear
        //  0     0     0     0     0    0   0   0  0    0     0      0  ear ear ear

        // 构造一个ConstraintPoseImu对象，为下一帧提供先验约束
        // 构造对象所使用的参数是当前帧P、V、BG、BA的估计值和H矩阵
        pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(), VA->estimate(), H);
        // 在PoseInertialOptimizationLastFrame函数中，会将ConstraintPoseImu信息作为“上一帧先验约束”生成一条优化边

        // 返回值：内点数 = 总地图点数目 - 坏点（外点）数目
        return nInitialCorrespondences - nBad;
    }

/**
 * @brief 使用上一帧+当前帧的视觉信息和IMU信息，优化当前帧位姿
 *
 * 可分为以下几个步骤：
 * // Step 1：创建g2o优化器，初始化顶点和边
 * // Step 2：启动多轮优化，剔除外点
 * // Step 3：更新当前帧位姿、速度、IMU偏置
 * // Step 4：记录当前帧的优化状态，包括参数信息和边缘化后的海森矩阵
 *
 * @param[in] pFrame 当前帧，也是待优化的帧
 * @param[in] bRecInit 调用这个函数的位置并没有传这个参数，因此它的值默认为false
 * @return int 返回优化后的内点数
 */
int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
    {
        // Step 1：创建g2o优化器，初始化顶点和边
        // 构建一个稀疏求解器
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        // 使用dense的求解器，（常见非dense求解器有cholmod线性求解器和shur补线性求解器）
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        // 使用高斯牛顿求解器
        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        // 当前帧单（左）目地图点数目
        int nInitialMonoCorrespondences = 0;
        int nInitialStereoCorrespondences = 0;
        int nInitialCorrespondences = 0;

        // Set Current Frame vertex
        // 当前帧的位姿，旋转+平移，6-dim
        VertexPose *VP = new VertexPose(pFrame);
        VP->setId(0);
        VP->setFixed(false);  // 需要优化，所以不固定
        optimizer.addVertex(VP);
        // 当前帧的速度，3-dim
        VertexVelocity *VV = new VertexVelocity(pFrame);
        VV->setId(1);
        VV->setFixed(false);
        optimizer.addVertex(VV);
        // 当前帧的陀螺仪偏置，3-dim
        VertexGyroBias *VG = new VertexGyroBias(pFrame);
        VG->setId(2);
        VG->setFixed(false);
        optimizer.addVertex(VG);
        // 当前帧的加速度偏置，3-dim
        VertexAccBias *VA = new VertexAccBias(pFrame);
        VA->setId(3);
        VA->setFixed(false);
        optimizer.addVertex(VA);

        // Set MapPoint vertices
        // 当前帧的特征点总数 N = Nleft + Nright
        // 对于单目 N!=0, Nleft=-1，Nright=-1
        // 对于双目 N!=0, Nleft!=-1，Nright!=-1
        const int N = pFrame->N;
        // 当前帧左目的特征点总数
        const int Nleft = pFrame->Nleft;
        // 当前帧是否存在右目（即是否为双目），存在为true
        //?感觉可以更直接点啊 bRight = (Nright!=-1)
        const bool bRight = (Nleft != -1);

        vector<EdgeMonoOnlyPose *> vpEdgesMono;
        vector<EdgeStereoOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeMono;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesMono.reserve(N);
        vpEdgesStereo.reserve(N);
        vnIndexEdgeMono.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值5.991
        const float thHuberMono = sqrt(5.991);
        // 自由度为3的卡方分布，显著性水平为0.05，对应的临界阈值7.815
        const float thHuberStereo = sqrt(7.815);

        {
            // 锁定地图点。由于需要使用地图点来构造顶点和边,因此不希望在构造的过程中部分地图点被改写造成不一致甚至是段错误
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP)
                {
                    cv::KeyPoint kpUn;
                    // Left monocular observation
                    // 这里说的Left monocular包含两种情况：1.单目情况 2.双目情况下的左目
                    if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft)
                    {
                        // 如果是双目情况下的左目
                        if (i < Nleft)  // pair left-right
                            // 使用未畸变校正的特征点
                            kpUn = pFrame->mvKeys[i];
                            // 如果是单目
                        else
                            // 使用畸变校正过的特征点
                            kpUn = pFrame->mvKeysUn[i];

                        // 单目地图点计数增加
                        nInitialMonoCorrespondences++;
                        // 当前地图点默认设置为不是外点
                        pFrame->mvbOutlier[i] = false;
                        // 观测的特征点
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        // 第一种边(视觉重投影约束)：地图点投影到该帧图像的坐标与特征点坐标偏差尽可能小
                        EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

                        // 将位姿作为第一个顶点
                        e->setVertex(0, VP);

                        // 设置观测值，即去畸变后的像素坐标
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        // 获取不确定度，这里调用uncertainty2返回固定值1.0
                        // ?这里返回1.0是作为缺省值，是否可以根据对视觉信息的信任度动态修改这个值，比如标定的误差？
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        // invSigma2 = (Inverse(协方差矩阵))^2，表明该约束在各个维度上的可信度
                        //  图像金字塔层数越高，可信度越差
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        // 设置该约束的信息矩阵
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        // 设置鲁棒核函数，避免其误差的平方项出现数值过大的增长 注：后续在优化2次后会用e->setRobustKernel(0)禁掉鲁棒核函数
                        e->setRobustKernel(rk);
                        // 重投影误差的自由度为2，设置对应的卡方阈值
                        rk->setDelta(thHuberMono);

                        // 将第一种边加入优化器
                        optimizer.addEdge(e);

                        // 将第一种边加入vpEdgesMono
                        vpEdgesMono.push_back(e);
                        // 将对应的特征点索引加入vnIndexEdgeMono
                        vnIndexEdgeMono.push_back(i);
                    }
                        // Stereo observation
                    else if (!bRight)
                    {
                        nInitialStereoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysUn[i];
                        const float kp_ur = pFrame->mvuRight[i];
                        Eigen::Matrix<double, 3, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        EdgeStereoOnlyPose *e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                        const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }

                    // Right monocular observation
                    if (bRight && i >= Nleft)
                    {
                        nInitialMonoCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        kpUn = pFrame->mvKeysRight[i - Nleft];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        EdgeMonoOnlyPose *e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

                        e->setVertex(0, VP);
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                }
            }
        }
        // 统计参与优化的地图点总数目
        nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

        // Set Previous Frame Vertex
        // pFp为上一帧
        Frame *pFp = pFrame->mpPrevFrame;

        // 上一帧的位姿，旋转+平移，6-dim
        VertexPose *VPk = new VertexPose(pFp);
        VPk->setId(4);
        VPk->setFixed(false);
        optimizer.addVertex(VPk);
        // 上一帧的速度，3-dim
        VertexVelocity *VVk = new VertexVelocity(pFp);
        VVk->setId(5);
        VVk->setFixed(false);
        optimizer.addVertex(VVk);
        // 上一帧的陀螺仪偏置，3-dim
        VertexGyroBias *VGk = new VertexGyroBias(pFp);
        VGk->setId(6);
        VGk->setFixed(false);
        optimizer.addVertex(VGk);
        // 上一帧的加速度偏置，3-dim
        VertexAccBias *VAk = new VertexAccBias(pFp);
        VAk->setId(7);
        VAk->setFixed(false);
        optimizer.addVertex(VAk);
        // setFixed(false)这个设置使以上四个顶点（15个参数）的值随优化而变，这样做会给上一帧再提供一些优化空间
        // 但理论上不应该优化过多，否则会有不良影响，故后面的代码会用第五种边来约束上一帧的变化量

        // 第二种边（IMU预积分约束）：两帧之间位姿的变化量与IMU预积分的值偏差尽可能小
        EdgeInertial *ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

        // 将上一帧四个顶点（P、V、BG、BA）和当前帧两个顶点（P、V）加入第二种边
        ei->setVertex(0, VPk);
        ei->setVertex(1, VVk);
        ei->setVertex(2, VGk);
        ei->setVertex(3, VAk);
        ei->setVertex(4, VP);
        ei->setVertex(5, VV);
        // 把第二种边加入优化器
        optimizer.addEdge(ei);

        // 第三种边（陀螺仪随机游走约束）：陀螺仪的随机游走值在相邻帧间不会相差太多  residual=VG-VGk
        // 用大白话来讲就是用固定的VGK拽住VG，防止VG在优化中放飞自我
        EdgeGyroRW *egr = new EdgeGyroRW();
        // 将上一帧的BG加入第三种边
        egr->setVertex(0, VGk);
        // 将当前帧的BG加入第三种边
        egr->setVertex(1, VG);
        // C值在预积分阶段更新，range(9,12)对应陀螺仪偏置的协方差，最终cvInfoG值为inv(∑(GyroRW^2/freq))
        Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        egr->setInformation(InfoG);
        // 把第三种边加入优化器
        optimizer.addEdge(egr);

        // 第四种边（加速度随机游走约束）：加速度的随机游走值在相近帧间不会相差太多  residual=VA-VAk
        // 用大白话来讲就是用固定的VAK拽住VA，防止VA在优化中放飞自我
        EdgeAccRW *ear = new EdgeAccRW();
        // 将上一帧的BA加入第四种边
        ear->setVertex(0, VAk);
        // 将当前帧的BA加入第四种边
        ear->setVertex(1, VA);
        // C值在预积分阶段更新，range(12,15)对应加速度偏置的协方差，最终cvInfoG值为inv(∑(AccRW^2/freq))
        Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        ear->setInformation(InfoA);
        // 把第四种边加入优化器
        optimizer.addEdge(ear);

        // ?既然有判空的操作，可以区分一下有先验信息（五条边）和无先验信息（四条边）的情况
        if (!pFp->mpcpi)
            Verbose::PrintMess("pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId), Verbose::VERBOSITY_NORMAL);

        // 第五种边（先验约束）：上一帧信息随优化的改变量要尽可能小
        EdgePriorPoseImu *ep = new EdgePriorPoseImu(pFp->mpcpi);

        // 将上一帧的四个顶点（P、V、BG、BA）加入第五种边
        ep->setVertex(0, VPk);
        ep->setVertex(1, VVk);
        ep->setVertex(2, VGk);
        ep->setVertex(3, VAk);
        g2o::RobustKernelHuber *rkp = new g2o::RobustKernelHuber;
        ep->setRobustKernel(rkp);
        rkp->setDelta(5);
        // 把第五种边加入优化器
        optimizer.addEdge(ep);

        // Step 2：启动多轮优化，剔除外点

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        //与PoseInertialOptimizationLastKeyFrame函数对比，区别在于：在优化过程中保持卡方阈值不变
        // 以下参数的解释
        // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值5.991
        // 自由度为3的卡方分布，显著性水平为0.05，对应的临界阈值7.815
        // 自由度为3的卡方分布，显著性水平为0.02，对应的临界阈值9.8
        // 自由度为3的卡方分布，显著性水平为0.001，对应的临界阈值15.6
        // 计算方法：https://stattrek.com/online-calculator/chi-square.aspx
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
        // 4次优化的迭代次数都为10
        const int its[4] = {10, 10, 10, 10};

        // 坏点数
        int nBad = 0;
        // 单目坏点数
        int nBadMono = 0;
        // 双目坏点数
        int nBadStereo = 0;
        // 单目内点数
        int nInliersMono = 0;
        // 双目内点数
        int nInliersStereo = 0;
        // 内点数
        int nInliers = 0;
        for (size_t it = 0; it < 4; it++)
        {
            // 初始化优化器,这里的参数0代表只对level为0的边进行优化（不传参数默认也是0）
            optimizer.initializeOptimization(0);
            // 每次优化迭代十次
            optimizer.optimize(its[it]);

            // 每次优化都重新统计各类点的数目
            nBad = 0;
            nBadMono = 0;
            nBadStereo = 0;
            nInliers = 0;
            nInliersMono = 0;
            nInliersStereo = 0;
            // 使用1.5倍的chi2Mono作为“近点”的卡方检验值，意味着地图点越近，检验越宽松
            // 地图点如何定义为“近点”在下面的代码中有解释
            float chi2close = 1.5 * chi2Mono[it];

            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                EdgeMonoOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];

                // 当地图点在当前帧的深度值小于10时，该地图点属于close（近点）
                // mTrackDepth是在Frame.cc的isInFrustum函数中计算出来的
                bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

                // 如果这条误差边是来自于outlier
                if (pFrame->mvbOutlier[idx])
                {
                    //计算本次优化后的误差
                    e->computeError();
                }

                // 就是error*\Omega*error，表示了这条边考虑置信度以后的误差大小
                const float chi2 = e->chi2();

                // 判断某地图点为外点的条件有以下三种：
                // 1.该地图点不是近点并且误差大于卡方检验值chi2Mono[it]
                // 2.该地图点是近点并且误差大于卡方检验值chi2close
                // 3.深度不为正
                // 每次优化后，用更小的卡方检验值，原因是随着优化的进行，对划分为内点的信任程度越来越低
                if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive())
                {
                    // 将该点设置为外点
                    pFrame->mvbOutlier[idx] = true;
                    // 外点不参与下一轮优化
                    e->setLevel(1);
                    // 单目坏点数+1
                    nBadMono++;
                }
                else
                {
                    // 将该点设置为内点（暂时）
                    pFrame->mvbOutlier[idx] = false;
                    // 内点继续参与下一轮优化
                    e->setLevel(0);
                    // 单目内点数+1
                    nInliersMono++;
                }

                // 从第三次优化开始就不设置鲁棒核函数了，原因是经过两轮优化已经趋向准确值，不会有太大误差
                if (it == 2)
                    e->setRobustKernel(0);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                EdgeStereoOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBadStereo++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                    nInliersStereo++;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            // 内点总数=单目内点数+双目内点数
            nInliers = nInliersMono + nInliersStereo;
            // 坏点数=单目坏点数+双目坏点数
            nBad = nBadMono + nBadStereo;

            if (optimizer.edges().size() < 10)
            {
                cout << "PIOLF: NOT ENOUGH EDGES" << endl;
                break;
            }
        }

        // 若4次优化后内点数小于30，尝试恢复一部分不那么糟糕的坏点
        if ((nInliers < 30) && !bRecInit)
        {
            // 重新从0开始统计坏点数
            nBad = 0;
            // 单目可容忍的卡方检验最大值（如果误差比这还大就不要挣扎了...）
            const float chi2MonoOut = 18.f;
            const float chi2StereoOut = 24.f;
            EdgeMonoOnlyPose *e1;
            EdgeStereoOnlyPose *e2;
            // 遍历所有单目特征点
            for (size_t i = 0, iend = vnIndexEdgeMono.size(); i < iend; i++)
            {
                const size_t idx = vnIndexEdgeMono[i];
                // 获取这些特征点对应的边
                e1 = vpEdgesMono[i];
                e1->computeError();
                if (e1->chi2() < chi2MonoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
            for (size_t i = 0, iend = vnIndexEdgeStereo.size(); i < iend; i++)
            {
                const size_t idx = vnIndexEdgeStereo[i];
                e2 = vpEdgesStereo[i];
                e2->computeError();
                if (e2->chi2() < chi2StereoOut)
                    pFrame->mvbOutlier[idx] = false;
                else
                    nBad++;
            }
        }

        // ?多此一举？优化过程中nInliers这个值已经计算过了，nInliersMono和nInliersStereo在后续代码中一直保持不变
        nInliers = nInliersMono + nInliersStereo;

        // Step 3：更新当前帧位姿、速度、IMU偏置

        // Recover optimized pose, velocity and biases
        // 给当前帧设置优化后的旋转、位移、速度，用来更新位姿
        pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
        Vector6d b;
        b << VG->estimate(), VA->estimate();
        // 给当前帧设置优化后的bg，ba
        pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

        // Step 4：记录当前帧的优化状态，包括参数信息和边缘化后的海森矩阵
        // Recover Hessian, marginalize previous frame states and generate new prior for frame
        // 包含本次优化所有信息矩阵的和，它代表本次优化对确定性的影响
        Eigen::Matrix<double, 30, 30> H;
        H.setZero();

        //第1步，加上EdgeInertial（IMU预积分约束）的海森矩阵
        // ei的定义
        // EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);
        // ei->setVertex(0, VPk);
        // ei->setVertex(1, VVk);
        // ei->setVertex(2, VGk);
        // ei->setVertex(3, VAk);
        // ei->setVertex(4, VP);  // VertexPose* VP = new VertexPose(pFrame);
        // ei->setVertex(5, VV);  // VertexVelocity* VV = new VertexVelocity(pFrame);
        // ei->GetHessian()  =  J.t * J 下同，不做详细标注了
        // J
        //       rn + tn  vn    gn   an   rn+1 + tn+1  vn+1
        // er      Jp1    Jv1  Jg1   Ja1      Jp2      Jv2
        // 角标1表示上一帧，2表示当前帧
        //      6            3             3           3            6            3
        // Jp1.t * Jp1  Jp1.t * Jv1  Jp1.t * Jg1  Jp1.t * Ja1  Jp1.t * Jp2  Jp1.t * Jv2     6
        // Jv1.t * Jp1  Jv1.t * Jv1  Jv1.t * Jg1  Jv1.t * Ja1  Jv1.t * Jp2  Jv1.t * Jv2     3
        // Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1  Jg1.t * Ja1  Jg1.t * Jp2  Jg1.t * Jv2     3
        // Ja1.t * Jp1  Ja1.t * Jv1  Ja1.t * Jg1  Ja1.t * Ja1  Ja1.t * Jp2  Ja1.t * Jv2     3
        // Jp2.t * Jp1  Jp2.t * Jv1  Jp2.t * Jg1  Jp2.t * Ja1  Jp2.t * Jp2  Jp2.t * Jv2     6
        // Jv2.t * Jp1  Jv2.t * Jv1  Jv2.t * Jg1  Jv2.t * Ja1  Jv2.t * Jp2  Jv2.t * Jv2     3
        // 所以矩阵是24*24 的
        H.block<24, 24>(0, 0) += ei->GetHessian();
        // 经过这步H变成了
        // 列数 6            3             3           3            6           3        6
        // ---------------------------------------------------------------------------------- 行数
        // Jp1.t * Jp1  Jp1.t * Jv1  Jp1.t * Jg1  Jp1.t * Ja1  Jp1.t * Jp2  Jp1.t * Jv2   0 |  6
        // Jv1.t * Jp1  Jv1.t * Jv1  Jv1.t * Jg1  Jv1.t * Ja1  Jv1.t * Jp2  Jv1.t * Jv2   0 |  3
        // Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1  Jg1.t * Ja1  Jg1.t * Jp2  Jg1.t * Jv2   0 |  3
        // Ja1.t * Jp1  Ja1.t * Jv1  Ja1.t * Jg1  Ja1.t * Ja1  Ja1.t * Jp2  Ja1.t * Jv2   0 |  3
        // Jp2.t * Jp1  Jp2.t * Jv1  Jp2.t * Jg1  Jp2.t * Ja1  Jp2.t * Jp2  Jp2.t * Jv2   0 |  6
        // Jv2.t * Jp1  Jv2.t * Jv1  Jv2.t * Jg1  Jv2.t * Ja1  Jv2.t * Jp2  Jv2.t * Jv2   0 |  3
        //     0            0            0            0            0           0          0 |  6
        // ----------------------------------------------------------------------------------

        //第2步，加上EdgeGyroRW（陀螺仪随机游走约束）的信息矩阵：
        //|   0~8   |       9~11     | 12~23 |     24~26     |27~29
        // 9~11是上一帧的bg(3-dim)，24~26是当前帧的bg(3-dim)
        //  egr的定义
        //  EdgeGyroRW* egr = new EdgeGyroRW();
        //  egr->setVertex(0, VGk);
        //  egr->setVertex(1, VG);
        Eigen::Matrix<double, 6, 6> Hgr = egr->GetHessian();
        H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);   // Jgr1.t * Jgr1
        H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);  // Jgr1.t * Jgr2
        H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);  // Jgr2.t * Jgr1
        H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3); // Jgr2.t * Jgr2
        // 经过这步H变成了
        // 列数 6            3                    3                      3            6           3             3         3
        // ----------------------------------------------------------------------------------------------------------------- 行数
        // Jp1.t * Jp1  Jp1.t * Jv1         Jp1.t * Jg1           Jp1.t * Ja1  Jp1.t * Jp2  Jp1.t * Jv2        0         0 |  6
        // Jv1.t * Jp1  Jv1.t * Jv1         Jv1.t * Jg1           Jv1.t * Ja1  Jv1.t * Jp2  Jv1.t * Jv2        0         0 |  3
        // Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1 + Jgr1.t * Jgr1  Jg1.t * Ja1  Jg1.t * Jp2  Jg1.t * Jv2  Jgr1.t * Jgr2   0 |  3
        // Ja1.t * Jp1  Ja1.t * Jv1         Ja1.t * Jg1           Ja1.t * Ja1  Ja1.t * Jp2  Ja1.t * Jv2        0         0 |  3
        // Jp2.t * Jp1  Jp2.t * Jv1         Jp2.t * Jg1           Jp2.t * Ja1  Jp2.t * Jp2  Jp2.t * Jv2        0         0 |  6
        // Jv2.t * Jp1  Jv2.t * Jv1         Jv2.t * Jg1           Jv2.t * Ja1  Jv2.t * Jp2  Jv2.t * Jv2        0         0 |  3
        //     0            0             Jgr2.t * Jgr1                 0            0           0       Jgr2.t * Jgr2   0 |  3
        //     0            0                    0                      0            0           0             0         0 |  3
        // -----------------------------------------------------------------------------------------------------------------

        //第3步，加上EdgeAccRW（加速度随机游走约束）的信息矩阵：
        //|   0~11   |      12~14    | 15~26 |     27~29     |30
        // 12~14是上一帧的ba(3-dim)，27~29是当前帧的ba(3-dim)
        // ear的定义
        // EdgeAccRW* ear = new EdgeAccRW();
        // ear->setVertex(0, VAk);
        // ear->setVertex(1, VA);
        Eigen::Matrix<double, 6, 6> Har = ear->GetHessian();
        H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0); // Jar1.t * Jar1
        H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3); // Jar1.t * Jar2
        H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0); // Jar2.t * Jar1
        H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3); // Jar2.t * Jar2
        // 经过这步H变成了
        // 列数 6            3                    3                            3                         6           3             3              3
        // --------------------------------------------------------------------------------------------------------------------------------------------------- 行数
        // |  Jp1.t * Jp1  Jp1.t * Jv1         Jp1.t * Jg1                 Jp1.t * Ja1            |  Jp1.t * Jp2  Jp1.t * Jv2        0              0        |  6
        // |  Jv1.t * Jp1  Jv1.t * Jv1         Jv1.t * Jg1                 Jv1.t * Ja1            |  Jv1.t * Jp2  Jv1.t * Jv2        0              0        |  3
        // |  Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1 + Jgr1.t * Jgr1        Jg1.t * Ja1            |  Jg1.t * Jp2  Jg1.t * Jv2  Jgr1.t * Jgr2        0        |  3
        // |  Ja1.t * Jp1  Ja1.t * Jv1         Ja1.t * Jg1           Ja1.t * Ja1 + Jar1.t * Jar1  |  Ja1.t * Jp2  Ja1.t * Jv2        0       Jar1.t * Jar2   |  3
        // |--------------------------------------------------------------------------------------------------------------------------------------------------
        // |  Jp2.t * Jp1  Jp2.t * Jv1         Jp2.t * Jg1                 Jp2.t * Ja1            |  Jp2.t * Jp2  Jp2.t * Jv2        0              0        |  6
        // |  Jv2.t * Jp1  Jv2.t * Jv1         Jv2.t * Jg1                 Jv2.t * Ja1            |  Jv2.t * Jp2  Jv2.t * Jv2        0              0        |  3
        // |      0            0              Jgr2.t * Jgr1                      0                |        0           0       Jgr2.t * Jgr2        0        |  3
        // |      0            0                    0                     Jar2.t * Jar1           |        0           0             0        Jar2.t * Jar2  |  3
        // ---------------------------------------------------------------------------------------------------------------------------------------------------

        //第4步，加上EdgePriorPoseImu（先验约束）的信息矩阵：
        //|   0~14   |  15~29
        // 0~14是上一帧的P(6-dim)、V(3-dim)、BG(3-dim)、BA(3-dim)
        // ep定义
        // EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);
        // ep->setVertex(0, VPk);
        // ep->setVertex(1, VVk);
        // ep->setVertex(2, VGk);
        // ep->setVertex(3, VAk);
        //      6            3             3           3
        // Jp1.t * Jp1  Jp1.t * Jv1  Jp1.t * Jg1  Jp1.t * Ja1     6
        // Jv1.t * Jp1  Jv1.t * Jv1  Jv1.t * Jg1  Jv1.t * Ja1     3
        // Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1  Jg1.t * Ja1     3
        // Ja1.t * Jp1  Ja1.t * Jv1  Ja1.t * Jg1  Ja1.t * Ja1     3
        H.block<15, 15>(0, 0) += ep->GetHessian(); // 上一帧 的H矩阵，矩阵太大了不写了。。。总之就是加到下标为1相关的了

        int tot_in = 0, tot_out = 0;
        // 第5步 关于位姿的海森
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeMonoOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (!pFrame->mvbOutlier[idx])
            {
                //  0~14  |     15~20   | 21~29
                // 15~20是当前帧的P(6-dim)
                H.block<6, 6>(15, 15) += e->GetHessian(); // 当前帧的H矩阵，矩阵太大了不写了。。。总之就是加到p2相关的了
                tot_in++;
            }
            else
                tot_out++;
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            EdgeStereoOnlyPose *e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if (!pFrame->mvbOutlier[idx])
            {
                //  0~14  |     15~20   | 21~29
                H.block<6, 6>(15, 15) += e->GetHessian();
                tot_in++;
            }
            else
                tot_out++;
        }

        // H  = |B  E.t| ------> |0             0|
        //      |E    A|         |0 A-E*B.inv*E.t|
        H = Marginalize(H, 0, 14);
        /*
        Marginalize里的函数在此处的调用等效于：
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.block(0, 0, 15, 15), Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
        for (int i = 0; i < 15; ++i)
        {
            if (singularValues_inv(i) > 1e-6)
                singularValues_inv(i) = 1.0 / singularValues_inv(i);
            else
                singularValues_inv(i) = 0;
        }
        Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
        H.block(15, 15, 15, 15) = H.block(15, 15, 15, 15) - H.block(15, 0, 15, 15) * invHb - H.block(0, 15, 15, 15);
        */

        //构造一个ConstraintPoseImu对象，为下一帧提供先验约束
        //构造对象所使用的参数是当前帧P、V、BG、BA的估计值和边缘化后的H矩阵
        pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb, VP->estimate().twb, VV->estimate(), VG->estimate(), VA->estimate(), H.block<15, 15>(15, 15));
        //下一帧使用的EdgePriorPoseImu参数来自于此
        delete pFp->mpcpi;
        pFp->mpcpi = NULL;

        //返回值：内点数 = 总地图点数目 - 坏点（外点）数目
        return nInitialCorrespondences - nBad;
    }

int Optimizer::PoseInertialKinematicOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
{
    // 1. 创建g2o优化器，初始化顶点和边
    g2o::SparseOptimizer optimizer;   // 构建一个稀疏求解器
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    // 使用dense的求解器，（常见非dense求解器有cholmod线性求解器和shur补线性求解器）
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    // 块求解器
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    // 使用高斯牛顿求解器
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);   // 高斯牛顿法
    optimizer.setVerbose(false);
    optimizer.setAlgorithm(solver);

    // 计数，当前帧单（左）目地图点数目 //todo 为啥要记录地图点，这里不是只优化pose吗？
    int nInitialMonoCorrespondences=0;
    int nInitialStereoCorrespondences=0;
    //上面两项的和，即参与优化的地图点总数目
    int nInitialCorrespondences=0;

    // Set Frame vertex
    // 2. 确定节点
    // 当前帧的位姿，旋转+平移，6-dim
    VertexPose* VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);
    optimizer.addVertex(VP);
    // 当前帧的速度，3-dim
    VertexVelocity* VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
    // 当前帧的陀螺仪偏置，3-dim
    VertexGyroBias* VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    // 当前帧的加速度计偏置，3-dim
    VertexAccBias* VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);
    // setFixed(false)这个设置使以上四个顶点（15个参数）在优化时更新

    // Set MapPoint vertices
    const int N = pFrame->N;             // 当前帧的特征点总数
    const int Nleft = pFrame->Nleft;     // 当前帧左目的特征点总数
    const bool bRight = (Nleft!=-1);     // 当前帧是否存在右目（即是否为双目）

    vector<EdgeMonoOnlyPose*> vpEdgesMono;
    vector<EdgeStereoOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeMono;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesMono.reserve(N);
    vpEdgesStereo.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float thHuberMono = sqrt(5.991);        // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值5.991
    const float thHuberStereo = sqrt(7.815);      // 自由度为3的卡方分布，显著性水平为0.05，对应的临界阈值7.815

    {
        // 锁定地图点。由于需要使用地图点来构造顶点和边,因此不希望在构造的过程中部分地图点被改写造成不一致甚至是段错误
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);
        // 3. 投影边
        for(int i=0; i<N; i++)    // 遍历所有特征点
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];    // 取出第i帧的地图点
            if(pMP)  // 地图点存在
            {
                cv::KeyPoint kpUn;
                // Left monocular observation
                // 这里说的Left monocular包含两种情况：1.单目情况 2.双目情况下的左目
                if((!bRight && pFrame->mvuRight[i]<0) || i < Nleft)
                {
                    // 如果是双目情况下的左目
                    if(i < Nleft) // pair left-right
                        kpUn = pFrame->mvKeys[i]; //使用未畸变校正的特征点
                    else
                        kpUn = pFrame->mvKeysUn[i]; // 使用畸变校正过的特征点

                    nInitialMonoCorrespondences++;  // 单目地图点计数增加
                    pFrame->mvbOutlier[i] = false;  // 当前地图点默认设置为不是外点

                    Eigen::Matrix<double,2,1> obs;  //特征点像素坐标
                    obs << kpUn.pt.x, kpUn.pt.y;

                    // 第一种边(视觉重投影约束)：地图点投影到该帧图像的坐标与特征点坐标偏差尽可能小
                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);

                    e->setVertex(0,VP);     // 将位姿作为第一个顶点
                    e->setMeasurement(obs);   // 设置观测值，即去畸变后的像素坐标

                    // Add here uncerteinty
                    // 获取不确定度，这里调用uncertainty2返回固定值1.0
                    // ?这里的1.0是作为缺省值的意思吗？是否可以根据对视觉信息的信任度动态修改这个值，比如标定的误差？
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);
                    // invSigma2 = (Inverse(协方差矩阵))^2，表明该约束在各个维度上的可信度
                    //  图像金字塔层数越高，可信度越差
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);// 设置该约束的信息矩阵

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    // 设置鲁棒核函数，避免其误差的平方项出现数值过大的增长 注：后续在优化2次后会用e->setRobustKernel(0)禁掉鲁棒核函数
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);   // 重投影误差的自由度为2，设置对应的卡方阈值

                    optimizer.addEdge(e);// 将第一种边加入优化器
                    vpEdgesMono.push_back(e);// 将第一种边加入vpEdgesMono，存储用
                    vnIndexEdgeMono.push_back(i);// 将对应的特征点索引加入vnIndexEdgeMono
                }
                // Stereo observation
                else if(!bRight)
                {
                    nInitialStereoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysUn[i];
                    const float kp_ur = pFrame->mvuRight[i];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                    const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }
                // Right monocular observation
                if(bRight && i >= Nleft)
                {
                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysRight[i - Nleft];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }
    }
    // 统计参与优化的地图点总数目
    nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;
    // 4. 上一个关键帧节点
    // pKF为上一关键帧
    KeyFrame* pKF = pFrame->mpLastKeyFrame;
    // 上一关键帧的位姿，旋转+平移，6-dim
    VertexPose* VPk = new VertexPose(pKF);
    VPk->setId(4);
    VPk->setFixed(true);
    optimizer.addVertex(VPk);
    // 上一关键帧的速度，3-dim
    VertexVelocity* VVk = new VertexVelocity(pKF);
    VVk->setId(5);
    VVk->setFixed(true);
    optimizer.addVertex(VVk);
    // 上一关键帧的陀螺仪偏置，3-dim
    VertexGyroBias* VGk = new VertexGyroBias(pKF);
    VGk->setId(6);
    VGk->setFixed(true);
    optimizer.addVertex(VGk);
    // 上一关键帧的加速度计偏置，3-dim
    VertexAccBias* VAk = new VertexAccBias(pKF);
    VAk->setId(7);
    VAk->setFixed(true);
    optimizer.addVertex(VAk);
    // setFixed(true)这个设置使以上四个顶点（15个参数）的值在优化时保持固定。既然被选为关键帧，就不能太善变

    // 5. 第二种边（IMU预积分约束）：两帧之间位姿的变化量与IMU预积分的值偏差尽可能小
    EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);
    // 顶点 6个
    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, VP);
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);    // 加入优化器

    // 6. 陀螺仪偏置边：陀螺仪的随机游走值在相近帧间不会相差太多  residual=VG-VGk
    EdgeGyroRW* egr = new EdgeGyroRW();
    egr->setVertex(0,VGk);
    egr->setVertex(1,VG);
    // C值在预积分阶段更新，range(9,12)对应陀螺仪偏置的协方差，最终cvInfoG值为inv(∑(GyroRW^2/freq))
    Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    // 7. 加速度计偏置边
    EdgeAccRW* ear = new EdgeAccRW();
    ear->setVertex(0,VAk);
    ear->setVertex(1,VA);
    // C值在预积分阶段更新，range(12,15)对应加速度偏置的协方差，最终cvInfoG值为inv(∑(AccRW^2/freq))
    Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    // todo new 运动学边
    EdgeKinematic* ek = new EdgeKinematic(pFrame->mpJointCalculated);
    ek->setVertex(0,VPk);
    ek->setVertex(1,VP);
    ek->setInformation(Eigen::Matrix<double,6,6>::Identity());  // information matrix
    optimizer.addEdge(ek);



    // 8. 启动多轮优化，剔除外点
    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    // 卡方检验值呈递减趋势，目的是让检验越来越苛刻
    float chi2Mono[4]={12,7.5,5.991,5.991};
    float chi2Stereo[4]={15.6,9.8,7.815,7.815};
    // 4次优化的迭代次数都为10
    int its[4]={10,10,10,10};

    int nBad = 0;               // 坏点数
    int nBadMono = 0;           // 单目坏点数
    int nBadStereo = 0;         // 双目坏点数
    int nInliersMono = 0;       // 单目内点数
    int nInliersStereo = 0;     // 双目内点数
    int nInliers = 0;           // 内点数
    // 进行4次优化
    for(size_t it=0; it<4; it++)
    {
        optimizer.initializeOptimization(0);  // 初始化优化器,这里的参数0代表只对level为0的边进行优化（不传参数默认也是0）
        optimizer.optimize(its[it]);       // 每次优化迭代十次
        // 每次优化都重新统计各类点的数目
        nBad = 0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers = 0;
        nInliersMono = 0;
        nInliersStereo = 0;
        // 使用1.5倍的chi2Mono作为“近点”的卡方检验值，意味着地图点越近，检验越宽松
        // 地图点如何定义为“近点”在下面的代码中有解释
        float chi2close = 1.5*chi2Mono[it];

        // For monocular observations
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            EdgeMonoOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];
            // 如果这条误差边是来自于outlier
            if(pFrame->mvbOutlier[idx])
            {
                // 计算这条边上次优化后的误差
                e->computeError();
            }
            // 就是error*\Omega*error，表示了这条边考虑置信度以后的误差大小
            const float chi2 = e->chi2();
            // 当地图点在当前帧的深度值小于10时，该地图点属于close（近点）
            // mTrackDepth 是在 Frame.cc 的 isInFrustum 函数中计算出来的
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth<10.f;
            // 判断某地图点为外点的条件有以下三种：
            // 1.该地图点不是近点并且误差大于卡方检验值chi2Mono[it]
            // 2.该地图点是近点并且误差大于卡方检验值chi2close
            // 3.深度不为正
            // 每次优化后，用更小的卡方检验值，原因是随着优化的进行，对划分为内点的信任程度越来越低
            if((chi2>chi2Mono[it]&&!bClose)||(bClose && chi2>chi2close)||!e->isDepthPositive())
            {
                pFrame->mvbOutlier[idx]=true;   // 将该点设置为外点
                e->setLevel(1);              // 外点不参与下一轮优化
                nBadMono++;                     // 单目坏点数+1
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;   // 将该点设置为内点（暂时）
                e->setLevel(0);                 // 内点继续参与下一轮优化
                nInliersMono++;                    // 单目内点数+1
            }
            // 从第三次优化开始就不设置鲁棒核函数了，原因是经过两轮优化已经趋向准确值，不会有太大误差
            if (it==2)
                e->setRobustKernel(0);
        }

        // For stereo observations
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1); // not included in next optimization
                nBadStereo++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
                nInliersStereo++;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        nInliers = nInliersMono + nInliersStereo;   // 内点总数=单目内点数+双目内点数
        nBad = nBadMono + nBadStereo;               // 坏点数=单目坏点数+双目坏点数

        if(optimizer.edges().size()<10)
        {
            break;
        }

    }

    // If not too much tracks, recover not too bad points
    // 9. 若4次优化后内点数小于30，尝试恢复一部分不那么糟糕的坏点
    if ((nInliers<30) && !bRecInit)
    {
        nBad=0;                            // 重新从0开始统计坏点数
        // 单目可容忍的卡方检验最大值（如果误差比这还大就不要挣扎了...）
        const float chi2MonoOut = 18.f;
        const float chi2StereoOut = 24.f;
        EdgeMonoOnlyPose* e1;
        EdgeStereoOnlyPose* e2;
        // 遍历所有单目特征点
        for(size_t i=0, iend=vnIndexEdgeMono.size(); i<iend; i++)
        {
            const size_t idx = vnIndexEdgeMono[i];
            e1 = vpEdgesMono[i];                    // 获取这些特征点对应的边
            e1->computeError();
            // 判断误差值是否超过单目可容忍的卡方检验最大值，是的话就把这个点保下来
            if (e1->chi2()<chi2MonoOut)
                pFrame->mvbOutlier[idx]=false;
            else
                nBad++;
        }
        for(size_t i=0, iend=vnIndexEdgeStereo.size(); i<iend; i++)
        {
            const size_t idx = vnIndexEdgeStereo[i];
            e2 = vpEdgesStereo[i];
            e2->computeError();
            if (e2->chi2()<chi2StereoOut)
                pFrame->mvbOutlier[idx]=false;
            else
                nBad++;
        }
    }

    // 10. 更新当前帧位姿、速度、IMU偏置

    // Recover optimized pose, velocity and biases
    // 给当前帧设置优化后的旋转、位移、速度，用来更新位姿
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    // 给当前帧设置优化后的bg，ba
    pFrame->mImuBias = IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]);

    // 11. 记录当前帧的优化状态，包括参数信息和对应的海森矩阵
    // Recover Hessian, marginalize keyFframe states and generate new prior for frame
    Eigen::Matrix<double,15,15> H;
    H.setZero();

    H.block<9,9>(0,0)+= ei->GetHessian2();
    H.block<3,3>(9,9) += egr->GetHessian2();
    H.block<3,3>(12,12) += ear->GetHessian2();

    int tot_in = 0, tot_out = 0;
    // 添加冲投影误差的hession
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
    {
        EdgeMonoOnlyPose* e = vpEdgesMono[i];

        const size_t idx = vnIndexEdgeMono[i];

        if(!pFrame->mvbOutlier[idx])
        {
            H.block<6,6>(0,0) += e->GetHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
    {
        EdgeStereoOnlyPose* e = vpEdgesStereo[i];

        const size_t idx = vnIndexEdgeStereo[i];

        if(!pFrame->mvbOutlier[idx])
        {
            H.block<6,6>(0,0) += e->GetHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    // todo 运动学
    H.block<6,6>(0,0) += ek->GetHessian2();
    // 则最终Hessian Matrix长这样
    // eieek eieek eieek eie eie eieek ei ei ei   0      0     0    0     0    0
    // eieek eieek eieek eie eie eieek ei ei ei   0      0     0    0     0    0
    // eieek eieek eieek eie eie eieek ei ei ei   0      0     0    0     0    0
    // eieek eieek eieek eie eie eieek ei ei ei   0      0     0    0     0    0
    // eieek eieek eieek eie eie eieek ei ei ei   0      0     0    0     0    0
    // eieek eieek eieek eie eie eieek ei ei ei   0      0     0    0     0    0
    // ei    ei    ei    ei   ei   ei  ei ei ei   0      0     0    0     0    0
    // ei    ei    ei    ei   ei   ei  ei ei ei   0      0     0    0     0    0
    // ei    ei    ei    ei   ei   ei  ei ei ei   0      0     0    0     0    0
    //  0     0     0     0    0   0   0  0  0   egr    egr    egr  0     0    0
    //  0     0     0     0    0   0   0  0  0   egr    egr    egr  0     0    0
    //  0     0     0     0    0   0   0  0  0   egr    egr    egr  0     0    0
    //  0     0     0     0    0   0   0  0  0    0      0      0   ear   ear   ear
    //  0     0     0     0    0   0   0  0  0    0      0      0   ear   ear   ear
    //  0     0     0     0    0   0   0  0  0    0      0      0   ear   ear   ear

    // 构造一个ConstraintPoseImu对象，为下一帧提供先验约束
    // todo 构造对象所使用的参数是当前帧P、V、BG、BA的估计值和H矩阵
    pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,VP->estimate().twb,VV->estimate(),VG->estimate(),VA->estimate(),H);
    // 在PoseInertialOptimizationLastFrame函数中，会将ConstraintPoseImu信息作为“上一帧先验约束”生成一条优化边

    // 返回值：内点数 = 总地图点数目 - 坏点（外点）数目
    return nInitialCorrespondences-nBad;
}
// notice
int Optimizer::PoseInertialKinematicOptimizationLastFrame(Frame *pFrame, bool bRecInit)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    int nInitialMonoCorrespondences=0;
    int nInitialStereoCorrespondences=0;
    int nInitialCorrespondences=0;

    // Set Current Frame vertex
    // 当前帧的位姿，旋转+平移，6-dim
    VertexPose* VP = new VertexPose(pFrame);
    VP->setId(0);
    VP->setFixed(false);         // 需要优化，所以不固定
    optimizer.addVertex(VP);
    VertexVelocity* VV = new VertexVelocity(pFrame);
    VV->setId(1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
    VertexGyroBias* VG = new VertexGyroBias(pFrame);
    VG->setId(2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pFrame);
    VA->setId(3);
    VA->setFixed(false);
    optimizer.addVertex(VA);

    // Set MapPoint vertices
    // 当前帧的特征点总数 N = Nleft + Nright
    // 对于单目 N!=0, Nleft=-1，Nright=-1
    // 对于双目 N!=0, Nleft!=-1，Nright!=-1
    const int N = pFrame->N;
    const int Nleft = pFrame->Nleft;     // 当前帧左目的特征点总数
    // 当前帧是否存在右目（即是否为双目），存在为true
    //?感觉可以更直接点啊 bRight = (Nright!=-1)
    const bool bRight = (Nleft!=-1);

    vector<EdgeMonoOnlyPose*> vpEdgesMono;
    vector<EdgeStereoOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeMono;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesMono.reserve(N);
    vpEdgesStereo.reserve(N);
    vnIndexEdgeMono.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for(int i=0; i<N; i++)
        {
            MapPoint* pMP = pFrame->mvpMapPoints[i];
            if(pMP)
            {
                cv::KeyPoint kpUn;
                // Left monocular observation
                if((!bRight && pFrame->mvuRight[i]<0) || i < Nleft)
                {
                    if(i < Nleft) // pair left-right
                        kpUn = pFrame->mvKeys[i];
                    else
                        kpUn = pFrame->mvKeysUn[i];

                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                // Stereo observation
                else if(!bRight)
                {
                    nInitialStereoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysUn[i];
                    const float kp_ur = pFrame->mvuRight[i];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

                    e->setVertex(0, VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

                    const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);
                }

                // Right monocular observation
                if(bRight && i >= Nleft)
                {
                    nInitialMonoCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    kpUn = pFrame->mvKeysRight[i - Nleft];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

                    e->setVertex(0,VP);
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pFrame->mpCamera->uncertainty2(obs);

                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
            }
        }
    }

    nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

    // Set Previous Frame Vertex
    // pFp为上一帧
    Frame* pFp = pFrame->mpPrevFrame;

    VertexPose* VPk = new VertexPose(pFp);
    VPk->setId(4);
    VPk->setFixed(false);
    optimizer.addVertex(VPk);
    VertexVelocity* VVk = new VertexVelocity(pFp);
    VVk->setId(5);
    VVk->setFixed(false);
    optimizer.addVertex(VVk);
    VertexGyroBias* VGk = new VertexGyroBias(pFp);
    VGk->setId(6);
    VGk->setFixed(false);
    optimizer.addVertex(VGk);
    VertexAccBias* VAk = new VertexAccBias(pFp);
    VAk->setId(7);
    VAk->setFixed(false);
    optimizer.addVertex(VAk);
    // setFixed(false)这个设置使以上四个顶点（15个参数）的值随优化而变，这样做会给上一帧再提供一些优化空间
    // 但理论上不应该优化过多，否则会有不良影响，故后面的代码会用第五种边来约束上一帧的变化量

    EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

    ei->setVertex(0, VPk);
    ei->setVertex(1, VVk);
    ei->setVertex(2, VGk);
    ei->setVertex(3, VAk);
    ei->setVertex(4, VP);
    ei->setVertex(5, VV);
    optimizer.addEdge(ei);

    EdgeGyroRW* egr = new EdgeGyroRW();
    egr->setVertex(0,VGk);
    egr->setVertex(1,VG);
    Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
    egr->setInformation(InfoG);
    optimizer.addEdge(egr);

    EdgeAccRW* ear = new EdgeAccRW();
    ear->setVertex(0,VAk);
    ear->setVertex(1,VA);
    Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
    ear->setInformation(InfoA);
    optimizer.addEdge(ear);

    // ?既然有判空的操作，可以区分一下有先验信息（五条边）和无先验信息（四条边）的情况
    if (!pFp->mpcpi)
        Verbose::PrintMess("pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId), Verbose::VERBOSITY_NORMAL);
    // 第五种边（先验约束）：上一帧信息随优化的改变量要尽可能小
    EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);
    // 将上一帧的四个顶点（P、V、BG、BA）加入第五种边
    ep->setVertex(0,VPk);
    ep->setVertex(1,VVk);
    ep->setVertex(2,VGk);
    ep->setVertex(3,VAk);
    g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
    ep->setRobustKernel(rkp);
    rkp->setDelta(5);
    // 把第五种边加入优化器
    optimizer.addEdge(ep);

    //todo new
    EdgeKinematic* ek = new EdgeKinematic(pFrame->mpJointCalculated);
    ek->setVertex(0,VPk);
    ek->setVertex(1,VP);
    ek->setInformation(Eigen::Matrix<double,6,6>::Identity());  // information matrix
    optimizer.addEdge(ek);

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    // Step 2：启动多轮优化，剔除外点
    // 以下参数的解释
    // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值5.991
    // 自由度为3的卡方分布，显著性水平为0.05，对应的临界阈值7.815
    // 自由度为3的卡方分布，显著性水平为0.02，对应的临界阈值9.8
    // 自由度为3的卡方分布，显著性水平为0.001，对应的临界阈值15.6
    // 计算方法：https://stattrek.com/online-calculator/chi-square.aspx
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={15.6f,9.8f,7.815f,7.815f};
    const int its[4]={10,10,10,10};

    int nBad=0;
    int nBadMono = 0;
    int nBadStereo = 0;
    int nInliersMono = 0;
    int nInliersStereo = 0;
    int nInliers=0;
    for(size_t it=0; it<4; it++)
    {
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        nBadMono = 0;
        nBadStereo = 0;
        nInliers=0;
        nInliersMono=0;
        nInliersStereo=0;
        float chi2close = 1.5*chi2Mono[it];

        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            EdgeMonoOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];
            bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth<10.f;

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if((chi2>chi2Mono[it]&&!bClose)||(bClose && chi2>chi2close)||!e->isDepthPositive())
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBadMono++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
                nInliersMono++;
            }

            if (it==2)
                e->setRobustKernel(0);

        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            const size_t idx = vnIndexEdgeStereo[i];

            if(pFrame->mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Stereo[it])
            {
                pFrame->mvbOutlier[idx]=true;
                e->setLevel(1);
                nBadStereo++;
            }
            else
            {
                pFrame->mvbOutlier[idx]=false;
                e->setLevel(0);
                nInliersStereo++;
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        nInliers = nInliersMono + nInliersStereo;   // 内点总数=单目内点数+双目内点数
        nBad = nBadMono + nBadStereo;               // 坏点数=单目坏点数+双目坏点数

        if(optimizer.edges().size()<10)
        {
            break;
        }
    }

    // 若4次优化后内点数小于30，尝试恢复一部分不那么糟糕的坏点
    if ((nInliers<30) && !bRecInit)
    {
        nBad=0;
        const float chi2MonoOut = 18.f;
        const float chi2StereoOut = 24.f;
        EdgeMonoOnlyPose* e1;
        EdgeStereoOnlyPose* e2;
        for(size_t i=0, iend=vnIndexEdgeMono.size(); i<iend; i++)
        {
            const size_t idx = vnIndexEdgeMono[i];
            e1 = vpEdgesMono[i];
            e1->computeError();
            if (e1->chi2()<chi2MonoOut)
                pFrame->mvbOutlier[idx]=false;
            else
                nBad++;

        }
        for(size_t i=0, iend=vnIndexEdgeStereo.size(); i<iend; i++)
        {
            const size_t idx = vnIndexEdgeStereo[i];
            e2 = vpEdgesStereo[i];
            e2->computeError();
            if (e2->chi2()<chi2StereoOut)
                pFrame->mvbOutlier[idx]=false;
            else
                nBad++;
        }
    }

    nInliers = nInliersMono + nInliersStereo;

    // Step 3：更新当前帧位姿、速度、IMU偏置
    // Recover optimized pose, velocity and biases
    // 给当前帧设置优化后的旋转、位移、速度，用来更新位姿
    pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
    Vector6d b;
    b << VG->estimate(), VA->estimate();
    pFrame->mImuBias = IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]);

    // Step 4：记录当前帧的优化状态，包括参数信息和边缘化后的海森矩阵
    // Recover Hessian, marginalize previous frame states and generate new prior for frame
    // 包含本次优化所有信息矩阵的和，它代表本次优化对确定性的影响
    Eigen::Matrix<double,30,30> H;
    H.setZero();
    //第1步，加上EdgeInertial（IMU预积分约束）的海森矩阵
    // ei的定义
    // EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);
    // ei->setVertex(0, VPk);
    // ei->setVertex(1, VVk);
    // ei->setVertex(2, VGk);
    // ei->setVertex(3, VAk);
    // ei->setVertex(4, VP);  // VertexPose* VP = new VertexPose(pFrame);
    // ei->setVertex(5, VV);  // VertexVelocity* VV = new VertexVelocity(pFrame);
    // ei->GetHessian()  =  J.t * J 下同，不做详细标注了
    // J
    //       rn + tn  vn    gn   an   rn+1 + tn+1  vn+1
    // er      Jp1    Jv1  Jg1   Ja1      Jp2      Jv2
    // 角标1表示上一帧，2表示当前帧
    //      6            3             3           3            6            3
    // Jp1.t * Jp1  Jp1.t * Jv1  Jp1.t * Jg1  Jp1.t * Ja1  Jp1.t * Jp2  Jp1.t * Jv2     6
    // Jv1.t * Jp1  Jv1.t * Jv1  Jv1.t * Jg1  Jv1.t * Ja1  Jv1.t * Jp2  Jv1.t * Jv2     3
    // Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1  Jg1.t * Ja1  Jg1.t * Jp2  Jg1.t * Jv2     3
    // Ja1.t * Jp1  Ja1.t * Jv1  Ja1.t * Jg1  Ja1.t * Ja1  Ja1.t * Jp2  Ja1.t * Jv2     3
    // Jp2.t * Jp1  Jp2.t * Jv1  Jp2.t * Jg1  Jp2.t * Ja1  Jp2.t * Jp2  Jp2.t * Jv2     6
    // Jv2.t * Jp1  Jv2.t * Jv1  Jv2.t * Jg1  Jv2.t * Ja1  Jv2.t * Jp2  Jv2.t * Jv2     3
    // 所以矩阵是24*24 的
    H.block<24,24>(0,0)+= ei->GetHessian();
    //第2步，加上EdgeGyroRW（陀螺仪随机游走约束）的信息矩阵：
    //|   0~8   |       9~11     | 12~23 |     24~26     |27~29
    // 9~11是上一帧的bg(3-dim)，24~26是当前帧的bg(3-dim)
    //  egr的定义
    //  EdgeGyroRW* egr = new EdgeGyroRW();
    //  egr->setVertex(0, VGk);
    //  egr->setVertex(1, VG);
    Eigen::Matrix<double,6,6> Hgr = egr->GetHessian();
    H.block<3,3>(9,9) += Hgr.block<3,3>(0,0);
    H.block<3,3>(9,24) += Hgr.block<3,3>(0,3);
    H.block<3,3>(24,9) += Hgr.block<3,3>(3,0);
    H.block<3,3>(24,24) += Hgr.block<3,3>(3,3);
    // 经过这步H变成了
    // 列数 6            3                    3                      3            6           3             3         3
    // ----------------------------------------------------------------------------------------------------------------- 行数
    // Jp1.t * Jp1  Jp1.t * Jv1         Jp1.t * Jg1           Jp1.t * Ja1  Jp1.t * Jp2  Jp1.t * Jv2        0         0 |  6
    // Jv1.t * Jp1  Jv1.t * Jv1         Jv1.t * Jg1           Jv1.t * Ja1  Jv1.t * Jp2  Jv1.t * Jv2        0         0 |  3
    // Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1 + Jgr1.t * Jgr1  Jg1.t * Ja1  Jg1.t * Jp2  Jg1.t * Jv2  Jgr1.t * Jgr2   0 |  3
    // Ja1.t * Jp1  Ja1.t * Jv1         Ja1.t * Jg1           Ja1.t * Ja1  Ja1.t * Jp2  Ja1.t * Jv2        0         0 |  3
    // Jp2.t * Jp1  Jp2.t * Jv1         Jp2.t * Jg1           Jp2.t * Ja1  Jp2.t * Jp2  Jp2.t * Jv2        0         0 |  6
    // Jv2.t * Jp1  Jv2.t * Jv1         Jv2.t * Jg1           Jv2.t * Ja1  Jv2.t * Jp2  Jv2.t * Jv2        0         0 |  3
    //     0            0             Jgr2.t * Jgr1                 0            0           0       Jgr2.t * Jgr2   0 |  3
    //     0            0                    0                      0            0           0             0         0 |  3
    // -----------------------------------------------------------------------------------------------------------------

    //第3步，加上EdgeAccRW（加速度随机游走约束）的信息矩阵：
    //|   0~11   |      12~14    | 15~26 |     27~29     |30
    // 12~14是上一帧的ba(3-dim)，27~29是当前帧的ba(3-dim)
    // ear的定义
    // EdgeAccRW* ear = new EdgeAccRW();
    // ear->setVertex(0, VAk);
    // ear->setVertex(1, VA);
    Eigen::Matrix<double,6,6> Har = ear->GetHessian();
    H.block<3,3>(12,12) += Har.block<3,3>(0,0);
    H.block<3,3>(12,27) += Har.block<3,3>(0,3);
    H.block<3,3>(27,12) += Har.block<3,3>(3,0);
    H.block<3,3>(27,27) += Har.block<3,3>(3,3);
    // 经过这步H变成了
    // 列数 6            3                    3                            3                         6           3             3              3
    // --------------------------------------------------------------------------------------------------------------------------------------------------- 行数
    // |  Jp1.t * Jp1  Jp1.t * Jv1         Jp1.t * Jg1                 Jp1.t * Ja1            |  Jp1.t * Jp2  Jp1.t * Jv2        0              0        |  6
    // |  Jv1.t * Jp1  Jv1.t * Jv1         Jv1.t * Jg1                 Jv1.t * Ja1            |  Jv1.t * Jp2  Jv1.t * Jv2        0              0        |  3
    // |  Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1 + Jgr1.t * Jgr1        Jg1.t * Ja1            |  Jg1.t * Jp2  Jg1.t * Jv2  Jgr1.t * Jgr2        0        |  3
    // |  Ja1.t * Jp1  Ja1.t * Jv1         Ja1.t * Jg1           Ja1.t * Ja1 + Jar1.t * Jar1  |  Ja1.t * Jp2  Ja1.t * Jv2        0       Jar1.t * Jar2   |  3
    // |--------------------------------------------------------------------------------------------------------------------------------------------------
    // |  Jp2.t * Jp1  Jp2.t * Jv1         Jp2.t * Jg1                 Jp2.t * Ja1            |  Jp2.t * Jp2  Jp2.t * Jv2        0              0        |  6
    // |  Jv2.t * Jp1  Jv2.t * Jv1         Jv2.t * Jg1                 Jv2.t * Ja1            |  Jv2.t * Jp2  Jv2.t * Jv2        0              0        |  3
    // |      0            0              Jgr2.t * Jgr1                      0                |        0           0       Jgr2.t * Jgr2        0        |  3
    // |      0            0                    0                     Jar2.t * Jar1           |        0           0             0        Jar2.t * Jar2  |  3
    // ---------------------------------------------------------------------------------------------------------------------------------------------------

    //第4步，加上EdgePriorPoseImu（先验约束）的信息矩阵：
    //|   0~14   |  15~29
    // 0~14是上一帧的P(6-dim)、V(3-dim)、BG(3-dim)、BA(3-dim)
    // ep定义
    // EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);
    // ep->setVertex(0, VPk);
    // ep->setVertex(1, VVk);
    // ep->setVertex(2, VGk);
    // ep->setVertex(3, VAk);
    //      6            3             3           3
    // Jp1.t * Jp1  Jp1.t * Jv1  Jp1.t * Jg1  Jp1.t * Ja1     6
    // Jv1.t * Jp1  Jv1.t * Jv1  Jv1.t * Jg1  Jv1.t * Ja1     3
    // Jg1.t * Jp1  Jg1.t * Jv1  Jg1.t * Jg1  Jg1.t * Ja1     3
    // Ja1.t * Jp1  Ja1.t * Jv1  Ja1.t * Jg1  Ja1.t * Ja1     3
    H.block<15,15>(0,0) += ep->GetHessian();

    // todo 第5步，加上运动学的信息矩阵
    // 0-5 | 6-14 | 15-20 |21-29
    Eigen::Matrix<double,12,12> Hk = ek->GetHessian();
    H.block<6,6>(0,0) += Hk.block<6,6>(0,0);
    H.block<6,6>(0,15) += Hk.block<6,6>(0,6);
    H.block<6,6>(15,0) += Hk.block<6,6>(6,0);
    H.block<6,6>(15,15) += Hk.block<6,6>(6,6);

    int tot_in = 0, tot_out = 0;
    // 第6步 关于位姿的海森
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
    {
        EdgeMonoOnlyPose* e = vpEdgesMono[i];

        const size_t idx = vnIndexEdgeMono[i];

        if(!pFrame->mvbOutlier[idx])
        {
            //  0~14  |     15~20   | 21~29
            // 15~20是当前帧的P(6-dim)
            H.block<6,6>(15,15) += e->GetHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
    {
        EdgeStereoOnlyPose* e = vpEdgesStereo[i];

        const size_t idx = vnIndexEdgeStereo[i];

        if(!pFrame->mvbOutlier[idx])
        {
            H.block<6,6>(15,15) += e->GetHessian();
            tot_in++;
        }
        else
            tot_out++;
    }

    // H  = |B  E.t| ------> |0             0|
    //      |E    A|         |0 A-E*B.inv*E.t|
    H = Marginalize(H,0,14);
    /*
    Marginalize里的函数在此处的调用等效于：
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H.block(0, 0, 15, 15), Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
    for (int i = 0; i < 15; ++i)
    {
        if (singularValues_inv(i) > 1e-6)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    Eigen::MatrixXd invHb = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
    H.block(15, 15, 15, 15) = H.block(15, 15, 15, 15) - H.block(15, 0, 15, 15) * invHb - H.block(0, 15, 15, 15);
    */

    //构造一个ConstraintPoseImu对象，为下一帧提供先验约束
    //构造对象所使用的参数是当前帧P、V、BG、BA的估计值和边缘化后的H矩阵
    pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,VP->estimate().twb,VV->estimate(),VG->estimate(),VA->estimate(),H.block<15,15>(15,15));
    //下一帧使用的EdgePriorPoseImu参数来自于此
    delete pFp->mpcpi;
    pFp->mpcpi = NULL;
    //返回值：内点数 = 总地图点数目 - 坏点（外点）数目
    return nInitialCorrespondences-nBad;
}


// 本质图优化
void Optimizer::OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections)
{
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<4, 4> > BlockSolver_4_4;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolverX::LinearSolverType * linearSolver =
            new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    optimizer.setAlgorithm(solver);

    const vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
    const vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFid();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);

    vector<VertexPose4DoF*> vpVertices(nMaxKFid+1);

    const int minFeat = 100;
    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        KeyFrame* pKF = vpKFs[i];
        if(pKF->isBad())
            continue;

        VertexPose4DoF* V4DoF;

        const int nIDi = pKF->mnId;

        LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            const g2o::Sim3 Swc = it->second.inverse();
            Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
            Eigen::Vector3d twc = Swc.translation();
            V4DoF = new VertexPose4DoF(Rwc, twc, pKF);
        }
        else
        {
            Sophus::SE3d Tcw = pKF->GetPose().cast<double>();
            g2o::Sim3 Siw(Tcw.unit_quaternion(),Tcw.translation(),1.0);

            vScw[nIDi] = Siw;
            V4DoF = new VertexPose4DoF(pKF);
        }

        if(pKF==pLoopKF)
            V4DoF->setFixed(true);

        V4DoF->setId(nIDi);
        V4DoF->setMarginalized(false);

        optimizer.addVertex(V4DoF);
        vpVertices[nIDi]=V4DoF;
    }
    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    // Edge used in posegraph has still 6Dof, even if updates of camera poses are just in 4DoF
    Eigen::Matrix<double,6,6> matLambda = Eigen::Matrix<double,6,6>::Identity();
    matLambda(0,0) = 1e3;
    matLambda(1,1) = 1e3;
    matLambda(0,0) = 1e3;

    // Set Loop edges
    Edge4DoF* e_loop;
    for(map<KeyFrame *, set<KeyFrame *> >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        const long unsigned int nIDi = pKF->mnId;
        const set<KeyFrame*> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];

        for(set<KeyFrame*>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            const long unsigned int nIDj = (*sit)->mnId;
            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sij = Siw * Sjw.inverse();
            Eigen::Matrix4d Tij;
            Tij.block<3,3>(0,0) = Sij.rotation().toRotationMatrix();
            Tij.block<3,1>(0,3) = Sij.translation();
            Tij(3,3) = 1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));

            e->information() = matLambda;
            e_loop = e;
            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // 1. Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        KeyFrame* pKF = vpKFs[i];

        const int nIDi = pKF->mnId;

        g2o::Sim3 Siw;

        // Use noncorrected poses for posegraph edges
        LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Siw = iti->second;
        else
            Siw = vScw[nIDi];

        // 1.1.0 Spanning tree edge
        KeyFrame* pParentKF = static_cast<KeyFrame*>(NULL);
        if(pParentKF)
        {
            int nIDj = pParentKF->mnId;

            g2o::Sim3 Swj;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Swj = (itj->second).inverse();
            else
                Swj =  vScw[nIDj].inverse();

            g2o::Sim3 Sij = Siw * Swj;
            Eigen::Matrix4d Tij;
            Tij.block<3,3>(0,0) = Sij.rotation().toRotationMatrix();
            Tij.block<3,1>(0,3) = Sij.translation();
            Tij(3,3)=1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // 1.1.1 Inertial edges
        KeyFrame* prevKF = pKF->mPrevKF;
        if(prevKF)
        {
            int nIDj = prevKF->mnId;

            g2o::Sim3 Swj;

            LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(prevKF);

            if(itj!=NonCorrectedSim3.end())
                Swj = (itj->second).inverse();
            else
                Swj =  vScw[nIDj].inverse();

            g2o::Sim3 Sij = Siw * Swj;
            Eigen::Matrix4d Tij;
            Tij.block<3,3>(0,0) = Sij.rotation().toRotationMatrix();
            Tij.block<3,1>(0,3) = Sij.translation();
            Tij(3,3)=1.;

            Edge4DoF* e = new Edge4DoF(Tij);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // 1.2 Loop edges
        const set<KeyFrame*> sLoopEdges = pKF->GetLoopEdges();
        for(set<KeyFrame*>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            KeyFrame* pLKF = *sit;
            if(pLKF->mnId<pKF->mnId)
            {
                g2o::Sim3 Swl;

                LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Swl = itl->second.inverse();
                else
                    Swl = vScw[pLKF->mnId].inverse();

                g2o::Sim3 Sil = Siw * Swl;
                Eigen::Matrix4d Til;
                Til.block<3,3>(0,0) = Sil.rotation().toRotationMatrix();
                Til.block<3,1>(0,3) = Sil.translation();
                Til(3,3) = 1.;

                Edge4DoF* e = new Edge4DoF(Til);
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pLKF->mnId)));
                e->information() = matLambda;
                optimizer.addEdge(e);
            }
        }

        // 1.3 Covisibility graph edges
        const vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            KeyFrame* pKFn = *vit;
            if(pKFn && pKFn!=pParentKF && pKFn!=prevKF && pKFn!=pKF->mNextKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                if(!pKFn->isBad() && pKFn->mnId<pKF->mnId)
                {
                    if(sInsertedEdges.count(make_pair(min(pKF->mnId,pKFn->mnId),max(pKF->mnId,pKFn->mnId))))
                        continue;

                    g2o::Sim3 Swn;

                    LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Swn = itn->second.inverse();
                    else
                        Swn = vScw[pKFn->mnId].inverse();

                    g2o::Sim3 Sin = Siw * Swn;
                    Eigen::Matrix4d Tin;
                    Tin.block<3,3>(0,0) = Sin.rotation().toRotationMatrix();
                    Tin.block<3,1>(0,3) = Sin.translation();
                    Tin(3,3) = 1.;
                    Edge4DoF* e = new Edge4DoF(Tin);
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFn->mnId)));
                    e->information() = matLambda;
                    optimizer.addEdge(e);
                }
            }
        }
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);

    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        KeyFrame* pKFi = vpKFs[i];

        const int nIDi = pKFi->mnId;

        VertexPose4DoF* Vi = static_cast<VertexPose4DoF*>(optimizer.vertex(nIDi));
        Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
        Eigen::Vector3d ti = Vi->estimate().tcw[0];

        g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri,ti,1.);
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();

        Sophus::SE3d Tiw(CorrectedSiw.rotation(),CorrectedSiw.translation());
        pKFi->SetPose(Tiw.cast<float>());
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;

        KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();
        nIDr = pRefKF->mnId;

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        Eigen::Matrix<double,3,1> eigP3Dw = pMP->GetWorldPos().cast<double>();
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
        pMP->SetWorldPos(eigCorrectedP3Dw.cast<float>());

        pMP->UpdateNormalAndDepth();
    }
    pMap->IncreaseChangeIndex();
}

} //namespace ORB_SLAM
