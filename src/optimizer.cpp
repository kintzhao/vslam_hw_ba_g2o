#include <cmath>
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "define.h"
#include "optimizer.h"
#include "convertor.h"

void two_view_ba(Frame &frame_last, Frame &frame_curr, LoaclMap &map, std::vector<FMatch> &matches, int n_iter)
{
    // TODO homework
    // after you complete this funtion, remove the "return"
    //return;

    const double fx = frame_last.K_(0, 0);
    const double fy = frame_last.K_(1, 1);
    const double cx = frame_last.K_(0, 2);
    const double cy = frame_last.K_(1, 2);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Set vertices
    Eigen::Matrix4d last_Tcw = frame_last.Twc_.inverse();
    Eigen::Matrix4d curr_Tcw = frame_curr.Twc_.inverse();
    //std::cout<<"last:    "<<std::endl;std::cout<<frame_last.Twc_<<std::endl;
    //std::cout<<"current: "<<std::endl;std::cout<<frame_curr.Twc_<<std::endl;

    const std::vector<Eigen::Vector2i> &features_last = frame_last.fts_;
    const std::vector<Eigen::Vector2i> &features_curr = frame_curr.fts_;

    int frame_id_last = frame_last.idx_;
    int frame_id_curr = frame_curr.idx_;
    //std::cout<<"frame_id_last:"<<frame_id_last<<"  frame_id_curr:"<<frame_id_curr<<std::endl;
    // TODO homework
    // add frame pose Vertex to optimizer
    // example:
    // g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    // ...
    // optimizer.addVertex(vSE3);

    g2o::VertexSE3Expmap *last_SE3 = new g2o::VertexSE3Expmap();
    last_SE3->setEstimate(Converter::toSE3Quat(last_Tcw));
    last_SE3->setId(frame_id_last);

    last_SE3->setFixed(true);
        //std::cout<<" fixed last_SE3" <<std::endl;
    optimizer.addVertex(last_SE3);

    g2o::VertexSE3Expmap *curr_SE3 = new g2o::VertexSE3Expmap();
    curr_SE3->setEstimate(Converter::toSE3Quat(curr_Tcw));
    curr_SE3->setId(frame_id_curr);
    curr_SE3->setFixed(false);

    optimizer.addVertex(curr_SE3);

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    bool bRobust = true;

    int max_frame_id = std::max(frame_id_last, frame_id_curr) + 1;

    // Set MapPoint vertices
    for(size_t i=0; i<matches.size(); i++)
    {
        if(matches[i].outlier)
            continue;

        uint32_t idx_curr = matches[i].first;
        uint32_t idx_last = matches[i].second;
        int32_t idx_mpt = frame_curr.mpt_track_[idx_curr];
        assert(idx_mpt >=0);
        assert(true == map.status_[idx_mpt]);
        assert(idx_mpt == frame_last.mpt_track_[idx_last]);

        Eigen::Vector3d &mpt = map.mpts_[idx_mpt];

        // TODO homework
        // add mappoint Vertex to optimizer
        // example:
        // g2o::VertexSBAPointXYZ * vPoint = new g2o::VertexSBAPointXYZ();
        // ...
        // optimizer.addVertex(vPoint);

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(mpt);
        const int id = i+max_frame_id;//idx_mpt+max_frame_id;
        //std::cout<<"matches:"<<i<<"  id:"<<id<<std::endl;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        // TODO homework
        // add edage to optimizer
        // example:
        // g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();
        // ...
        // optimizer.addEdge(e);

        const float invSigma2 = 1.0;
        g2o::EdgeSE3ProjectXYZ *e_last = new g2o::EdgeSE3ProjectXYZ();
        e_last->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
        e_last->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(frame_id_last)));

        e_last->setMeasurement(features_last[idx_last].cast<double>());
        e_last->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
        if (bRobust)
        {
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e_last->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
        }
        e_last->fx = fx;
        e_last->fy = fy;
        e_last->cx = cx;
        e_last->cy = cy;
        optimizer.addEdge(e_last);

        //
        g2o::EdgeSE3ProjectXYZ *e_curr = new g2o::EdgeSE3ProjectXYZ();
        e_curr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
        e_curr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(frame_id_curr)));

        e_curr->setMeasurement(features_curr[idx_curr].cast<double>());
        e_curr->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
        if (bRobust)
        {
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e_curr->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
        }
        e_curr->fx = fx;
        e_curr->fy = fy;
        e_curr->cx = cx;
        e_curr->cy = cy;
        optimizer.addEdge(e_curr);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(n_iter);

    // TODO homework
    // Recover optimized data
    // Frame Pose
    {
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(frame_id_last));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        std::cout<<"frame_id_last:"<<frame_id_last<<" "<<vSE3->fixed()<<std::endl;
        frame_last.Twc_ = Converter::toEigenMat(SE3quat).inverse();
    }

    {
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(frame_id_curr));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        std::cout<<"frame_id_curr:"<<frame_id_curr<<" "<<vSE3->fixed()<<std::endl;
        frame_curr.Twc_ = Converter::toEigenMat(SE3quat).inverse();
    }


    // Points
    for(size_t i = 0; i < matches.size(); i++)
    {
        if(matches[i].outlier) { continue; }
        uint32_t idx_last = matches[i].second;
        int32_t idx_mpt = frame_last.mpt_track_[idx_last];

        Eigen::Vector3d &mpt = map.mpts_[idx_mpt];
        //std::cout<<"matches:"<<i<<"  id:"<<idx_mpt+max_frame_id<<std::endl;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+max_frame_id/*idx_mpt+max_frame_id*/));

        mpt = vPoint->estimate();
    }
}
