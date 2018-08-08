/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2018, Rawashdeh Research Group
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************/
// Author: Mohamed Aladem

#include "lvt_pnp_solver.h"
#include "lvt_logging_utils.h"
#include "lvt_local_map.h"
#include "lvt_image_features_struct.h"
#include <Eigen/StdVector>
#include <numeric>
#include <g2o/config.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/icp/types_icp.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#define N_PASSES 2

lvt_pnp_solver::lvt_pnp_solver(double fx, double fy, double cx, double cy, double baseline)
    : m_fx(fx), m_fy(fy), m_cx(cx), m_cy(cy), m_baseline(baseline), m_optimizer(nullptr)
{
    m_optimizer = new g2o::SparseOptimizer();
    m_optimizer->setVerbose(false);
    g2o::BlockSolver_6_3::LinearSolverType *linear_solver = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linear_solver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(block_solver);
    m_optimizer->setAlgorithm(solver);
}

lvt_pnp_solver::~lvt_pnp_solver()
{
    delete m_optimizer;
}

lvt_pose lvt_pnp_solver::compute_pose(const lvt_pose &cam_pose, lvt_image_features_struct *left_struct,
                                      const lvt_vector3_array &map_points, const std::vector<int> &matches_left)
{
    LVT_LOG("Started computing pose...");
    // add camera
    const Eigen::Quaterniond orientation = cam_pose.get_orientation_quaternion();
    const Eigen::Vector3d position = cam_pose.get_position();
    g2o::SBACam sba_cam(orientation, position);
    sba_cam.setKcam(m_fx, m_fy, m_cx, m_cy, m_baseline);
    g2o::VertexCam *cam_vertex = new g2o::VertexCam();
    cam_vertex->setId(0);
    cam_vertex->setEstimate(sba_cam);
    cam_vertex->setFixed(false);
    m_optimizer->addVertex(cam_vertex);

    // add mono measurments
    static const double mono_chi = sqrt(LVT_REPROJECTION_TH2);
    int vertex_id = 1;
    std::vector<g2o::EdgeProjectP2MC *> mono_edges(map_points.size());
    for (size_t i = 0, count = map_points.size(); i < count; i++)
    {
        g2o::VertexSBAPointXYZ *point_vertex = new g2o::VertexSBAPointXYZ();
        point_vertex->setId(vertex_id++);
        point_vertex->setMarginalized(false);
        point_vertex->setEstimate(map_points[i]);
        point_vertex->setFixed(true);
        m_optimizer->addVertex(point_vertex);

        g2o::EdgeProjectP2MC *edge = new g2o::EdgeProjectP2MC();
        edge->setVertex(0, point_vertex);
        edge->setVertex(1, cam_vertex);
        cv::Point2f mp_cv = left_struct->get_keypoint(matches_left[i]).pt;
        lvt_vector2 img_pt;
        img_pt << mp_cv.x, mp_cv.y;
        edge->setMeasurement(img_pt);
        edge->information() = Eigen::Matrix2d::Identity();
        g2o::RobustKernel *rkh = new g2o::RobustKernelCauchy;
        edge->setRobustKernel(rkh);
        rkh->setDelta(mono_chi);
        m_optimizer->addEdge(edge);
        mono_edges[i] = edge;
    }

    // perform optimzation
    std::vector<int> mono_inlier_marks(mono_edges.size(), 1);
    for (int i = 0; i < N_PASSES; i++)
    {
        m_optimizer->initializeOptimization(0);
        m_optimizer->optimize(5);
        for (int k = 0; k < mono_edges.size(); k++)
        {
            if (mono_edges[k]->chi2() > LVT_REPROJECTION_TH2)
            {
                mono_edges[k]->setLevel(1);
                mono_inlier_marks[k] = 0;
            }
        }
    }

    // retrieve optimized pose
    const Eigen::Vector3d op_position = cam_vertex->estimate().translation();
    const Eigen::Quaterniond op_orientation = cam_vertex->estimate().rotation();
    lvt_pose op_pose = lvt_pose(op_position, op_orientation);

    m_optimizer->clear();
    LVT_LOG("Finished computing pose.");
    LVT_RECORD("inlier count", std::accumulate(mono_inlier_marks.begin(), mono_inlier_marks.end(), 0));
    return op_pose;
}
