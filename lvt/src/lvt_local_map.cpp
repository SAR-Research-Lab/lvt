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

#include "lvt_local_map.h"
#include "lvt_image_features_handler.h"
#include "lvt_image_features_struct.h"
#include "lvt_visualization.h"
#include "lvt_logging_utils.h"
#include <cmath>
#include <Eigen/SVD>

#ifdef LVT_ENABLE_VISUALIZATION

#define VIS_BEGIN_ADDING_MATCHES()               \
    if (m_vo_params.enable_visualization)        \
    {                                            \
        m_visualization->begin_adding_matches(); \
    }

#define VIS_ADD_MATCH(_mp, _ip)               \
    if (m_vo_params.enable_visualization)     \
    {                                         \
        m_visualization->add_match(_mp, _ip); \
    }

#define VIS_FINISH_ADDING_MATCHES()               \
    if (m_vo_params.enable_visualization)         \
    {                                             \
        m_visualization->finish_adding_matches(); \
    }

#else
#define VIS_BEGIN_ADDING_MATCHES() ((void)0)
#define VIS_ADD_MATCH(_mp, _ip) ((void)0)
#define VIS_FINISH_ADDING_MATCHES() ((void)0)
#endif //LVT_ENABLE_VISUALIZATION

static float min_x, max_x, min_y, max_y;

static inline bool is_point_visible(const lvt_vector3 &pt, const lvt_matrix34 &w2c, const lvt_parameters &params,
                                    lvt_vector2 &out_projected_pt)
{
    lvt_vector4 pt_h;
    pt_h << pt, 1.0;
    lvt_vector3 pt_cam = w2c * pt_h;
    if (pt_cam.z() < params.near_plane_distance || pt_cam.z() > params.far_plane_distance)
    {
        return false;
    }
    const double inv_z = 1.0 / pt_cam.z();
    const double u = params.fx * pt_cam.x() * inv_z + params.cx;
    const double v = params.fy * pt_cam.y() * inv_z + params.cy;
    if (u < min_x || u > max_x ||
        v < min_y || v > max_y)
    {
        return false;
    }
    out_projected_pt << u, v;
    return true;
}

lvt_local_map::lvt_local_map(const lvt_parameters &vo_params,
                             lvt_image_features_handler *features_handler, lvt_visualization *vis) : m_vo_params(vo_params), m_features_handler(features_handler), m_visualization(vis)
{
    // compute image bounds
    if (fabs(m_vo_params.k1) < 1e-5)
    {
        min_x = 0.0;
        max_x = m_vo_params.img_width;
        min_y = 0.0;
        max_y = m_vo_params.img_height;
    }
    else
    {
        cv::Mat kps_mat(4, 2, CV_32F);
        kps_mat.at<float>(0, 0) = 0.0;
        kps_mat.at<float>(0, 1) = 0.0;
        kps_mat.at<float>(1, 0) = m_vo_params.img_width;
        kps_mat.at<float>(1, 1) = 0.0;
        kps_mat.at<float>(2, 0) = 0.0;
        kps_mat.at<float>(2, 1) = m_vo_params.img_height;
        kps_mat.at<float>(3, 0) = m_vo_params.img_width;
        kps_mat.at<float>(3, 1) = m_vo_params.img_height;
        cv::Matx33f intrinsics_mtrx(m_vo_params.fx, 0.0, m_vo_params.cx,
                                    0.0, m_vo_params.fy, m_vo_params.cy,
                                    0.0, 0.0, 1.0);
        std::vector<float> dist;
        dist.push_back(m_vo_params.k1);
        dist.push_back(m_vo_params.k2);
        dist.push_back(m_vo_params.p1);
        dist.push_back(m_vo_params.p2);
        dist.push_back(m_vo_params.k3);
        kps_mat = kps_mat.reshape(2);
        cv::undistortPoints(kps_mat, kps_mat, cv::Mat(intrinsics_mtrx), cv::Mat(dist), cv::Mat(), intrinsics_mtrx);
        kps_mat = kps_mat.reshape(1);
        min_x = std::min(kps_mat.at<float>(0, 0), kps_mat.at<float>(2, 0));
        max_x = std::max(kps_mat.at<float>(1, 0), kps_mat.at<float>(3, 0));
        min_y = std::min(kps_mat.at<float>(0, 1), kps_mat.at<float>(1, 1));
        max_y = std::max(kps_mat.at<float>(2, 1), kps_mat.at<float>(3, 1));
    }
}

lvt_local_map::~lvt_local_map()
{
    reset();
}

void lvt_local_map::reset()
{
    m_map_points.clear();
    m_staged_points.clear();
}

int lvt_local_map::find_matches(const lvt_pose &cam_pose, lvt_image_features_struct *left_struct,
                                lvt_vector3_array *out_map_points, std::vector<int> *out_matches_left)
{
    const lvt_matrix34 cml = lvt_pose_utils::compute_world_to_camera_transform(cam_pose);
    int matches_count = 0;
    VIS_BEGIN_ADDING_MATCHES();
    std::vector<int> matches(m_map_points.size(), -2); // mark each map point with its matching index from left_struct, -2 if not visible
    lvt_vector2_array projections(m_map_points.size());
#ifdef LVT_ENABLE_MEASURMENT_RECORD
    std::vector<float> d1s(m_map_points.size());
    std::vector<float> d2s(m_map_points.size());
#endif //LVT_ENABLE_MEASURMENT_RECORD

    for (int i = 0; i < m_map_points.size(); i++)
    {
        lvt_vector2 proj_pt_left;
        if (!is_point_visible(m_map_points[i].m_position, cml, m_vo_params, proj_pt_left))
        {
            m_map_points[i].m_counter += 1;
            matches[i] = -2;
            continue;
        }
        projections[i] = proj_pt_left;
        float d1, d2;
        int match_idx_left = left_struct->find_match_index(proj_pt_left, m_map_points[i].m_descriptor, &d1, &d2);
        matches[i] = match_idx_left;
        if (match_idx_left != -1)
        {
            matches_count++;
            left_struct->mark_as_matched(match_idx_left, true);
#ifdef LVT_ENABLE_MEASURMENT_RECORD
            d1s[i] = d1;
            d2s[i] = d2;
#endif //LVT_ENABLE_MEASURMENT_RECORD
        }
    }

    if (matches_count < LVT_N_MATCHES_TH)
    {
        matches_count = 0;
        left_struct->reset_matched_marks();
        int original_tracking_radius = left_struct->get_tracking_radius();
        left_struct->set_tracking_radius(2 * original_tracking_radius);
        for (int i = 0; i < m_map_points.size(); i++)
        {
            if (matches[i] == -2)
            {
                continue;
            }
            float d1, d2;
            int match_idx_left = left_struct->find_match_index(projections[i], m_map_points[i].m_descriptor, &d1, &d2);
            matches[i] = match_idx_left;
            if (match_idx_left != -1)
            {
                matches_count++;
                left_struct->mark_as_matched(match_idx_left, true);
#ifdef LVT_ENABLE_MEASURMENT_RECORD
                d1s[i] = d1;
                d2s[i] = d2;
#endif //LVT_ENABLE_MEASURMENT_RECORD
            }
        }
        left_struct->set_tracking_radius(original_tracking_radius);
    }

    for (int i = 0; i < matches.size(); i++)
    {
        m_map_points[i].m_match_idx = matches[i];
        if (matches[i] == -2)
        {
            continue;
        }

        if (matches[i] == -1)
        {
            m_map_points[i].m_counter += 1;
            continue;
        }

        m_map_points[i].m_age += 1;
        out_map_points->push_back(m_map_points[i].m_position);
        out_matches_left->push_back(matches[i]);
        LVT_RECORD("img feature x", left_struct->get_keypoint(matches[i]).pt.x);
        LVT_RECORD("img feature y", left_struct->get_keypoint(matches[i]).pt.y);
        LVT_RECORD("age", m_map_points[i].m_age);
        LVT_RECORD("closest descriptor distance", d1s[i]);
        LVT_RECORD("second descriptor distance", d2s[i]);
        VIS_ADD_MATCH(i, matches[i]);
    }

    VIS_FINISH_ADDING_MATCHES();
    LVT_RECORD("tracked map points", matches_count);
    return matches_count;
}

void lvt_local_map::triangulate_rgbd(const lvt_pose &cam_pose, lvt_image_features_struct *img_struct, lvt_map_point_array *out_points)
{
    const float inv_fx = 1.0f / m_vo_params.fx;
    const float inv_fy = 1.0f / m_vo_params.fy;
    lvt_matrix34 cam_to_world_mtrx;
    cam_to_world_mtrx << cam_pose.get_orientation_matrix(), cam_pose.get_position();
    for (int i = 0, count = img_struct->get_features_count(); i < count; i++)
    {
        const cv::Point2f pt = img_struct->get_keypoint(i).pt;
        const float u = pt.x;
        const float v = pt.y;
        const float z = img_struct->get_keypoint_depth(i);
        const float x = (u - m_vo_params.cx) * z * inv_fx;
        const float y = (v - m_vo_params.cy) * z * inv_fy;
        lvt_vector4 pt_h;
        pt_h << x, y, z, 1.0;
        lvt_vector3 pt_w = cam_to_world_mtrx * pt_h;

        lvt_map_point mp;
        mp.m_position = pt_w;
        mp.m_descriptor = img_struct->get_descriptor(i);
        mp.m_counter = 0;
        mp.m_age = 0;
        out_points->push_back(mp);
    }
}

void lvt_local_map::triangulate(const lvt_pose &cam_pose, lvt_image_features_struct *left_struct,
                                lvt_image_features_struct *right_struct, lvt_map_point_array *out_points)
{
    std::vector<cv::DMatch> matches;
    m_features_handler->row_match(left_struct, right_struct, &matches);
    if (matches.empty())
    {
        LVT_ASSERT(0 && "No row matches for triangulation found.");
        return;
    }

    const lvt_pose cam_pose_right = lvt_pose_utils::compute_right_camera_pose(cam_pose, m_vo_params.baseline);
    const lvt_matrix34 cml = lvt_pose_utils::compute_world_to_camera_transform(cam_pose);
    const lvt_matrix34 cmr = lvt_pose_utils::compute_world_to_camera_transform(cam_pose_right);
    const double cx = m_vo_params.cx, cy = m_vo_params.cy; // , f_inv = 1.0 / m_vo_params.focal_length;
    const double inv_fx = 1.0 / m_vo_params.fx, inv_fy = 1.0 / m_vo_params.fy;
    out_points->reserve(matches.size());

    for (size_t i = 0, count = matches.size(); i < count; i++)
    {
        const cv::Point2f u1 = left_struct->get_keypoint(matches[i].queryIdx).pt;
        const cv::Point2f u2 = right_struct->get_keypoint(matches[i].trainIdx).pt;
        double u1_x = (u1.x - cx) * inv_fx;
        double u1_y = (u1.y - cy) * inv_fy;
        double u2_x = (u2.x - cx) * inv_fx;
        double u2_y = (u2.y - cy) * inv_fy;

        // Linear-LS triangulation
        Eigen::Matrix<double, 4, 4> A;
        A << u1_x * cml.row(2) - cml.row(0),
            u1_y * cml.row(2) - cml.row(1),
            u2_x * cmr.row(2) - cmr.row(0),
            u2_y * cmr.row(2) - cmr.row(1);

        lvt_vector3 world_pt = A.leftCols<3>().jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(-A.col(3));
        assert(std::isfinite(world_pt.x()) && std::isfinite(world_pt.y()) && std::isfinite(world_pt.z()));

        // check if the point is in viewable region by camera
        lvt_vector2 proj_pt_l, proj_pt_r;
        if (!is_point_visible(world_pt, cml, m_vo_params, proj_pt_l) || !is_point_visible(world_pt, cmr, m_vo_params, proj_pt_r))
        {
            continue;
        }

        // check reprojection error
        {
            double err_x = proj_pt_l.x() - u1.x;
            double err_y = proj_pt_l.y() - u1.y;
            if ((err_x * err_x + err_y * err_y) > LVT_REPROJECTION_TH2)
            {
                continue;
            }
        }

        {
            double err_x = proj_pt_r.x() - u2.x;
            double err_y = proj_pt_r.y() - u2.y;
            if ((err_x * err_x + err_y * err_y) > LVT_REPROJECTION_TH2)
            {
                continue;
            }
        }

        // create map point
        lvt_map_point mp;
        mp.m_position = world_pt;
        mp.m_descriptor = left_struct->get_descriptor(matches[i].queryIdx);
        mp.m_counter = 0;
        mp.m_age = 0;
        out_points->push_back(mp);
    }
}

void lvt_local_map::update_with_new_triangulation(const lvt_pose &cam_pose,
                                                  lvt_image_features_struct *left_struct, lvt_image_features_struct *right_struct, bool dont_stage)
{
    // Triangulate new map points from features that were not matched/tracked
    lvt_map_point_array new_triangulations;
    if (left_struct->is_depth_associated())
    {
        triangulate_rgbd(cam_pose, left_struct, &new_triangulations);
    }
    else
    {
        triangulate(cam_pose, left_struct, right_struct, &new_triangulations);
    }
    LVT_ASSERT(!new_triangulations.empty() && "Nothing was triangulated");
    if (dont_stage || m_vo_params.staged_threshold == 0 || get_map_size() < LVT_N_MAP_POINTS)
    {
        m_map_points.insert(m_map_points.end(), new_triangulations.begin(), new_triangulations.end());
    }
    else
    {
        m_staged_points.insert(m_staged_points.end(), new_triangulations.begin(), new_triangulations.end());
    }
}

void lvt_local_map::update_staged_map_points(const lvt_pose &cam_pose, lvt_image_features_struct *left_struct)
{
    const lvt_matrix34 cml = lvt_pose_utils::compute_world_to_camera_transform(cam_pose);
    int n_erased_points = 0;
    int n_upgraded_points = 0;
    std::set<lvt_map_point *> points_to_be_deleted;
    for (int i = 0, count = m_staged_points.size(); i < count; i++)
    {
        lvt_map_point *mp = &(m_staged_points[i]);
        const lvt_vector3 world_pt = mp->m_position;
        lvt_vector2 proj_pt_left;
        float d1, d2;
        int match_idx_left = -1;
        if (!is_point_visible(world_pt, cml, m_vo_params, proj_pt_left) ||
            (match_idx_left = left_struct->find_match_index(proj_pt_left, mp->m_descriptor, &d1, &d2)) == -1)
        {
            points_to_be_deleted.insert(mp);
            n_erased_points++;
            continue;
        }
        left_struct->mark_as_matched(match_idx_left, true);
        mp->m_counter += 1;
        if (mp->m_counter == m_vo_params.staged_threshold || get_map_size() < LVT_N_MAP_POINTS)
        {
            m_map_points.push_back(m_staged_points[i]);
            points_to_be_deleted.insert(mp);
            n_upgraded_points++;
        }
    }

    m_staged_points.erase(std::remove_if(m_staged_points.begin(), m_staged_points.end(),
                                         [&points_to_be_deleted](lvt_map_point &pt) { return points_to_be_deleted.count(&pt) != 0; }),
                          m_staged_points.end());

    LVT_LOG("Staged map points updated: " + std::to_string(n_erased_points) + " erased, " +
            std::to_string(n_upgraded_points) + " upgraded to map points, " + std::to_string(m_staged_points.size()) + " remain.");
}

void lvt_local_map::clean_untracked_points(lvt_image_features_struct *left_struct)
{
    const int th = m_vo_params.untracked_threshold;
    lvt_map_point_array cleaned_map_points;
    cleaned_map_points.reserve(m_map_points.size());
    for (int i = 0; i < m_map_points.size(); i++)
    {
        if (m_map_points[i].m_counter >= th)
        {
            if (m_map_points[i].m_match_idx >= 0)
            {
                left_struct->mark_as_matched(m_map_points[i].m_match_idx, false);
            }
        }
        else
        {
            cleaned_map_points.push_back(m_map_points[i]);
        }
    }
    cleaned_map_points.swap(m_map_points);
}
