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

#include "lvt_image_features_handler.h"
#include "lvt_logging_utils.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <limits>
#include <thread>

// This function is from code in this answer: http://answers.opencv.org/question/93317/orb-keypoints-distribution-over-an-image/
static void _adaptive_non_maximal_suppresion(std::vector<cv::KeyPoint> &keypoints, const int num_to_keep,
                                             const float tx, const float ty)
{
    // Sort by response
    std::sort(keypoints.begin(), keypoints.end(),
              [&keypoints](const cv::KeyPoint &lhs, const cv::KeyPoint &rhs) {
                  return lhs.response > rhs.response;
              });

    std::vector<cv::KeyPoint> anmsPts;
    anmsPts.reserve(num_to_keep);

    std::vector<float> radii;
    radii.resize(keypoints.size());
    std::vector<float> radiiSorted;
    radiiSorted.resize(keypoints.size());

    const float robustCoeff = 1.11;
    for (int i = 0, count_i = keypoints.size(); i < count_i; i++)
    {
        const float response = keypoints[i].response * robustCoeff;
        float radius = (std::numeric_limits<float>::max)();
        for (int j = 0; j < i && keypoints[j].response > response; j++)
        {
            const cv::Point2f diff_pt = keypoints[i].pt - keypoints[j].pt;
            radius = (std::min)(radius, diff_pt.x * diff_pt.x + diff_pt.y * diff_pt.y);
        }
        radius = sqrtf(radius);
        radii[i] = radius;
        radiiSorted[i] = radius;
    }

    std::sort(radiiSorted.begin(), radiiSorted.end(),
              [&radiiSorted](const float &lhs, const float &rhs) {
                  return lhs > rhs;
              });

    const float decisionRadius = radiiSorted[num_to_keep];
    for (int i = 0, count = radii.size(); i < count; i++)
    {
        if (radii[i] >= decisionRadius)
        {
            keypoints[i].pt.x += tx;
            keypoints[i].pt.y += ty;
            anmsPts.push_back(keypoints[i]);
        }
    }

    anmsPts.swap(keypoints);
}

lvt_image_features_handler::lvt_image_features_handler(const lvt_parameters &vo_params)
    : m_vo_params(vo_params)
{
    assert(m_vo_params.img_height > 0);
    assert(m_vo_params.img_width > 0);
    assert(m_vo_params.detection_cell_size > 0);
    assert(m_vo_params.max_keypoints_per_cell > 0);
    assert(m_vo_params.tracking_radius > 0);
    assert(m_vo_params.agast_threshold > 0);

    int num_cells_y = 1 + ((m_vo_params.img_height - 1) / m_vo_params.detection_cell_size);
    int num_cells_x = 1 + ((m_vo_params.img_width - 1) / m_vo_params.detection_cell_size);
    int s = m_vo_params.detection_cell_size;
    for (int i = 0; i < num_cells_y; i++)
    {
        for (int k = 0; k < num_cells_x; k++)
        {
            int sy = s;
            if ((i == num_cells_y - 1) && ((i + 1) * s > m_vo_params.img_height))
            {
                sy = m_vo_params.img_height - (i * s);
            }
            int sx = s;
            if ((k == num_cells_x - 1) && ((k + 1) * s > m_vo_params.img_width))
            {
                sx = m_vo_params.img_width - (k * s);
            }
            m_sub_imgs_rects.push_back(cv::Rect(k * s, i * s, sx, sy));
        }
    }

    m_th_data[0].detector = cv::AgastFeatureDetector::create(m_vo_params.agast_threshold);
    m_th_data[0].extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    m_th_data[0].sub_imgs_rects = m_sub_imgs_rects;
    m_th_data[0].vo_params = &m_vo_params;

    m_th_data[1].detector = cv::AgastFeatureDetector::create(m_vo_params.agast_threshold);
    m_th_data[1].extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    m_th_data[1].sub_imgs_rects = m_sub_imgs_rects;
    m_th_data[1].vo_params = &m_vo_params;
}

lvt_image_features_handler::~lvt_image_features_handler()
{
}

static void perform_detect_corners(compute_features_data *p, std::vector<cv::KeyPoint> *all_keypoints)
{
    for (int r = 0; r < p->sub_imgs_rects.size(); r++)
    {
        cv::Rect rect = p->sub_imgs_rects[r];
        cv::Mat sub_img = p->img(rect);
        std::vector<cv::KeyPoint> keypoints;
        keypoints.reserve(p->vo_params->max_keypoints_per_cell);
        p->detector->detect(sub_img, keypoints);
        if (keypoints.size() > p->vo_params->max_keypoints_per_cell)
        {
            _adaptive_non_maximal_suppresion(keypoints, p->vo_params->max_keypoints_per_cell, (float)rect.x, (float)rect.y);
        }
        else
        {
            for (int i = 0; i < keypoints.size(); i++)
            {
                keypoints[i].pt.x += (float)rect.x;
                keypoints[i].pt.y += (float)rect.y;
            }
        }
        all_keypoints->insert(all_keypoints->end(), keypoints.begin(), keypoints.end());
    }
}

void lvt_image_features_handler::perform_compute_features(compute_features_data *p)
{
    std::vector<cv::KeyPoint> all_keypoints;
    all_keypoints.reserve(p->sub_imgs_rects.size() * p->vo_params->max_keypoints_per_cell);
    perform_detect_corners(p, &all_keypoints);
    if (all_keypoints.size() < LVT_CORNERS_LOW_TH)
    {
        all_keypoints.clear();
        int original_agast_th = p->detector->getThreshold();
        int lowered_agast_th = (double)original_agast_th * 0.5 + 0.5;
        p->detector->setThreshold(lowered_agast_th);
        perform_detect_corners(p, &all_keypoints);
        p->detector->setThreshold(original_agast_th);
    }

    cv::Mat desc;
    p->extractor->compute(p->img, all_keypoints, desc);
    p->features_struct->init(p->img, all_keypoints, desc, p->vo_params->tracking_radius, LVT_HASHING_CELL_SIZE,
                             LVT_ROW_MATCHING_VERTICAL_SEARCH_RADIUS, p->vo_params->triangulation_ratio_test_threshold,
                             p->vo_params->tracking_ratio_test_threshold, p->vo_params->descriptor_matching_threshold);
}

void lvt_image_features_handler::perform_compute_descriptors_only(compute_features_data *p)
{
    cv::Mat desc;
    const std::vector<cv::Point2f> &ext_kp = *(p->ext_kp);
    std::vector<cv::KeyPoint> keypoints;
    keypoints.reserve(ext_kp.size());
    for (int i = 0, count = ext_kp.size(); i < count; i++)
    {
        cv::KeyPoint kp;
        kp.pt = ext_kp[i];
        keypoints.push_back(kp);
    }
    p->extractor->compute(p->img, keypoints, desc);
    p->features_struct->init(p->img, keypoints, desc, p->vo_params->tracking_radius, LVT_HASHING_CELL_SIZE,
                             LVT_ROW_MATCHING_VERTICAL_SEARCH_RADIUS, p->vo_params->triangulation_ratio_test_threshold,
                             p->vo_params->tracking_ratio_test_threshold, p->vo_params->descriptor_matching_threshold);
}

void lvt_image_features_handler::compute_features(const cv::Mat &img_left, const cv::Mat &img_right,
                                                  lvt_image_features_struct *out_left, lvt_image_features_struct *out_right)
{
    // Compute left image features on the main thread while the right one in a parallel thread.
    m_th_data[0].img = img_left;
    m_th_data[0].features_struct = out_left;
    m_th_data[1].img = img_right;
    m_th_data[1].features_struct = out_right;
    std::thread th(&lvt_image_features_handler::perform_compute_features, this, &(m_th_data[1]));
    perform_compute_features(&(m_th_data[0]));
    th.join();
    m_th_data[0].img = cv::Mat();
    m_th_data[1].img = cv::Mat();
}

void lvt_image_features_handler::compute_descriptors_only(const cv::Mat &img_left, std::vector<cv::Point2f> &ext_kp_left, lvt_image_features_struct *out_left,
                                                          const cv::Mat &img_right, std::vector<cv::Point2f> &ext_kp_right, lvt_image_features_struct *out_right)
{
    m_th_data[0].img = img_left;
    m_th_data[0].ext_kp = &ext_kp_left;
    m_th_data[0].features_struct = out_left;
    m_th_data[1].img = img_right;
    m_th_data[1].ext_kp = &ext_kp_right;
    m_th_data[1].features_struct = out_right;
    std::thread th(&lvt_image_features_handler::perform_compute_descriptors_only, this, &(m_th_data[1]));
    perform_compute_descriptors_only(&(m_th_data[0]));
    th.join();
    m_th_data[0].img = cv::Mat();
    m_th_data[1].img = cv::Mat();
}

void lvt_image_features_handler::compute_features_rgbd(const cv::Mat &img_gray, const cv::Mat &in_img_depth, lvt_image_features_struct *out_struct)
{
    // detect corners in the image as normal
    m_th_data[0].img = img_gray;
    compute_features_data *p = &m_th_data[0];
    std::vector<cv::KeyPoint> all_keypoints;
    all_keypoints.reserve(p->sub_imgs_rects.size() * p->vo_params->max_keypoints_per_cell);
    perform_detect_corners(p, &all_keypoints);
    if (all_keypoints.size() < LVT_CORNERS_LOW_TH)
    {
        all_keypoints.clear();
        int original_agast_th = p->detector->getThreshold();
        int lowered_agast_th = (double)original_agast_th * 0.5 + 0.5;
        p->detector->setThreshold(lowered_agast_th);
        perform_detect_corners(p, &all_keypoints);
        p->detector->setThreshold(original_agast_th);
    }

    // compute descriptors
    cv::Mat desc;
    p->extractor->compute(p->img, all_keypoints, desc);

    // retain corners with valid depth values
    std::vector<float> kps_depths;
    std::vector<cv::KeyPoint> filtered_kps;
    cv::Mat filtered_desc;
    kps_depths.reserve(all_keypoints.size());
    filtered_kps.reserve(all_keypoints.size());
    for (int i = 0; i < all_keypoints.size(); i++)
    {
        const cv::KeyPoint &kp = all_keypoints[i];
        const float d = in_img_depth.at<float>(kp.pt.y, kp.pt.x);
        if (d >= m_vo_params.near_plane_distance && d <= m_vo_params.far_plane_distance)
        {
            kps_depths.push_back(d);
            filtered_kps.push_back(kp);
            filtered_desc.push_back(desc.row(i).clone());
        }
    }

    // Undistort keypoints if the img is distorted
    if (fabs(m_vo_params.k1) > 1e-5)
    {
        cv::Mat kps_mat(filtered_kps.size(), 2, CV_32F);
        for (int i = 0; i < filtered_kps.size(); i++)
        {
            kps_mat.at<float>(i, 0) = filtered_kps[i].pt.x;
            kps_mat.at<float>(i, 1) = filtered_kps[i].pt.y;
        }
        kps_mat = kps_mat.reshape(2);
        cv::Matx33f intrinsics_mtrx(m_vo_params.fx, 0.0, m_vo_params.cx,
                                    0.0, m_vo_params.fy, m_vo_params.cy,
                                    0.0, 0.0, 1.0);
        std::vector<float> dist;
        dist.push_back(m_vo_params.k1);
        dist.push_back(m_vo_params.k2);
        dist.push_back(m_vo_params.p1);
        dist.push_back(m_vo_params.p2);
        dist.push_back(m_vo_params.k3);
        cv::undistortPoints(kps_mat, kps_mat, cv::Mat(intrinsics_mtrx), cv::Mat(dist), cv::Mat(), intrinsics_mtrx);
        kps_mat = kps_mat.reshape(1);
        for (int i = 0; i < filtered_kps.size(); i++)
        {
            cv::KeyPoint &kp = filtered_kps[i];
            kp.pt.x = kps_mat.at<float>(i, 0);
            kp.pt.y = kps_mat.at<float>(i, 1);
        }
    }

    // initialize output structs
    out_struct->init(img_gray, filtered_kps, filtered_desc, m_vo_params.tracking_radius, LVT_HASHING_CELL_SIZE,
                     LVT_ROW_MATCHING_VERTICAL_SEARCH_RADIUS, m_vo_params.triangulation_ratio_test_threshold,
                     m_vo_params.tracking_ratio_test_threshold, m_vo_params.descriptor_matching_threshold, &kps_depths);
}

void lvt_image_features_handler::row_match(lvt_image_features_struct *features_left, lvt_image_features_struct *features_right,
                                           std::vector<cv::DMatch> *out_matches)
{
    for (int i = 0, count = features_left->get_features_count(); i < count; i++)
    {
        if (features_left->is_matched(i))
        { // if the feature in the left camera image is matched from tracking then ignore it
            continue;
        }
        cv::Mat desc = features_left->get_descriptor(i);
        const int match_idx = features_right->row_match(features_left->get_keypoint(i).pt, desc);
        if (match_idx != -1)
        {
            cv::DMatch m;
            m.queryIdx = i;
            m.trainIdx = match_idx;
            out_matches->push_back(m);
            features_left->mark_as_matched(i, true);
            features_right->mark_as_matched(match_idx, true);
        }
    }
}
