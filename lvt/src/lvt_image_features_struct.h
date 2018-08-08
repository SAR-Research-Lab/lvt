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

#ifndef LVT_IMAGE_FEATURES_STRUCT_H__
#define LVT_IMAGE_FEATURES_STRUCT_H__

#include "lvt_pose.h"
#include <opencv2/opencv.hpp>
#include <vector>

/*
 * A container for computed image features and their descriptors.
 */

class lvt_image_features_struct
{
  public:
    lvt_image_features_struct();
    void init(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, const cv::Mat &descs,
              int tracking_radius, int hashing_cell_size, int vertical_search_radius, float triangulation_ratio_th,
              float tracking_ratio_th, float desc_dist_th, std::vector<float> *kps_depth = nullptr);

    int find_match_index(const lvt_vector2 &pt, const cv::Mat &desc, float *d1, float *d2) const; // find the best feature in the struct that matches the passed one and return its index, or -1 otherwise. (ratio is the ratio to use for the ratio test).
    int row_match(const cv::Point2f &pt, const cv::Mat &desc) const;

    inline cv::Mat get_descriptor(const int index) const { return m_descriptors.row(index); }
    inline const cv::KeyPoint &get_keypoint(const int index) const { return m_keypoints[index]; }
    inline int get_features_count() const { return (int)m_keypoints.size(); }

    void mark_as_matched(const int idx, const bool val) { m_matched_marks[idx] = val; }
    bool is_matched(const int idx) const { return m_matched_marks[idx]; }
    void reset_matched_marks() { m_matched_marks = std::vector<bool>(m_keypoints.size(), false); }

    void set_tracking_radius(int tracking_radius) { m_tracking_radius = tracking_radius; }
    int get_tracking_radius() const { return m_tracking_radius; }

    bool is_depth_associated() const { return !m_kps_depths.empty(); }
    float get_keypoint_depth(const int idx) const { return m_kps_depths[idx]; }

  private:
    std::vector<cv::KeyPoint> m_keypoints;
    cv::Mat m_descriptors;
    cv::Ptr<cv::DescriptorMatcher> m_matcher;
    std::vector<bool> m_matched_marks;
    int m_cell_size;
    int m_cell_count_x, m_cell_count_y;
    int m_cell_search_radius;
    int m_tracking_radius;
    int m_img_rows, m_img_cols;
    int m_vertical_search_radius;
    float m_triangulation_ratio_th;
    float m_tracking_ratio_th;
    float m_desc_dist_th;
    std::vector<float> m_kps_depths;

    typedef std::vector<int> index_list_t;
    std::vector<std::vector<index_list_t>> m_index_hashmap;

    typedef std::pair<int, int> index_pair_t;
    inline index_pair_t compute_hashed_index(const cv::Point2f &val, const float cell_size) const
    {
        return index_pair_t(floor(val.y / cell_size), floor(val.x / cell_size));
    }
};

#endif //LVT_IMAGE_FEATURES_STRUCT_H__
