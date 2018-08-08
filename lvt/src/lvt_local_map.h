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

#ifndef LVT_LOCAL_MAP_H__
#define LVT_LOCAL_MAP_H__

#include "lvt_pose.h"
#include "lvt_parameters.h"
#include <opencv2/core/core.hpp>
#include <vector>

class lvt_image_features_struct;
class lvt_image_features_handler;
class lvt_visualization;

class lvt_local_map
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    lvt_local_map(const lvt_parameters &vo_params,
                  lvt_image_features_handler *features_handler, lvt_visualization *vis);
    ~lvt_local_map();
    void reset();

    int get_map_size() const { return m_map_points.size(); }
    int get_staged_points_count() const { return m_staged_points.size(); }

    int find_matches(const lvt_pose &cam_pose, lvt_image_features_struct *left_struct,
                     lvt_vector3_array *out_map_points, std::vector<int> *out_matches_left);

    void update_with_new_triangulation(const lvt_pose &cam_pose,
                                       lvt_image_features_struct *left_struct, lvt_image_features_struct *right_struct, bool dont_stage = false);

    void update_staged_map_points(const lvt_pose &cam_pose, lvt_image_features_struct *left_struct);
    void clean_untracked_points();

  private:
    LVT_ADD_LOGGING;
    LVT_ADD_MEASURMENTS_RECORDER;
    friend class lvt_visualization;

    struct lvt_map_point
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        cv::Mat m_descriptor;   // descriptor for the keypoint used initially to triangulate this map point
        lvt_vector3 m_position; // world position of this point
        int m_counter;          // depending on context, if the point is staged then this is the number of frames it has been tracked while staged. If it is a map point, then this is the number of times it failed tracking.
        int m_age;              // number of frames this map point was successfully tracked and thus used in pose estimation
    };

    typedef std::vector<lvt_map_point, Eigen::aligned_allocator<lvt_map_point>> lvt_map_point_array;

    void triangulate(const lvt_pose &cam_pose, lvt_image_features_struct *left_struct,
                     lvt_image_features_struct *right_struct, lvt_map_point_array *out_points);

    void triangulate_rgbd(const lvt_pose &cam_pose, lvt_image_features_struct *img_struct, lvt_map_point_array *out_points);

    lvt_parameters m_vo_params;
    lvt_image_features_handler *m_features_handler;
    lvt_visualization *m_visualization;
    lvt_map_point_array m_map_points;
    lvt_map_point_array m_staged_points;
};

#endif //LVT_LOCAL_MAP_H__
