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

#ifndef LVT_VISUALIZATION_H__
#define LVT_VISUALIZATION_H__

#include "lvt_pose.h"
#include <opencv2/core.hpp>
#include <vector>

class lvt_local_map;
class lvt_image_features_struct;

namespace pangolin
{
class OpenGlRenderState;
class View;
class OpenGlMatrix;
} // namespace pangolin

class lvt_visualization
{
  public:
    lvt_visualization(float camera_size, int point_size);
    ~lvt_visualization();
    void reset();

    void set_map(lvt_local_map *local_map) { m_local_map = local_map; }

    void begin_adding_matches();
    void add_match(int map_point_idx, int image_struct_idx);
    void finish_adding_matches();
    cv::Mat draw_features(const cv::Mat &left_img, lvt_image_features_struct *left_struct);
    void display_features(const cv::Mat &left_img, lvt_image_features_struct *left_struct);

    void init_map_viewer();
    void destroy_map_viewer();
    void push_camera_pose(const lvt_pose &cam_pose);
    bool process_events(); // return true if the user requested to quit

  private:
    void draw_map();
    void draw_current_frame(const pangolin::OpenGlMatrix &cam_gl_mtrx);
    lvt_local_map *m_local_map;
    bool m_is_adding_features;
    std::vector<int> m_map_indices;
    std::vector<int> m_img_indices;
    std::vector<lvt_pose> m_cam_poses;
    pangolin::OpenGlRenderState *m_state_cam;
    pangolin::View *m_display_cam;
    pangolin::OpenGlMatrix *m_cam_gl_mtrx;
    float m_camera_size, m_current_camera_size;
    int m_point_size;
};

#endif //LVT_VISUALIZATION_H__
