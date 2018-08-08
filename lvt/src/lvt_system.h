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

#ifndef LVT_VISUAL_ODOMETRY_SYSTEM_H__
#define LVT_VISUAL_ODOMETRY_SYSTEM_H__

#include "lvt_definitions.h"
#include "lvt_pose.h"
#include "lvt_parameters.h"
#include <opencv2/core/core.hpp>
#include <deque>

class lvt_image_features_struct;

/*
 * This is the main visual odometry class
 */

class lvt_system
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    enum eState
    {
        eState_NOT_INITIALIZED = 1,
        eState_TRACKING,
        eState_LOST
    };
    enum eSensor
    {
        eSensor_STEREO = 1,
        eSensor_RGBD
    };

    static lvt_system *create(const lvt_parameters &params, eSensor sensor_type);
    static void destroy(lvt_system *);
    void reset();

    // In stereo case: We assume a stereo rectified grayscale input images.
    // In RGBD case: img1 is the grayscale image while img2 is corresponding depth image of type CV_32F
    lvt_pose track(cv::Mat img1, cv::Mat img2);

    lvt_pose track_with_external_corners(cv::Mat left_image, cv::Mat right_image,
                                         std::vector<cv::Point2f> &corners_locations_left, std::vector<cv::Point2f> &corners_locations_right); // This was a requested feature and added for some experiments

    inline lvt_system::eSensor get_sensor_type() const { return m_sensor; }
    inline lvt_system::eState get_state() const { return m_state; }
    inline bool should_quit() const { return m_should_quit; }

    // disable copying
    lvt_system(const lvt_system &) = delete;
    lvt_system &operator=(const lvt_system &) = delete;

  private:
    lvt_system();
    ~lvt_system();

    lvt_pose perform_tracking(const lvt_pose &estimated_pose, lvt_image_features_struct *left_struct,
                              lvt_image_features_struct *right_struct, bool *is_tracking, const cv::Mat *left_img_dbg = nullptr);

    bool need_new_triangulation();
    bool triangulation_policy_decreasing_matches();
    bool triangulation_policy_always_triangulate();

    void register_measurments_values();

    class lvt_local_map *m_local_map;
    class lvt_image_features_handler *m_features_handler;
    class lvt_motion_model *m_motion_model;
    class lvt_pnp_solver *m_pnp_solver;
    class lvt_visualization *m_visualization;
    class lvt_log *_the_log;
    class lvt_value_recorder *_the_value_recorder;
    lvt_parameters m_params;
    lvt_pose m_last_pose;
    int m_frame_number;
    eSensor m_sensor;
    bool (lvt_system::*triangulation_policy)();
    enum
    {
        N_MATCHES_WINDOWS = 3
    };
    std::deque<int> m_last_matches;
    eState m_state;
    bool m_should_quit; // flag set to true from the visualization if the user requested to quit
};

#endif //LVT_VISUAL_ODOMETRY_SYSTEM_H__
