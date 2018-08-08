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

#include "lvt_parameters.h"
#include <opencv2/core/core.hpp>

lvt_parameters::lvt_parameters()
{
    fx = fy = cx = cy = 0.5;
    img_width = img_height = 0.0;
    k1 = k2 = p1 = p2 = k3 = 0.f;
    baseline = 0.f;
    near_plane_distance = 0.1;
    far_plane_distance = 500.0;
    triangulation_ratio_test_threshold = 0.60;
    tracking_ratio_test_threshold = 0.80;
    descriptor_matching_threshold = 30.0;
    min_num_matches_for_tracking = 10;
    tracking_radius = 25;
    agast_threshold = 25;
    untracked_threshold = 10;
    staged_threshold = 2;
    detection_cell_size = 250;
    max_keypoints_per_cell = 150;
    triangulation_policy = etriangulation_policy_decreasing_matches;
    enable_logging = true;
    enable_visualization = false;
    viewer_camera_size = 0.6;
    viewer_point_size = 5;
}

bool lvt_parameters::init_from_file(const char *config_file_name)
{
    cv::FileStorage config_file(config_file_name, cv::FileStorage::READ);
    if (!config_file.isOpened())
    {
        return false;
    }

    fx = config_file["fx"];
    fy = config_file["fy"];
    cx = config_file["cx"];
    cy = config_file["cy"];
    k1 = config_file["k1"];
    k2 = config_file["k2"];
    p1 = config_file["p1"];
    p2 = config_file["p2"];
    k3 = config_file["k3"];
    baseline = config_file["baseline"];
    img_width = config_file["img_width"];
    img_height = config_file["img_height"];
    near_plane_distance = config_file["near_plane_distance"];
    far_plane_distance = config_file["far_plane_distance"];
    triangulation_ratio_test_threshold = config_file["triangulation_ratio_test_threshold"];
    tracking_ratio_test_threshold = config_file["tracking_ratio_test_threshold"];
    min_num_matches_for_tracking = config_file["min_num_matches_for_tracking"];
    tracking_radius = config_file["tracking_radius"];
    agast_threshold = config_file["agast_threshold"];
    untracked_threshold = config_file["untracked_threshold"];
    staged_threshold = config_file["staged_threshold"];
    descriptor_matching_threshold = config_file["descriptor_matching_threshold"];
    detection_cell_size = config_file["detection_cell_size"];
    max_keypoints_per_cell = config_file["max_keypoints_per_cell"];
    enable_logging = (int)config_file["enable_logging"];
    enable_visualization = (int)config_file["enable_visualization"];
    triangulation_policy = config_file["triangulation_policy"];
    viewer_camera_size = config_file["viewer_camera_size"];
    viewer_point_size = config_file["viewer_point_size"];
    config_file.release();
    return true;
}
