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

#ifndef LVT_VISUAL_ODOMETRY_PARAMETERS_H__
#define LVT_VISUAL_ODOMETRY_PARAMETERS_H__

struct lvt_parameters
{
    lvt_parameters();
    bool init_from_file(const char *config_file_name);

    // The following parameters must be specified.
    // Stereo is assumed to be undistorted and rectified.
    float fx, fy, cx, cy;
    float baseline;
    int img_width, img_height;
    float k1, k2, p1, p2, k3; // distortion, only in rgbd case

    // The rest are optional.
    float near_plane_distance, far_plane_distance; // viewable region
    float triangulation_ratio_test_threshold;      // cutoff for the ratio test performed when row matching for triangulating new map points
    float tracking_ratio_test_threshold;           // cutoff for the ratio test when matching during tracking
    float descriptor_matching_threshold;           // when matching for triangulation or tracking, if only one candidate feature is found for the traget feature, ratio test cannot be performed and thus the candidate is declared a match if the descriptors distance is smaller than this value.
    int min_num_matches_for_tracking;              // minimum required matched for succesful tracking, otherwise tracking is considered lost
    int tracking_radius;                           // the radius in pixels around each projected map point when looking for its detected image feature. Default 25px
    int detection_cell_size;                       // To enhance detected features distrubution, the image is devided into cells and features detetin is attempted in each. This is the side length of each cell.
    int max_keypoints_per_cell;                    // maximum number of features to detect in each cell.
    int agast_threshold;                           // Threshold for AGAST corner detector used.
    int untracked_threshold;                       // When a map point is not tracked in untracked_threshold number of frames then it is removed from the map.
    int staged_threshold;                          // newly triangulated map points are initially put into a staging phase, if they were successfully tracked for staged_threshold number of frames then they are declared good and added to the local map to be used for pose estimation. If staged_threshold is set to zero then this feature is effectively disabled.
    bool enable_logging;
    bool enable_visualization;
    enum
    {
        etriangulation_policy_decreasing_matches = 1,
        etriangulation_policy_always_triangulate
    };
    int triangulation_policy;
    float viewer_camera_size;
    int viewer_point_size;
};

#endif //LVT_VISUAL_ODOMETRY_PARAMETERS_H__
