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

#define LVT_EXPORT_FUNCTIONS

#include "lvt_c.h"
#include "lvt_system.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>

LVT_API lvt_handle lvt_create(const char *config_file_name, int sensor_type)
{
    lvt_system *vo = NULL;
    try
    {
        lvt_parameters params;
        if (params.init_from_file(config_file_name) && (sensor_type == 1 || sensor_type == 2))
        {
            vo = lvt_system::create(params, static_cast<lvt_system::eSensor>(sensor_type));
        }
    }
    catch (...)
    {
    }
    return static_cast<lvt_handle>(vo);
}

LVT_API void lvt_destroy(lvt_handle vo_system)
{
    assert(vo_system != NULL);
    try
    {
        lvt_system *vo = static_cast<lvt_system *>(vo_system);
        lvt_system::destroy(vo);
    }
    catch (...)
    {
    }
}

LVT_API void lvt_track(lvt_handle vo_system, unsigned char *left_img_data, unsigned char *right_img_data, int n_rows, int n_cols, double R[3][3], double t[3])
{
    assert(vo_system != NULL);
    try
    {
        lvt_system *vo = static_cast<lvt_system *>(vo_system);
        cv::Mat img_left(n_rows, n_cols, CV_8UC1, left_img_data);
        cv::Mat img_right(n_rows, n_cols, CV_8UC1, right_img_data);
        lvt_pose pose = vo->track(img_left, img_right);
        const lvt_matrix33 rot_mtrx = pose.get_orientation_matrix();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R[i][j] = rot_mtrx(i, j);
            }
        }
        const lvt_vector3 pos = pose.get_position();
        t[0] = pos.x();
        t[1] = pos.y();
        t[2] = pos.z();
    }
    catch (...)
    {
    }
}

LVT_API void lvt_track_with_external_corners(lvt_handle vo_system, unsigned char *left_img_data, unsigned char *right_img_data, int n_rows, int n_cols,
                                             double corners_left_data[][2], int n_corners_left, double corners_right_data[][2], int n_corners_right, double R[3][3], double t[3])
{
    assert(vo_system != NULL);
    try
    {
        lvt_system *vo = static_cast<lvt_system *>(vo_system);
        cv::Mat img_left(n_rows, n_cols, CV_8UC1, left_img_data);
        cv::Mat img_right(n_rows, n_cols, CV_8UC1, right_img_data);
        std::vector<cv::Point2f> corners_left, corners_right;
        corners_left.reserve(n_corners_left);
        corners_right.reserve(n_corners_right);
        for (int i = 0; i < n_corners_left; i++)
        {
            cv::Point2f pt;
            pt.x = corners_left_data[i][0];
            pt.y = corners_left_data[i][1];
            corners_left.push_back(pt);
        }
        for (int i = 0; i < n_corners_right; i++)
        {
            cv::Point2f pt;
            pt.x = corners_right_data[i][0];
            pt.y = corners_right_data[i][1];
            corners_right.push_back(pt);
        }

        lvt_pose pose = vo->track_with_external_corners(img_left, img_right, corners_left, corners_right);
        const lvt_matrix33 rot_mtrx = pose.get_orientation_matrix();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R[i][j] = rot_mtrx(i, j);
            }
        }
        const lvt_vector3 pos = pose.get_position();
        t[0] = pos.x();
        t[1] = pos.y();
        t[2] = pos.z();
    }
    catch (...)
    {
    }
}

LVT_API int lvt_get_status(lvt_handle vo_system)
{
    assert(vo_system != NULL);
    try
    {
        lvt_system *vo = static_cast<lvt_system *>(vo_system);
        return static_cast<int>(vo->get_state());
    }
    catch (...)
    {
    }
    return -1;
}
