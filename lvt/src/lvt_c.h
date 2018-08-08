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

#ifndef LVT_C_INTERFACE_H__
#define LVT_C_INTERFACE_H__

/*
 * C-interface for the vo system. This is intended to be built as a shared library.
 */

#if defined(LVT_EXPORT_FUNCTIONS)
#if defined(_MSC_VER)
#define LVT_API __declspec(dllexport)
#elif defined(__GNUC__)
#define LVT_API __attribute__((visibility("default")))
#else
#define LVT_API
#pragma warning unknown compiler
#endif
#else
#if defined(_MSC_VER)
#define LVT_API __declspec(dllimport)
#else
#define LVT_API
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    typedef void *lvt_handle;

    LVT_API lvt_handle lvt_create(const char *config_file_name, int sensor_type); // create vo system using parameters in passed yaml config file, sensor_type: 1 = STEREO, 2 = RGBD
    LVT_API void lvt_destroy(lvt_handle vo_system); // destroy the passed vo system handle
    LVT_API void lvt_track(lvt_handle vo_system, unsigned char *left_img, unsigned char *right_img, int n_rows, int n_cols, double R[3][3], double t[3]); // perform tracking and return estimated pose in R, t. 
    LVT_API void lvt_track_with_external_corners(lvt_handle vo_system, unsigned char *left_img, unsigned char *right_img, int n_rows, int n_cols,
                                                    double corners_left[][2], int n_corners_left, double corners_right[][2], int n_corners_right, double R[3][3], double t[3]);
    LVT_API int lvt_get_status(lvt_handle vo_system); // 1 == vo not initialized yet. 2 == tracking. 3 == tracking is lost.

#ifdef __cplusplus
}
#endif

#endif //LVT_C_INTERFACE_H__
