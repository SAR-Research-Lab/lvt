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

#ifndef LVT_IMAGE_FEATURES_HANDLER_H__
#define LVT_IMAGE_FEATURES_HANDLER_H__

#include "lvt_definitions.h"
#include "lvt_image_features_struct.h"
#include "lvt_parameters.h"
#include "opencv2/opencv.hpp"
#include <vector>

/*
 * This class encapsulates image feature handling stuff.
 */

struct compute_features_data
{
    cv::Mat img;
    cv::Ptr<cv::AgastFeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;
    std::vector<cv::Rect> sub_imgs_rects;
    std::vector<cv::Point2f> *ext_kp;
    lvt_image_features_struct *features_struct;
    lvt_parameters *vo_params;
};

class lvt_image_features_handler
{
  public:
    lvt_image_features_handler(const lvt_parameters &vo_params);
    ~lvt_image_features_handler();

    void compute_features(const cv::Mat &img_left, const cv::Mat &img_right,
                          lvt_image_features_struct *out_left, lvt_image_features_struct *out_right);

    void compute_features_rgbd(const cv::Mat &img_gray, const cv::Mat &img_depth, lvt_image_features_struct *out_struct);

    void compute_descriptors_only(const cv::Mat &img_left, std::vector<cv::Point2f> &ext_kp_left, lvt_image_features_struct *out_left,
                                  const cv::Mat &img_right, std::vector<cv::Point2f> &ext_kp_right, lvt_image_features_struct *out_right); // Use supplied corners and only compute the descriptors.

    void row_match(lvt_image_features_struct *features_left, lvt_image_features_struct *features_right,
                   std::vector<cv::DMatch> *out_matches);

  private:
    LVT_ADD_LOGGING;
    void perform_compute_features(compute_features_data *);
    void perform_compute_descriptors_only(compute_features_data *);
    lvt_parameters m_vo_params;
    std::vector<cv::Rect> m_sub_imgs_rects;
    compute_features_data m_th_data[2];
};

#endif //LVT_IMAGE_FEATURES_HANDLER_H__
