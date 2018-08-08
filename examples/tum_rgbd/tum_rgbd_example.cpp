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

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sstream>
#include "lvt_system.h"
#include <opencv2/opencv.hpp>

static void dump_tum_trajectory(const std::string &fileName, const lvt_pose_array &seq_poses, const std::vector<double> &time_stamps)
{
    std::ofstream file(fileName.c_str());
    assert(file.is_open());
    file << std::fixed;
    for (size_t i = 0, count = seq_poses.size(); i < count; i++)
    {
        lvt_quaternion q = seq_poses[i].get_orientation_quaternion();
        lvt_vector3 pos = seq_poses[i].get_position();
        file << std::setprecision(6) << time_stamps[i] << std::setprecision(7) << " " << pos.x() << " " << pos.y() << " " << pos.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    file.close();
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::cout << "Usage ./tum_rgbd_example tum_sequences_root_dir associations_dir dataset_name config_file_name" << std::endl;
        return -1;
    }

    std::string root_dir = std::string(argv[1]);
    std::string associations_dir = std::string(argv[2]);
    std::string dataset_name = std::string(argv[3]);
    std::string config_file_name = std::string(argv[4]);

    std::vector<std::string> rgb_titles, depth_titles;
    std::vector<double> time_stamps;
    {
        std::string associations_file_str = associations_dir + std::string("/") + dataset_name + std::string(".txt");
        std::ifstream associations_file(associations_file_str);
        if (!associations_file.is_open())
        {
            std::cout << "Unable to open asscoiations files " << associations_file_str << std::endl;
            return -1;
        }
        while (associations_file)
        {
            std::string str;
            if (!std::getline(associations_file, str))
                break;

            std::stringstream ss;
            ss << str;
            double t;
            ss >> t;
            time_stamps.push_back(t);
            std::string rgb_str, depth_str;
            ss >> rgb_str;
            rgb_titles.push_back(rgb_str);
            ss >> t;
            ss >> depth_str;
            depth_titles.push_back(depth_str);
        }
    }

    if (rgb_titles.empty() || rgb_titles.size() != depth_titles.size())
    {
        std::cout << "Image asscoiations was not read correctly" << std::endl;
        return -1;
    }

    lvt_parameters params;
    if (!params.init_from_file(config_file_name.c_str()))
    {
        std::cout << "Failed to initialize from " << config_file_name << std::endl;
        return -1;
    }

    lvt_system *vo = lvt_system::create(params, lvt_system::eSensor_RGBD);

    const int frameCount = rgb_titles.size();
    lvt_pose_array seq_poses;
    seq_poses.resize(frameCount);

    const float depth_scale = 1.0f / 5000.0f; // depth values in TUM are scaled

    for (int i = 0; i < frameCount; i++)
    {
        std::cout << "Frame number: " << i << "/" << frameCount << "\r" << std::flush;

        cv::Mat img_rgb, img_depth;
        std::string img_rgb_str = root_dir + std::string("/") + dataset_name + std::string("/") + rgb_titles[i];
        std::string img_depth_str = root_dir + std::string("/") + dataset_name + std::string("/") + depth_titles[i];
        img_rgb = cv::imread(img_rgb_str, CV_LOAD_IMAGE_UNCHANGED);
        img_depth = cv::imread(img_depth_str, CV_LOAD_IMAGE_UNCHANGED);
        if (img_rgb.empty() || img_depth.empty())
        {
            std::cout << "Failed to load image " << std::endl;
            break;
        }

        cv::Mat img_gray;
        cv::cvtColor(img_rgb, img_gray, CV_BGR2GRAY);
        img_depth.convertTo(img_depth, CV_32F, depth_scale);

        seq_poses[i] = vo->track(img_gray, img_depth);

        if (vo->get_state() == lvt_system::eState_LOST ||
            vo->should_quit())
        {
            cv::waitKey();
            break;
        }
    }

    dump_tum_trajectory(dataset_name + std::string(".txt"), seq_poses, time_stamps);
    lvt_system::destroy(vo);
    return 0;
}
