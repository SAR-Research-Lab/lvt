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
#include "lvt_system.h"
#include <opencv2/opencv.hpp>

static void dump_kitti_trajectory(const std::string &fileName, const lvt_pose_array &seq_poses)
{
    std::ofstream file(fileName.c_str());
    assert(file.is_open());
    file << std::fixed;
    for (size_t i = 0, count = seq_poses.size(); i < count; i++)
    {
        lvt_matrix33 r = seq_poses[i].get_orientation_matrix();
        lvt_vector3 pos = seq_poses[i].get_position();
        file << std::setprecision(9) << r(0, 0) << " " << r(0, 1) << " " << r(0, 2) << " " << pos.x() << " "
             << r(1, 0) << " " << r(1, 1) << " " << r(1, 2) << " " << pos.y() << " "
             << r(2, 0) << " " << r(2, 1) << " " << r(2, 2) << " " << pos.z() << std::endl;
    }
    file.close();
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Usage ./kitti_example sequences_dir seq_number" << std::endl;
        return -1;
    }

    int seq_num = atoi(argv[2]);
    char seq_cstr[5];
    std::sprintf(seq_cstr, "%02d", seq_num);
    std::string seq_str = std::string(seq_cstr);
    std::string dir_prefix = std::string(argv[1]) + std::string("/") + seq_str;
    std::string dir_postfix_left = "/image_0/%06d.png";
    std::string dir_postfix_right = "/image_1/%06d.png";
    std::string capL_str = dir_prefix + dir_postfix_left;
    std::string calR_str = dir_prefix + dir_postfix_right;
    cv::VideoCapture capL(capL_str);
    cv::VideoCapture capR(calR_str);
    if (!capL.isOpened() || !capR.isOpened())
    {
        std::cout << "failed to get image sequences" << std::endl;
        return -1;
    }

    std::string cam_fname = std::string("calib/") + seq_str + ".yml";
    cv::FileStorage cam_fs(cam_fname, cv::FileStorage::READ);
    if (!cam_fs.isOpened())
    {
        std::cout << "failed to open camera matrix yml file" << std::endl;
        return -1;
    }

    lvt_parameters params;
    if (!params.init_from_file("vo_config.yaml"))
    {
        std::cout << "failed to initialize from vo_config.yml file." << std::endl;
        return -1;
    }

    cv::Mat cam_mtrx;
    cam_fs["camera_matrix"] >> cam_mtrx;
    double baseline = cam_fs["baseline"];
    cam_fs.release();

    int frameCount = (int)capL.get(CV_CAP_PROP_FRAME_COUNT);
    int w = (int)capL.get(CV_CAP_PROP_FRAME_WIDTH);
    int h = (int)capL.get(CV_CAP_PROP_FRAME_HEIGHT);

    params.fx = cam_mtrx.at<double>(0, 0);
    params.fy = cam_mtrx.at<double>(1, 1);
    params.cx = cam_mtrx.at<double>(0, 2);
    params.cy = cam_mtrx.at<double>(1, 2);
    params.baseline = baseline;
    params.img_width = w;
    params.img_height = h;

    lvt_system *vo = lvt_system::create(params, lvt_system::eSensor_STEREO);

    lvt_pose_array seq_poses;
    seq_poses.resize(frameCount);
    std::vector<double> frame_exec_times(frameCount, 0.0);
    std::string kitti_out_file_name = seq_str + std::string(".txt");
    const double tick_f = cv::getTickFrequency();
    for (int i = 0; i < frameCount; i++)
    {
        std::cout << "Frame number: " << i << "/" << frameCount << "\r" << std::flush;

        cv::Mat imgLeft, imgRight;
        capL >> imgLeft;
        capR >> imgRight;
        if (imgLeft.channels() != 1)
        {
            cv::cvtColor(imgLeft, imgLeft, cv::COLOR_BGR2GRAY);
        }
        if (imgRight.channels() != 1)
        {
            cv::cvtColor(imgRight, imgRight, cv::COLOR_BGR2GRAY);
        }

        double t = (double)cv::getTickCount();
        seq_poses[i] = vo->track(imgLeft, imgRight);
        frame_exec_times[i] = (((double)cv::getTickCount() - t) / tick_f);

        if (vo->get_state() == lvt_system::eState_LOST ||
            vo->should_quit())
        {
            break;
        }
    }

    dump_kitti_trajectory(kitti_out_file_name, seq_poses);
    lvt_system::destroy(vo);

    double total_time = 0.0;
    for (size_t i = 0; i < frameCount; i++)
    {
        total_time += frame_exec_times[i];
    }

    std::cout << "Aerage frame processing time: " << total_time / double(frameCount) << std::endl;

    return 0;
}
