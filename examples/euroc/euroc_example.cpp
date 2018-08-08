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
        std::cout << "Usage ./euroc_example euroc_root_dir stamps_dir dataset_name config_file_name" << std::endl;
        return -1;
    }

    std::string root_dir = std::string(argv[1]);
    std::string stamps_dir = std::string(argv[2]);
    std::string dataset_name = std::string(argv[3]);
    std::string seq_dir = root_dir + std::string("/") + dataset_name + std::string("/mav0");
    std::string config_file_name = std::string(argv[4]);

    std::vector<std::string> imgs_titles;
    std::vector<double> time_stamps;
    {
        std::string stamps_file_str = stamps_dir + std::string("/") + dataset_name + std::string(".txt");
        std::ifstream stamps_file(stamps_file_str);
        if (!stamps_file.is_open())
        {
            std::cout << "Unable to open stamps files " << stamps_file_str << std::endl;
            return -1;
        }
        while (stamps_file)
        {
            std::string str;
            if (!std::getline(stamps_file, str))
                break;

            std::stringstream ss;
            ss << str;
            imgs_titles.push_back(ss.str() + std::string(".png"));
            double t;
            ss >> t;
            time_stamps.push_back((t / 1e9));
        }
    }

    lvt_parameters params;
    if (!params.init_from_file(config_file_name.c_str()))
    {
        std::cout << "Failed to initialize from " << config_file_name << std::endl;
        return -1;
    }

    cv::Mat kl = (cv::Mat_<double>(3, 3) << 458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0);
    cv::Mat kr = (cv::Mat_<double>(3, 3) << 457.587, 0.0, 379.999, 0.0, 456.134, 255.238, 0.0, 0.0, 1.0);
    cv::Mat pl = (cv::Mat_<double>(3, 4) << 435.2046959714599, 0, 367.4517211914062, 0, 0, 435.2046959714599, 252.2008514404297, 0, 0, 0, 1, 0);
    cv::Mat pr = (cv::Mat_<double>(3, 4) << 435.2046959714599, 0, 367.4517211914062, -47.90639384423901, 0, 435.2046959714599, 252.2008514404297, 0, 0, 0, 1, 0);
    cv::Mat rl = (cv::Mat_<double>(3, 3) << 0.999966347530033, -0.001422739138722922, 0.008079580483432283, 0.001365741834644127, 0.9999741760894847, 0.007055629199258132, -0.008089410156878961, -0.007044357138835809, 0.9999424675829176);
    cv::Mat rr = (cv::Mat_<double>(3, 3) << 0.9999633526194376, -0.003625811871560086, 0.007755443660172947, 0.003680398547259526, 0.9999684752771629, -0.007035845251224894, -0.007729688520722713, 0.007064130529506649, 0.999945173484644);
    cv::Mat dl = (cv::Mat_<double>(1, 5) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0);
    cv::Mat dr = (cv::Mat_<double>(1, 5) << -0.28368365, 0.07451284, -0.00010473, -3.555907e-05, 0.0);
    cv::Size img_size = cv::Size(752, 480);

    cv::Mat M1l, M2l, M1r, M2r;
    cv::initUndistortRectifyMap(kl, dl, rl, pl.rowRange(0, 3).colRange(0, 3), img_size, CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(kr, dr, rr, pr.rowRange(0, 3).colRange(0, 3), img_size, CV_32F, M1r, M2r);

    params.fx = 435.2046959714599;
    params.fy = 435.2046959714599;
    params.cx = 367.4517211914062;
    params.cy = 252.2008514404297;
    params.baseline = 0.110077842;

    lvt_matrix44 Tbs;
    Tbs << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
        0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
        0.0, 0.0, 0.0, 1.0;

    lvt_system *vo = nullptr;

    const int frameCount = imgs_titles.size();
    lvt_pose_array seq_poses;
    seq_poses.resize(frameCount);

    for (int i = 0; i < frameCount; i++)
    {
        std::cout << "Frame number: " << i << "/" << frameCount << "\r" << std::flush;

        cv::Mat imgLeft, imgRight, imgLeftRect, imgRightRect;
        std::string img_left_str = seq_dir + std::string("/cam0/data/") + imgs_titles[i];
        std::string img_right_str = seq_dir + std::string("/cam1/data/") + imgs_titles[i];
        imgLeft = cv::imread(img_left_str, CV_LOAD_IMAGE_UNCHANGED);
        imgRight = cv::imread(img_right_str, CV_LOAD_IMAGE_UNCHANGED);
        if (imgLeft.empty() || imgRight.empty())
        {
            std::cout << "Failed to load image " << imgs_titles[i] << std::endl;
            break;
        }

        cv::remap(imgLeft, imgLeftRect, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imgRight, imgRightRect, M1r, M2r, cv::INTER_LINEAR);

        if (!vo)
        {
            params.img_width = imgLeftRect.cols;
            params.img_height = imgLeftRect.rows;
            vo = lvt_system::create(params, lvt_system::eSensor_STEREO);
        }

        lvt_pose cam_pose = vo->track(imgLeftRect, imgRightRect);
        lvt_matrix44 cam_mtrx;
        cam_mtrx << cam_pose.get_orientation_matrix(), cam_pose.get_position(), 0, 0, 0, 1;
        lvt_matrix44 body_mtrx = Tbs * cam_mtrx;
        lvt_pose body_pose(lvt_vector3(body_mtrx(0, 3), body_mtrx(1, 3), body_mtrx(2, 3)),
                           lvt_quaternion(body_mtrx.block<3, 3>(0, 0)));
        seq_poses[i] = body_pose;

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
