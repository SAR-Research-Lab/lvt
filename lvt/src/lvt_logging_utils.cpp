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

#include "lvt_logging_utils.h"
#include <iomanip>
#include <ctime>
#include <sstream>
#include <algorithm>

lvt_log::lvt_log() : m_init(false)
{
}

lvt_log::~lvt_log()
{
    if (m_init)
    {
        m_file.close();
    }
}

void lvt_log::init()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream ss;
    ss << "vo-" << std::put_time(&tm, "%d-%b-%Y-%H-%M-%S") << ".txt";
    m_file.open(ss.str());
    assert(m_file.is_open());
    if (m_file.is_open())
    {
        m_init = true;
    }
    m_start_time = std::chrono::high_resolution_clock::now();
}

void lvt_log::log(const std::string &str)
{
    if (m_init)
    {
        std::chrono::duration<double, std::milli> d = std::chrono::high_resolution_clock::now() - m_start_time;
        m_file << d.count() << "  " << str << std::endl;
    }
}

void lvt_log::log_params(const lvt_parameters &params)
{
    if (!m_init)
    {
        return;
    }
    m_file << "VO system is configured with these parameters:" << std::endl;
    m_file << "fx = " << params.fx << "  fy = " << params.fy << std::endl;
    m_file << "cx = " << params.cx << "  cy = " << params.cy << std::endl;
    m_file << "Stereo baseline = " << params.baseline << std::endl;
    m_file << "Image width = " << params.img_width << std::endl;
    m_file << "Image height = " << params.img_height << std::endl;
    m_file << "Near plane distance = " << params.near_plane_distance << std::endl;
    m_file << "Far plane distance = " << params.far_plane_distance << std::endl;
    m_file << "Triangulation ratio test threshold = " << params.triangulation_ratio_test_threshold << std::endl;
    m_file << "Tracking ratio test threshold = " << params.tracking_ratio_test_threshold << std::endl;
    m_file << "Single descriptor matching distance = " << params.descriptor_matching_threshold << std::endl;
    m_file << "Minimum number of matches for tracking = " << params.min_num_matches_for_tracking << std::endl;
    m_file << "Tracking radius = " << params.tracking_radius << std::endl;
    m_file << "Detection cell size = " << params.detection_cell_size << std::endl;
    m_file << "Max keypoints per cell = " << params.max_keypoints_per_cell << std::endl;
    m_file << "Agast threshold = " << params.agast_threshold << std::endl;
    m_file << "Untracked threshold = " << params.untracked_threshold << std::endl;
    m_file << "Staged threshold = " << params.staged_threshold << std::endl;
    m_file << "====================================================================" << std::endl;
}

lvt_value_recorder::lvt_value_recorder()
{
}

lvt_value_recorder::~lvt_value_recorder()
{
}

void lvt_value_recorder::init()
{
    m_values_file.open("measurments.txt");
}

void lvt_value_recorder::finish()
{
    m_values_file.close();

    std::ofstream titles_file("titles.txt");
    for (int i = 0; i < m_titles.size(); i++)
    {
        titles_file << m_titles[i] << std::endl;
    }
    m_titles.clear();
    m_values.clear();
    titles_file.close();
}

void lvt_value_recorder::register_value(const std::string &val_name)
{
    assert(std::find(m_titles.begin(), m_titles.end(), val_name) == m_titles.end());
    m_titles.push_back(val_name);
    m_values[val_name];
}

void lvt_value_recorder::flush_frame()
{
    assert(m_values_file.is_open());
    if (!m_values_file.is_open())
    {
        return;
    }

    std::stringstream ss;
    for (int i = 0; i < m_titles.size(); i++)
    {
        std::vector<value_holder> &v = m_values[m_titles[i]];
        for (int k = 0; k < v.size(); k++)
        {
            ss << (v[k].value_type == value_holder::integer ? v[k].as_int : v[k].as_float) << ',';
        }
        v.clear();
    }
    std::string str = ss.str();
    str.pop_back();
    m_values_file << str << std::endl;
}
