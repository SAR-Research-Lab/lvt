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

#include "lvt_system.h"
#include "lvt_local_map.h"
#include "lvt_image_features_handler.h"
#include "lvt_motion_model.h"
#include "lvt_pnp_solver.h"
#include "lvt_visualization.h"
#include "lvt_logging_utils.h"

lvt_system::lvt_system() : m_local_map(nullptr), m_features_handler(nullptr), m_motion_model(nullptr), m_pnp_solver(nullptr), _the_log(nullptr), _the_value_recorder(nullptr),
                           m_frame_number(0), m_state(eState_NOT_INITIALIZED), m_should_quit(false)
{
    m_last_matches = std::deque<int>(N_MATCHES_WINDOWS, std::numeric_limits<int>::max());
}

lvt_system::~lvt_system()
{
}

void lvt_system::reset()
{
    m_local_map->reset();
    m_motion_model->reset();

#ifdef LVT_ENABLE_VISUALIZATION
    if (m_params.enable_visualization)
    {
        m_visualization->reset();
    }
#endif //LVT_ENABLE_VISUALIZATION

#ifdef LVT_ENABLE_MEASURMENT_RECORD
    _the_value_recorder->finish();
    _the_value_recorder->init();
    register_measurments_values();
#endif //LVT_ENABLE_MEASURMENT_RECORD

    m_last_pose = lvt_pose();
    m_frame_number = 0;
    m_last_matches = std::deque<int>(N_MATCHES_WINDOWS, std::numeric_limits<int>::max());
    m_state = eState_NOT_INITIALIZED;
    m_should_quit = false;
    LVT_LOG("VO was just reset.");
}

lvt_system *lvt_system::create(const lvt_parameters &params, eSensor sensor_type)
{
    lvt_system *instance = new lvt_system();
    instance->m_params = params;
    instance->m_sensor = sensor_type;
    instance->m_features_handler = new lvt_image_features_handler(instance->m_params);
    instance->m_motion_model = new lvt_motion_model();

    instance->triangulation_policy = &lvt_system::triangulation_policy_decreasing_matches;
    if (params.triangulation_policy == lvt_parameters::etriangulation_policy_always_triangulate)
    {
        instance->triangulation_policy = &lvt_system::triangulation_policy_always_triangulate;
    }

#ifdef LVT_ENABLE_VISUALIZATION
    if (params.enable_visualization)
    {
        instance->m_visualization = new lvt_visualization(params.viewer_camera_size, params.viewer_point_size);
    }
#endif //LVT_ENABLE_VISUALIZATION

    instance->m_local_map = new lvt_local_map(params, instance->m_features_handler, instance->m_visualization);
    instance->m_pnp_solver = new lvt_pnp_solver(params.fx, params.fy, params.cx, params.cy, params.baseline);

#ifdef LVT_ENABLE_VISUALIZATION
    if (params.enable_visualization)
    {
        instance->m_visualization->set_map(instance->m_local_map);
        instance->m_visualization->init_map_viewer();
    }
#endif //LVT_ENABLE_VISUALIZATION

#ifdef LVT_ENABLE_LOG
    instance->_the_log = new lvt_log();
    instance->m_features_handler->_the_log = instance->_the_log;
    instance->m_local_map->_the_log = instance->_the_log;
    instance->m_pnp_solver->_the_log = instance->_the_log;
    if (params.enable_logging)
    {
        instance->_the_log->init();
        instance->_the_log->log_params(params);
    }
#endif //LVT_ENABLE_LOG

#ifdef LVT_ENABLE_MEASURMENT_RECORD
    instance->_the_value_recorder = new lvt_value_recorder();
    instance->m_local_map->_the_value_recorder = instance->_the_value_recorder;
    instance->m_pnp_solver->_the_value_recorder = instance->_the_value_recorder;
    instance->_the_value_recorder->init();
    instance->register_measurments_values();
#endif //LVT_ENABLE_MEASURMENT_RECORD

    return instance;
}

void lvt_system::destroy(lvt_system *p_instance)
{
    assert(p_instance && "lvt_system instance passed is null!");
    delete p_instance->m_motion_model;
    delete p_instance->m_features_handler;
    delete p_instance->m_local_map;
    delete p_instance->m_pnp_solver;

#ifdef LVT_ENABLE_VISUALIZATION
    if (p_instance->m_params.enable_visualization)
    {
        p_instance->m_visualization->destroy_map_viewer();
        delete p_instance->m_visualization;
    }
#endif //LVT_ENABLE_VISUALIZATION

#ifdef LVT_ENABLE_MEASURMENT_RECORD
    p_instance->_the_value_recorder->finish();
    delete p_instance->_the_value_recorder;
#endif //LVT_ENABLE_MEASURMENT_RECORD

#ifdef LVT_ENABLE_LOG
    delete p_instance->_the_log;
#endif //LVT_ENABLE_LOG

    delete p_instance;
}

lvt_pose lvt_system::track(cv::Mat img1, cv::Mat img2)
{
    LVT_LOG("-------------------- Frame #" + std::to_string(m_frame_number) + " --------------------");
    m_frame_number++;
    if (m_state == eState_LOST)
    {
        // TODO: handle lost state
        LVT_LOG("track is being called while tracking is lost, returning last pose");
        return m_last_pose;
    }

    lvt_image_features_struct left_struct, right_struct;
    if (m_sensor == eSensor_STEREO)
    {
        LVT_ASSERT(img1.channels() == 1 && img2.channels() == 1 && "passed images must be grayscale");
        LVT_LOG("Started computing images features...");
        m_features_handler->compute_features(img1, img2, &left_struct, &right_struct);
        LVT_LOG("Finished image features computation. Features count found: Left: " +
                std::to_string(left_struct.get_features_count()) + " Right: " + std::to_string(right_struct.get_features_count()));
    }
    else if (m_sensor == eSensor_RGBD)
    {
        LVT_ASSERT(img1.channels() == 1 && img2.type() == CV_32F && "In RGBD case, img1 must be the grayscale image and img2 the depth");
        LVT_LOG("Started computing images features (RGBD)...");
        m_features_handler->compute_features_rgbd(img1, img2, &left_struct);
        LVT_LOG("Finished image features computation. Features count found: " + std::to_string(left_struct.get_features_count()));
    }

    if (m_state == eState_NOT_INITIALIZED)
    {
        lvt_pose identity_pose; // The first frame sets the world coordinate frame that all subsequent poses will be reported with respect to. Hence, the first frame will report identity pose always.
        m_local_map->update_with_new_triangulation(identity_pose, &left_struct, &right_struct, true);
        m_state = eState_TRACKING;
        m_last_matches[0] = m_local_map->get_map_size();
        LVT_LOG("Tracking initialized. Map size: " + std::to_string(m_local_map->get_map_size()));
        return identity_pose;
    }

    bool is_tracking = false;
    lvt_pose predicted_pose = m_motion_model->predict_next_pose(m_last_pose);
    lvt_pose computed_pose = perform_tracking(predicted_pose, &left_struct, &right_struct, &is_tracking, &img1);
    LVT_FLUSH_RECORD();
    if (!is_tracking)
    {
        m_state = eState_LOST;
        LVT_LOG("Tracking was just lost. Returning last pose");
        return m_last_pose;
    }
    m_last_pose = computed_pose;
    return computed_pose;
}

lvt_pose lvt_system::track_with_external_corners(cv::Mat left_image, cv::Mat right_image,
                                                 std::vector<cv::Point2f> &corners_locations_left, std::vector<cv::Point2f> &corners_locations_right)
{
    assert(left_image.channels() == 1 && right_image.channels() == 1 && "passed images must be grayscale");
    LVT_LOG("-------------------- Frame #" + std::to_string(m_frame_number) + " --------------------");
    m_frame_number++;
    if (m_state == eState_LOST)
    {
        // TODO: handle lost state
        LVT_LOG("track_with_external_corners is being called while tracking is lost, returning last pose");
        return m_last_pose;
    }

    LVT_LOG("Started computing images descriptors only...");
    lvt_image_features_struct left_struct, right_struct;
    m_features_handler->compute_descriptors_only(left_image, corners_locations_left, &left_struct,
                                                 right_image, corners_locations_right, &right_struct);
    LVT_LOG("Finished computing images descriptors only.");

    if (m_state == eState_NOT_INITIALIZED)
    {
        lvt_pose identity_pose; // The first frame sets the world coordinate frame that all subsequent poses will be reported with respect to. Hence, the first frame will report identity pose always.
        m_local_map->update_with_new_triangulation(identity_pose, &left_struct, &right_struct, true);
        m_state = eState_TRACKING;
        m_last_matches[0] = m_local_map->get_map_size();
        LVT_LOG("Tracking initialized. Map size: " + std::to_string(m_local_map->get_map_size()));
        return identity_pose;
    }

    bool is_tracking = false;
    lvt_pose predicted_pose = m_motion_model->predict_next_pose(m_last_pose);
    lvt_pose computed_pose = perform_tracking(predicted_pose, &left_struct, &right_struct, &is_tracking);
    LVT_FLUSH_RECORD();
    if (!is_tracking)
    {
        m_state = eState_LOST;
        LVT_LOG("Tracking was just lost. Returning last pose");
        return m_last_pose;
    }
    m_last_pose = computed_pose;
    return computed_pose;
}

lvt_pose lvt_system::perform_tracking(const lvt_pose &estimated_pose, lvt_image_features_struct *left_struct,
                                      lvt_image_features_struct *right_struct, bool *is_tracking, const cv::Mat *left_img_dbg)
{
    LVT_RECORD("map points count", m_local_map->get_map_size());
    LVT_RECORD("staged points count", m_local_map->get_staged_points_count());
    LVT_RECORD("image keypoints", left_struct->get_features_count());
    LVT_LOG("Map size: " + std::to_string(m_local_map->get_map_size()));

    // projection matching, find 2D-3D associations.
    lvt_vector3_array map_points;
    std::vector<int> matches_left;
    LVT_LOG("Started looking for matching 2D-3D points...");
    m_local_map->find_matches(estimated_pose, left_struct, &map_points, &matches_left);
    LVT_LOG("Matching finished. #: " + std::to_string(matches_left.size()));
    const int matches_count = map_points.size();
    if (matches_count < m_params.min_num_matches_for_tracking)
    {
        LVT_LOG("Not enough matched features for tracking.");
        *is_tracking = false;
        return m_last_pose;
    }
    m_last_matches.push_back(matches_count);
    m_last_matches.pop_front();

    lvt_pose optimized_pose = m_pnp_solver->compute_pose(estimated_pose, left_struct, map_points, matches_left);

    if (m_params.staged_threshold > 0)
    {
        m_local_map->update_staged_map_points(optimized_pose, left_struct);
    }

#ifdef LVT_ENABLE_VISUALIZATION
    if (m_params.enable_visualization)
    {
        m_visualization->display_features(*left_img_dbg, left_struct);
        m_visualization->push_camera_pose(optimized_pose);
        m_should_quit = m_visualization->process_events();
    }
#endif //LVT_ENABLE_VISUALIZATION

    if (need_new_triangulation())
    {
        m_local_map->update_with_new_triangulation(optimized_pose, left_struct, right_struct);
        LVT_LOG("New triangulation performed.");
    }
    else
    {
        LVT_LOG("No new triangulation performed this frame.");
    }

    m_local_map->clean_untracked_points();

    *is_tracking = true;
    return optimized_pose;
}

bool lvt_system::need_new_triangulation()
{
    return ((this->*triangulation_policy)());
}

bool lvt_system::triangulation_policy_decreasing_matches()
{
    const float ratio = 0.99;
    for (int i = N_MATCHES_WINDOWS - 1; i > 0; --i)
    {
        if (float(m_last_matches[i]) > ratio * float(m_last_matches[i - 1]))
        {
            return false;
        }
    }
    return true;
}

bool lvt_system::triangulation_policy_always_triangulate()
{
    return true;
}

void lvt_system::register_measurments_values()
{
#ifdef LVT_ENABLE_MEASURMENT_RECORD
    _the_value_recorder->register_value("map points count");
    _the_value_recorder->register_value("staged points count");
    _the_value_recorder->register_value("image keypoints");
    _the_value_recorder->register_value("tracked map points");
    _the_value_recorder->register_value("age");
    _the_value_recorder->register_value("closest descriptor distance");
    _the_value_recorder->register_value("second descriptor distance");
    _the_value_recorder->register_value("img feature x");
    _the_value_recorder->register_value("img feature y");
    _the_value_recorder->register_value("inlier count");
#endif //LVT_ENABLE_MEASURMENT_RECORD
}
