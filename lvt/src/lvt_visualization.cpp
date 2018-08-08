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

#include "lvt_visualization.h"
#include "lvt_local_map.h"
#include "lvt_image_features_struct.h"

#ifdef LVT_ENABLE_VISUALIZATION
#include <pangolin/pangolin.h>
#endif //LVT_ENABLE_VISUALIZATION

static float s_viewPointX = 0;
static float s_viewPointY = -50;
static float s_viewPointZ = -50;
static float s_viewPointF = 2000;

static void get_camera_gl_matrix(const lvt_pose &cam_pose, pangolin::OpenGlMatrix &cam_gl_mtrx);

lvt_visualization::lvt_visualization(float camera_size, int point_size)
    : m_local_map(nullptr), m_is_adding_features(false)
{
    m_camera_size = camera_size;
    m_current_camera_size = m_camera_size * 1.2;
    m_point_size = point_size;
}

lvt_visualization::~lvt_visualization()
{
}

void lvt_visualization::reset()
{
    m_is_adding_features = false;
    m_map_indices.clear();
    m_img_indices.clear();
    m_cam_poses.clear();
#ifdef LVT_ENABLE_VISUALIZATION
    m_cam_gl_mtrx->SetIdentity();
#endif //LVT_ENABLE_VISUALIZATION
}

void lvt_visualization::begin_adding_matches()
{
    assert(!m_is_adding_features && "It seems matches addition is already in process!");
    m_map_indices.clear();
    m_img_indices.clear();
    m_is_adding_features = true;
}

void lvt_visualization::add_match(int map_point_idx, int image_struct_idx)
{
    m_map_indices.push_back(map_point_idx);
    m_img_indices.push_back(image_struct_idx);
}

void lvt_visualization::finish_adding_matches()
{
    assert(m_is_adding_features && "Matches addition has already finished.");
    m_is_adding_features = false;
}

std::vector<cv::Scalar> get_color_gradient(int depth)
{
    float dr = -1.0 / (depth - 1);
    float dg = 0.0;
    float db = 1.0 / (depth - 1);
    std::vector<cv::Scalar> grad(depth);
    for (int i = 0; i < depth; i++)
    {
        float b = db * i;
        float r = 1.0 + dr * i;
        grad[i] = cv::Scalar(b, 0.0, r);
    }
    return grad;
}

cv::Mat lvt_visualization::draw_features(const cv::Mat &left_img, lvt_image_features_struct *left_struct)
{
    cv::Mat dbg_img;
    cv::cvtColor(left_img, dbg_img, cv::COLOR_GRAY2BGR);
    int color_depth = 2;
    for (size_t i = 0, count = m_map_indices.size(); i < count; i++)
    {
        int age = m_local_map->m_map_points[m_map_indices[i]].m_age;
        if (age > color_depth)
            color_depth = age;
    }
    std::vector<cv::Scalar> color_grad = get_color_gradient(color_depth);
    for (size_t i = 0, count = m_map_indices.size(); i < count; i++)
    {
        int age = m_local_map->m_map_points[m_map_indices[i]].m_age;
        cv::Point2f loc = left_struct->get_keypoint(m_img_indices[i]).pt;
        cv::circle(dbg_img, loc, 2, color_grad[age - 1] * 255.0, 2);
    }

    for (int i = 0, count = left_struct->get_features_count(); i < count; i++)
    {
        if (!left_struct->is_matched(i))
        {
            cv::Point2f pt = left_struct->get_keypoint(i).pt;
            const float hr = 2.0f;
            cv::rectangle(dbg_img, cv::Point(pt.x - hr, pt.y - hr), cv::Point(pt.x + hr, pt.y + hr), cv::Scalar(255, 255, 255), 1);
        }
    }

    return dbg_img;
}

void lvt_visualization::display_features(const cv::Mat &left_img, lvt_image_features_struct *left_struct)
{
    cv::Mat dbg_img = draw_features(left_img, left_struct);
    cv::imshow("Frame tracking", dbg_img);
}

void lvt_visualization::init_map_viewer()
{
#ifdef LVT_ENABLE_VISUALIZATION
    pangolin::CreateWindowAndBind("SARLAB Visual Odometry", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    m_state_cam = new pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1024, 768, s_viewPointF, s_viewPointF, 512, 389, 0.01, 2000),
        pangolin::ModelViewLookAt(s_viewPointX, s_viewPointY, s_viewPointZ, 0, 0, 0, 0.0, -1.0, 0.0));

    m_display_cam = &pangolin::CreateDisplay()
                         .SetBounds(0.0, 1.0, 0, 1.0, -1024.0f / 768.0f)
                         .SetHandler(new pangolin::Handler3D(*m_state_cam));
    m_cam_gl_mtrx = new pangolin::OpenGlMatrix;
    m_cam_gl_mtrx->SetIdentity();
#endif //LVT_ENABLE_VISUALIZATION
}

void lvt_visualization::destroy_map_viewer()
{
#ifdef LVT_ENABLE_VISUALIZATION
    pangolin::DestroyWindow("SARLAB Visual Odometry");
    delete m_state_cam;
    delete m_cam_gl_mtrx;
#endif //LVT_ENABLE_VISUALIZATION
}

void lvt_visualization::push_camera_pose(const lvt_pose &cam_pose)
{
#ifdef LVT_ENABLE_VISUALIZATION
    m_cam_poses.push_back(cam_pose);
    draw_map();
#endif //LVT_ENABLE_VISUALIZATION
}

void get_camera_gl_matrix(const lvt_pose &cam_pose, pangolin::OpenGlMatrix &cam_gl_mtrx)
{
#ifdef LVT_ENABLE_VISUALIZATION
    const lvt_vector3 t = cam_pose.get_position();
    const lvt_matrix33 rot = cam_pose.get_orientation_matrix();
    cam_gl_mtrx.m[0] = rot(0, 0);
    cam_gl_mtrx.m[1] = rot(1, 0);
    cam_gl_mtrx.m[2] = rot(2, 0);
    cam_gl_mtrx.m[3] = 0.0;
    cam_gl_mtrx.m[4] = rot(0, 1);
    cam_gl_mtrx.m[5] = rot(1, 1);
    cam_gl_mtrx.m[6] = rot(2, 1);
    cam_gl_mtrx.m[7] = 0.0;
    cam_gl_mtrx.m[8] = rot(0, 2);
    cam_gl_mtrx.m[9] = rot(1, 2);
    cam_gl_mtrx.m[10] = rot(2, 2);
    cam_gl_mtrx.m[11] = 0.0;
    cam_gl_mtrx.m[12] = t.x();
    cam_gl_mtrx.m[13] = t.y();
    cam_gl_mtrx.m[14] = t.z();
    cam_gl_mtrx.m[15] = 1.0;
#endif //LVT_ENABLE_VISUALIZATION
}

void lvt_visualization::draw_current_frame(const pangolin::OpenGlMatrix &cam_gl_mtrx)
{
#ifdef LVT_ENABLE_VISUALIZATION
    const float w = m_current_camera_size;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();
    glMultMatrixd(cam_gl_mtrx.m);
    glLineWidth(3);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);
    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);
    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();
    glPopMatrix();
#endif //LVT_ENABLE_VISUALIZATION
}

void lvt_visualization::draw_map()
{
#ifdef LVT_ENABLE_VISUALIZATION
    const float w = m_camera_size;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const lvt_pose &cam_pose = m_cam_poses.back();
    get_camera_gl_matrix(cam_pose, *m_cam_gl_mtrx);
    m_state_cam->Follow(*m_cam_gl_mtrx);
    m_display_cam->Activate(*m_state_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    const int rows = 100;
    const int columns = 100;
    const int scale = 10;
    const lvt_vector3 pose_pos = cam_pose.get_position();

    glColor3f(119.0 / 255.0, 136.0 / 255.0, 153.0 / 255.0);
    glBegin(GL_LINES);
    // Horizontal lines
    for (int k = -rows; k < rows; k++)
    {
        float x1 = pose_pos.x() - scale * columns * 0.5;
        float x2 = pose_pos.x() + scale * columns * 0.5;
        float z = pose_pos.z() + scale * k;
        glVertex3f(x1, 0, z);
        glVertex3f(x2, 0, z);
    }
    // Vertical lines
    for (int k = -columns; k < columns; k++)
    {
        float x = pose_pos.x() + scale * k;
        float z1 = pose_pos.z() - scale * rows * 0.5;
        float z2 = pose_pos.z() + scale * rows * 0.5;
        glVertex3f(x, 0, z1);
        glVertex3f(x, 0, z2);
    }
    glEnd();

    draw_current_frame(*m_cam_gl_mtrx);

    for (int i = 0; i < m_cam_poses.size(); i++)
    {
        lvt_matrix44 mtrx;
        mtrx << m_cam_poses[i].get_orientation_matrix(), m_cam_poses[i].get_position(), 0.0, 0.0, 0.0, 1.0;

        glPushMatrix();
        glMultMatrixf(mtrx.cast<GLfloat>().eval().data());
        glLineWidth(1);
        glColor3f(0.0f, 0.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);
        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();
        glPopMatrix();
    }

    glPointSize(m_point_size);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 1.0);
    for (int i = 0; i < m_local_map->m_map_points.size(); i++)
    {
        const lvt_vector3 pos = m_local_map->m_map_points[i].m_position;
        glVertex3f(pos.x(), pos.y(), pos.z());
    }
    glEnd();

    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 0.0);
    for (int i = 0; i < m_local_map->m_staged_points.size(); i++)
    {
        const lvt_vector3 pos = m_local_map->m_staged_points[i].m_position;
        glVertex3f(pos.x(), pos.y(), pos.z());
    }
    glEnd();
    pangolin::FinishFrame();
#endif //LVT_ENABLE_VISUALIZATION
}

bool lvt_visualization::process_events()
{
#ifdef LVT_ENABLE_VISUALIZATION
    char key = (char)cv::waitKey(50);
    if (key == 'q' || key == 'Q' || pangolin::ShouldQuit())
    {
        return true;
    }
    else if (key == 'r' || key == 'R')
    {
        m_state_cam->SetModelViewMatrix(pangolin::ModelViewLookAt(s_viewPointX, s_viewPointY, s_viewPointZ, 0, 0, 0, 0.0, -1.0, 0.0));
        m_state_cam->Follow(*m_cam_gl_mtrx);
    }
    else if (key == 'p' || key == 'P')
    {
        bool resume = false;
        while (!resume)
        {
            draw_map();
            char key = (char)cv::waitKey(50);
            resume = (key == 'p' || key == 'P');
        }
    }
#endif //LVT_ENABLE_VISUALIZATION
    return false;
}
