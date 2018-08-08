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

#ifndef LVT_POSE_TRANSFORM_H__
#define LVT_POSE_TRANSFORM_H__

#include "lvt_definitions.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>

//-------------------------------------------------------
typedef Eigen::Matrix<double, 2, 1> lvt_vector2;
typedef Eigen::Matrix<double, 3, 1> lvt_vector3;
typedef Eigen::Matrix<double, 4, 1> lvt_vector4;
typedef Eigen::Matrix<double, 3, 3> lvt_matrix33;
typedef Eigen::Matrix<double, 3, 4> lvt_matrix34;
typedef Eigen::Matrix<double, 4, 4> lvt_matrix44;
typedef Eigen::Quaternion<double> lvt_quaternion;

typedef std::vector<lvt_vector2, Eigen::aligned_allocator<lvt_vector2>> lvt_vector2_array;
typedef std::vector<lvt_vector3, Eigen::aligned_allocator<lvt_vector3>> lvt_vector3_array;
typedef std::vector<lvt_vector4, Eigen::aligned_allocator<lvt_vector4>> lvt_vector4_array;
typedef std::vector<lvt_matrix33, Eigen::aligned_allocator<lvt_matrix33>> lvt_matrix33_array;
typedef std::vector<lvt_matrix34, Eigen::aligned_allocator<lvt_matrix34>> lvt_matrix34_array;
typedef std::vector<lvt_matrix44, Eigen::aligned_allocator<lvt_matrix44>> lvt_matrix44_array;
typedef std::vector<lvt_quaternion, Eigen::aligned_allocator<lvt_quaternion>> lvt_quaternion_array;

//-------------------------------------------------------
class lvt_pose
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    lvt_pose()
    {
        m_position.setZero();
        m_orientation.setIdentity();
    }
    lvt_pose(const lvt_vector3 &position, const lvt_quaternion &orientation)
    {
        set_parameters(position, orientation);
    }

    void set_parameters(const lvt_vector3 &position, const lvt_quaternion &orientation)
    {
        m_orientation = orientation;
        m_position = position;
    }

    lvt_vector3 get_position() const { return m_position; }
    lvt_matrix33 get_orientation_matrix() const { return m_orientation.toRotationMatrix(); }
    lvt_quaternion get_orientation_quaternion() const { return m_orientation; }

  private:
    lvt_quaternion m_orientation;
    lvt_vector3 m_position;
};

typedef std::vector<lvt_pose, Eigen::aligned_allocator<lvt_pose>> lvt_pose_array;

//-------------------------------------------------------
class lvt_pose_utils
{
  public:
    static lvt_pose compute_right_camera_pose(const lvt_pose &left_cam_pose, double baseline);
    static lvt_matrix34 compute_world_to_camera_transform(const lvt_pose &camera_pose);
    static lvt_matrix34 compute_projection_matrix(const lvt_pose &camera_pose, const lvt_matrix33 &intrinsics);
    inline static lvt_vector2 project_point(const lvt_vector3 &pt, const lvt_matrix34 &proj_mtrx)
    {
        lvt_vector3 proj_pt = proj_mtrx * lvt_vector4(pt.x(), pt.y(), pt.z(), 1.0);
        const double inv_z = 1.0 / proj_pt.z();
        return lvt_vector2(proj_pt.x() * inv_z, proj_pt.y() * inv_z);
    }
};

#endif //LVT_POSE_TRANSFORM_H__
