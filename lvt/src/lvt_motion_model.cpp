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

#include "lvt_motion_model.h"
#include <Eigen/Geometry>

lvt_motion_model::lvt_motion_model()
{
    reset();
}

void lvt_motion_model::reset()
{
    m_last_q.setIdentity();
    m_angular_velocity.setIdentity();
    m_last_position.setZero();
    m_linear_velocity.setZero();
}

lvt_pose lvt_motion_model::predict_next_pose(const lvt_pose &current_pose)
{
    // compute new linear velocity
    lvt_vector3 new_lin_velocity = current_pose.get_position() - m_last_position;
    new_lin_velocity = (new_lin_velocity + m_linear_velocity) * 0.5; // smooth velocity over time

    // compute new angular velocity
    lvt_quaternion current_q = current_pose.get_orientation_quaternion();
    lvt_quaternion ang_vel_diff = current_q * m_last_q.inverse();
    lvt_quaternion new_ang_vel = ang_vel_diff.slerp(0.5, m_angular_velocity);
    new_ang_vel.normalize();

    // Update state
    m_last_q = current_q;
    m_angular_velocity = new_ang_vel;
    m_last_position = current_pose.get_position();
    m_linear_velocity = new_lin_velocity;

    // Integrate to compute predictions
    lvt_vector3 int_pos = m_last_position + m_linear_velocity;
    lvt_quaternion int_q = current_q * new_ang_vel;
    int_q.normalize();
    return lvt_pose(int_pos, int_q);
}
