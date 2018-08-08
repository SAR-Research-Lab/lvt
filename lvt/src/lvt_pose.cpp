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

#include "lvt_pose.h"

lvt_pose lvt_pose_utils::compute_right_camera_pose(const lvt_pose &left_cam_pose, double baseline)
{
    // The right camera will have the same orientation as the left and will be translated by stereo baseline
    lvt_vector3 pt(baseline, 0.0, 0.0);
    lvt_vector3 pos = (left_cam_pose.get_orientation_matrix() * pt) + left_cam_pose.get_position();
    return lvt_pose(pos, left_cam_pose.get_orientation_quaternion());
}

lvt_matrix34 lvt_pose_utils::compute_world_to_camera_transform(const lvt_pose &camera_pose)
{
    lvt_matrix33 world_to_cam_rot_mtrx = (camera_pose.get_orientation_matrix()).transpose();
    lvt_vector3 world_to_cam_translation = (-world_to_cam_rot_mtrx) * camera_pose.get_position();
    lvt_matrix34 world_to_cam_mtrx;
    world_to_cam_mtrx << world_to_cam_rot_mtrx, world_to_cam_translation;
    return world_to_cam_mtrx;
}

lvt_matrix34 lvt_pose_utils::compute_projection_matrix(const lvt_pose &camera_pose, const lvt_matrix33 &intrinsics)
{
    lvt_matrix34 world_to_cam_mtrx, proj_mtrx;
    world_to_cam_mtrx = compute_world_to_camera_transform(camera_pose);
    proj_mtrx = intrinsics * world_to_cam_mtrx;
    return proj_mtrx;
}
