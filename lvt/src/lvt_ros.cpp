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
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class lvt_ros
{
public:
    lvt_ros(const std::string &img_transport);
    ~lvt_ros();

private:
    lvt_system *m_vo_system;
    lvt_parameters m_vo_params;

    image_transport::SubscriberFilter m_img_left_sub, m_img_right_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> m_info_left_sub, m_info_right_sub;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ExactSync> m_exact_sync;
    boost::shared_ptr<ApproximateSync> m_approximate_sync;

    ros::ServiceServer m_reset_srv;
    ros::Publisher m_odom_pub;

    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;
    std::string m_camera_frame_id, m_odom_frame_id, m_baselink_frame_id;

    tf2::Transform m_base_to_odom, m_base_to_sensor;
    lvt_matrix33 m_rot_fix; // to align the poses so that z-axis is up and x-axis to the front.
    lvt_matrix33 m_last_rotation;
    lvt_vector3 m_last_position;
    ros::Time m_last_update_time;
    bool m_reset_pose_on_lost_vo; // in case when vo is lost and reset you want to continue accumelating poses from where you left off

    void create_vo_system(const sensor_msgs::CameraInfoConstPtr &info_msg_left, const sensor_msgs::CameraInfoConstPtr &info_msg_right);
    bool reset_vo(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
    void init_transforms();
    void on_stereo_image(
            const sensor_msgs::ImageConstPtr &img_msg_left, const sensor_msgs::CameraInfoConstPtr &info_msg_left,
            const sensor_msgs::ImageConstPtr &img_msg_right, const sensor_msgs::CameraInfoConstPtr &info_msg_right);
};

lvt_ros::lvt_ros(const std::string &img_transport)
        : m_vo_system(nullptr), m_tf_listener(m_tf_buffer)
{
    m_rot_fix.setIdentity();// = Eigen::AngleAxisd(-1.57079632679, Eigen::Vector3d::UnitZ()).toRotationMatrix() * Eigen::AngleAxisd(-1.57079632679, Eigen::Vector3d::UnitX()).toRotationMatrix();
    m_base_to_odom.setIdentity();
    m_base_to_sensor.setIdentity();
    m_last_rotation = m_rot_fix;
    m_last_position.setZero();

    ros::NodeHandle nh;
    std::string left_img_topic = "/left/image_rect_gray";
    std::string right_img_topic = "/right/image_rect_gray";
    std::string left_info_topic = "/left/camera_info";
    std::string right_info_topic = "/right/camera_info";

    ROS_INFO("Subscribed to:\n\t* %s\n\t* %s\n\t* %s\n\t* %s",
             left_img_topic.c_str(), right_img_topic.c_str(),
             left_info_topic.c_str(), right_info_topic.c_str());

    image_transport::ImageTransport it(nh);
    m_img_left_sub.subscribe(it, left_img_topic, 10, img_transport);
    m_img_right_sub.subscribe(it, right_img_topic, 10, img_transport);
    m_info_left_sub.subscribe(nh, left_info_topic, 10);
    m_info_right_sub.subscribe(nh, right_info_topic, 10);

    ros::NodeHandle local_nh("~");

    int queue_size;
    local_nh.param("queue_size", queue_size, 10);

    bool use_approx;
    local_nh.param("approximate_sync", use_approx, false);
    if (use_approx)
    {
        m_approximate_sync.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                     m_img_left_sub, m_info_left_sub,
                                                     m_img_right_sub, m_info_right_sub));
        m_approximate_sync->registerCallback(boost::bind(&lvt_ros::on_stereo_image, this, _1, _2, _3, _4));
    }
    else
    {
        m_exact_sync.reset(new ExactSync(ExactPolicy(queue_size),
                                         m_img_left_sub, m_info_left_sub,
                                         m_img_right_sub, m_info_right_sub));
        m_exact_sync->registerCallback(boost::bind(&lvt_ros::on_stereo_image, this, _1, _2, _3, _4));
    }

    m_reset_srv = local_nh.advertiseService("reset_vo", &lvt_ros::reset_vo, this);
    m_odom_pub = local_nh.advertise<nav_msgs::Odometry>("odometry", 1);

    local_nh.param("sensor_frame_id", m_camera_frame_id, std::string("camera"));
    local_nh.param("odom_frame_id", m_odom_frame_id, std::string("odom"));
    local_nh.param("base_link_frame_id", m_baselink_frame_id, std::string("base_link"));

    local_nh.param<float>("near_plane_distance", m_vo_params.near_plane_distance, 0.1);
    local_nh.param<float>("far_plane_distance", m_vo_params.far_plane_distance, 500.0);
    local_nh.param<float>("triangulation_ratio_test_threshold", m_vo_params.triangulation_ratio_test_threshold, 0.6);
    local_nh.param<float>("tracking_ratio_test_threshold", m_vo_params.tracking_ratio_test_threshold, 0.8);
    local_nh.param<float>("descriptor_matching_threshold", m_vo_params.descriptor_matching_threshold, 30.0);
    local_nh.param<int>("tracking_radius", m_vo_params.tracking_radius, 25);
    local_nh.param<int>("detection_cell_size", m_vo_params.detection_cell_size, 250);
    local_nh.param<int>("max_keypoints_per_cell", m_vo_params.max_keypoints_per_cell, 150);
    local_nh.param<int>("agast_threshold", m_vo_params.agast_threshold, 20);
    local_nh.param<int>("untracked_threshold", m_vo_params.untracked_threshold, 10);
    local_nh.param<int>("staged_threshold", m_vo_params.staged_threshold, 0);
    int dumb = 0;
    local_nh.param<int>("enable_logging", dumb, 1);
    m_vo_params.enable_logging = dumb;
    local_nh.param<int>("enable_visualization", dumb, 1);
    m_vo_params.enable_visualization = dumb;
    local_nh.param<int>("triangulation_policy", m_vo_params.triangulation_policy, 3);

    local_nh.param<int>("m_reset_pose_on_lost_vo", dumb, 1);
    m_reset_pose_on_lost_vo = dumb;
}

lvt_ros::~lvt_ros()
{
    if (m_vo_system)
    {
        lvt_system::destroy(m_vo_system);
    }
}

void lvt_ros::create_vo_system(const sensor_msgs::CameraInfoConstPtr &info_msg_left, const sensor_msgs::CameraInfoConstPtr &info_msg_right)
{
    m_vo_params.baseline = fabs(info_msg_right->P[3] / info_msg_right->P[0]);
    m_vo_params.fx = info_msg_right->P[0];
    m_vo_params.fy = info_msg_right->P[0];
    m_vo_params.cx = info_msg_right->P[2];
    m_vo_params.cy = info_msg_right->P[6];
    m_vo_params.img_width = info_msg_left->width;
    m_vo_params.img_height = info_msg_left->height;
    m_vo_system = lvt_system::create(m_vo_params, lvt_system::eSensor_STEREO);
}

bool lvt_ros::reset_vo(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
{
    ROS_WARN_COND(!m_vo_system, "Trying to reset uninitialized vo system.");
    if (m_vo_system)
    {
        m_vo_system->reset();
        m_last_rotation = m_rot_fix;
        m_last_position.setZero();
        m_base_to_odom.setIdentity();
        m_tf_buffer.clear();
        ROS_INFO("VO was reset.");
    }
    init_transforms();
    return true;
}

void lvt_ros::init_transforms()
{
     std::string error_msg;
     if (m_tf_buffer.canTransform(m_baselink_frame_id, m_camera_frame_id, ros::Time(), &error_msg))
     {
     	tf2::fromMsg(m_tf_buffer.lookupTransform(m_baselink_frame_id, m_camera_frame_id, ros::Time(), ros::Duration(15)).transform, m_base_to_sensor);
     }
     else
     {
     	ROS_WARN_THROTTLE(10.0, "Cannot transform from '%s' to '%s'. Will assume identity.", m_baselink_frame_id.c_str(), m_camera_frame_id.c_str());
     	ROS_DEBUG("Transform error: %s", error_msg.c_str());
     	m_base_to_sensor.setIdentity();
     }

    m_base_to_sensor.setIdentity();
}

void lvt_ros::on_stereo_image(
        const sensor_msgs::ImageConstPtr &img_msg_left, const sensor_msgs::CameraInfoConstPtr &info_msg_left,
        const sensor_msgs::ImageConstPtr &img_msg_right, const sensor_msgs::CameraInfoConstPtr &info_msg_right)
{
    if (!m_vo_system)
    {
        create_vo_system(info_msg_left, info_msg_right);
        init_transforms();
    }

    const ros::Time &timestamp = img_msg_left->header.stamp;
    if (m_last_update_time > timestamp)
    {
        ROS_WARN("Images with older time stamps recieved. Will be ignored.");
        return;
    }

    cv_bridge::CvImageConstPtr img_ptr_l = cv_bridge::toCvShare(img_msg_left, "mono8");
    cv_bridge::CvImageConstPtr img_ptr_r = cv_bridge::toCvShare(img_msg_right, "mono8");

    ROS_WARN_COND(img_ptr_l->image.empty(), "LEFT IMAGE IS EMPTY!!");
    ROS_WARN_COND(img_ptr_r->image.empty(), "RIGHT IMAGE IS EMPTY!!");

    ROS_ASSERT(img_msg_left->width == img_msg_right->width);
    ROS_ASSERT(img_msg_left->height == img_msg_right->height);
    lvt_pose sl_pose = m_vo_system->track(img_ptr_l->image, img_ptr_r->image);
    if (m_vo_system->get_state() == lvt_system::eState_LOST)
    {
        ROS_ERROR("Tracking was lost. Reseting VO.");
        m_vo_system->reset();
        if (m_reset_pose_on_lost_vo)
        {
            m_base_to_odom.setIdentity();
            m_last_rotation = m_rot_fix;
            m_last_position.setZero();
            init_transforms();
        }

        return;
    }

     const lvt_matrix33 current_rotation_mtrx =  sl_pose.get_orientation_matrix();
     const lvt_matrix33 rot_delta = current_rotation_mtrx * m_last_rotation.transpose();
     const lvt_quaternion delta_q = lvt_quaternion(rot_delta);
     const lvt_vector3 current_pos = sl_pose.get_position();
     const lvt_vector3 pos_delta = current_pos - m_last_position;

    tf2::Transform delta_odom_sensor_tf;
     delta_odom_sensor_tf.setOrigin(tf2::Vector3(pos_delta.z(), -pos_delta.x(), -pos_delta.y()));
     delta_odom_sensor_tf.setRotation(tf2::Quaternion(delta_q.z(), -delta_q.x(), -delta_q.y(), delta_q.w()));

    lvt_vector3 current_position = sl_pose.get_position();
    lvt_quaternion current_q = sl_pose.get_orientation_quaternion();
    geometry_msgs::Transform pose_msg;
    // You can uncomment the following to test with the coordinate correction
     pose_msg.translation.x = current_position.z();
     pose_msg.translation.y = -current_position.x();
     pose_msg.translation.z = -current_position.y();
     pose_msg.rotation.x = current_q.z();
     pose_msg.rotation.y = -current_q.x();
     pose_msg.rotation.z = -current_q.y();
     pose_msg.rotation.w = current_q.w();
//    pose_msg.translation.x = current_position.x();
//    pose_msg.translation.y = current_position.y();
//    pose_msg.translation.z = current_position.z();
//    pose_msg.rotation.x = current_q.x();
//    pose_msg.rotation.y = current_q.y();
//    pose_msg.rotation.z = current_q.z();
//    pose_msg.rotation.w = current_q.w();

//    tf2::fromMsg(deltaTransf, delta_odom_sensor_tf);

    // tf2::Transform delta_odom_base_tf = m_base_to_sensor * delta_odom_sensor_tf * m_base_to_sensor.inverse();
    //      m_base_to_odom = m_base_to_odom * delta_odom_base_tf;
    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = m_odom_frame_id;
    odometry_msg.child_frame_id = m_baselink_frame_id;
    //geometry_msgs::Transform base2odom = tf2::toMsg(m_base_to_odom);
    odometry_msg.pose.pose.position.x = pose_msg.translation.x;
    odometry_msg.pose.pose.position.y = pose_msg.translation.y;
    odometry_msg.pose.pose.position.z = pose_msg.translation.z;
    odometry_msg.pose.pose.orientation.x = pose_msg.rotation.x;
    odometry_msg.pose.pose.orientation.y = pose_msg.rotation.y;
    odometry_msg.pose.pose.orientation.z = pose_msg.rotation.z;
    odometry_msg.pose.pose.orientation.w = pose_msg.rotation.w;

    if (!m_last_update_time.isZero())
    {
        double delta_t = (timestamp - m_last_update_time).toSec();
        int x = 0;
        if (delta_t)
        {
            odometry_msg.twist.twist.linear.x = delta_odom_sensor_tf.getOrigin().getX() / delta_t;
            odometry_msg.twist.twist.linear.y = delta_odom_sensor_tf.getOrigin().getY() / delta_t;
            odometry_msg.twist.twist.linear.z = delta_odom_sensor_tf.getOrigin().getZ() / delta_t;
            tf2::Quaternion delta_rot = delta_odom_sensor_tf.getRotation();
            tf2Scalar angle = delta_rot.getAngle();
            tf2::Vector3 axis = delta_rot.getAxis();
            tf2::Vector3 angular_twist = axis * angle / delta_t;
            odometry_msg.twist.twist.angular.x = angular_twist.x();
            odometry_msg.twist.twist.angular.y = angular_twist.y();
            odometry_msg.twist.twist.angular.z = angular_twist.z();
        }
    }

    m_odom_pub.publish(odometry_msg);

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = timestamp;
    transform_stamped.header.frame_id = m_odom_frame_id;
    transform_stamped.child_frame_id = m_baselink_frame_id;
    transform_stamped.transform = pose_msg; // tf2::toMsg(m_base_to_odom);
    m_tf_broadcaster.sendTransform(transform_stamped);

    m_last_update_time = timestamp;
    m_last_rotation = current_rotation_mtrx;
    m_last_position = current_pos;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lvt");
    lvt_ros vo("raw");
    ros::spin();
    return 0;
}