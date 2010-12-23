/*
    Example ROS subscription to stereo images being broadcast from a stereo webcam (eg. Minoru)
    Copyright (C) 2010 Bob Mottram and Giacomo Spigler
    fuzzgun@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "stereocam/camera_active.h"

ros::ServiceClient client_camera_active;

/*!
 * \brief callback when a point cloud is received
 */
void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& ptr)
{
    ROS_INFO("Received point cloud");

    sensor_msgs::PointCloud point_cloud = *ptr;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereocam_subscribe");
    ros::NodeHandle n;

    std::string topic_str = "stereo/point_cloud";
    ros::Subscriber point_cloud_sub = n.subscribe(topic_str, 1, PointCloudCallback);

    std::string service_str = "camera_active";
    client_camera_active = n.serviceClient<stereocam::camera_active>(service_str);

    ros::spin();
    return 0;
}


