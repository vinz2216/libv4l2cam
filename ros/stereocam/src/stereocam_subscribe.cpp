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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

void leftImageCallback(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO("Received left image");
}

void rightImageCallback(const sensor_msgs::ImageConstPtr& img)
{
  ROS_INFO("Received right image");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereocam_subscribe");
  ros::NodeHandle n;
  ros::Subscriber left_sub = n.subscribe("stereo/left/image_raw", 30, leftImageCallback);
  ros::Subscriber right_sub = n.subscribe("stereo/right/image_raw", 30, rightImageCallback);
  ros::spin();
}

