/*
    ROS driver to broadcast stereo images from a stereo webcam (eg. Minoru)    
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
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sstream>

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

#include "libcam.h"
#include "libcam.cpp"

int main(int argc, char** argv)
{
  std::string dev0 = "/dev/video1";
  std::string dev1 = "/dev/video0";

  const int fps = 30;
  const int ww = 320;
  const int hh = 240;

  ros::init(argc, argv, "stereocam_broadcast");
  ros::NodeHandle n;
  ros::Publisher left_pub = n.advertise<sensor_msgs::Image>("stereo/left/image_raw", fps);
  ros::Publisher right_pub = n.advertise<sensor_msgs::Image>("stereo/right/image_raw", fps);
  ros::Rate loop_rate(20);
  int count = 0;

  Camera c(dev0.c_str(), ww, hh, fps);
  Camera c2(dev1.c_str(), ww, hh, fps);

  IplImage *l=cvCreateImage(cvSize(ww, hh), 8, 3);
  IplImage *r=cvCreateImage(cvSize(ww, hh), 8, 3);

  unsigned char *l_=(unsigned char *)l->imageData;
  unsigned char *r_=(unsigned char *)r->imageData;

  sensor_msgs::Image left;
  sensor_msgs::Image right;

  left.width  = ww;
  left.height = hh;
  left.step = ww * 3;
  left.encoding = "bgr8";
  left.set_data_size(ww*hh*3);

  right.width  = ww;
  right.height = hh;
  right.step = ww * 3;
  right.encoding = "bgr8";
  right.set_data_size(ww*hh*3);

  while (ros::ok())
  {
    // Read image data
    while(c.Get()==0 || c2.Get()==0) usleep(100);

    // Convert to IplImage
    c.toIplImage(l);
    c2.toIplImage(r);

    // Convert to sensor_msgs::Image
    memcpy ((void*)(&left.data[0]), (void*)l_, ww*hh*3);
    memcpy ((void*)(&right.data[0]), (void*)r_, ww*hh*3);

    // Publish
    left_pub.publish(left);
    right_pub.publish(right);

    ROS_INFO("Stereo images published");
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  cvReleaseImage(&l);
  cvReleaseImage(&r);
}

