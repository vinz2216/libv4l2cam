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
#include <image_transport/image_transport.h>
#include <sstream>

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

#include "stereocam/camera_active.h"
#include "stereocam/stereocam_params.h"
#include "libcam.h"
#include "libcam.cpp"

std::string dev0 = "";
std::string dev1 = "";

int fps = 30;
Camera *c1 = NULL;
Camera *c2 = NULL;
sensor_msgs::Image left;
sensor_msgs::Image right;
bool cam_active = false;
bool cam_active_request = false;

/*!
 * \brief stop the stereo camera
 * \param left_camera left camera object
 * \param right_camera right camera object
 */
void stop_cameras(
  Camera *&left_camera,
  Camera *&right_camera)
{
    if (left_camera != NULL) {
        delete left_camera;
        delete right_camera;
        left_camera = NULL;
        right_camera = NULL;
        cam_active = false;
    }
}

/*!
 * \brief received a request to start or stop the stereo camera
 * \param req requested parameters
 * \param res returned parameters
 */
bool camera_active(
  stereocam::camera_active::Request &req,
  stereocam::camera_active::Response &res)
{
    if ((int)req.camera_active > 0) {
        if (dev0 != "") {
            ROS_INFO("Camera On");
            cam_active_request = true;
            res.ack = 1;
        }
        else {
            ROS_WARN("Camera parameters have not been specified");
            res.ack = -1;
        }
    }
    else {
        ROS_INFO("Camera Off");
        cam_active_request = false;
        res.ack = 1;
    }
    
    return(true);
} 

/*!
 * \brief service requests camera parameters to be changed
 * \param req requested parameters
 * \param res returned parameters
 */ 
bool request_params(
  stereocam::stereocam_params::Request &req,
  stereocam::stereocam_params::Response &res)
{
    if (!cam_active) {
        ROS_INFO("Resolution: %dx%d", (int)req.width, (int)req.height);

        dev0 = req.left_device;
        dev1 = req.right_device;

        left.width  = (int)req.width;
        left.height = (int)req.height;
        left.step = (int)req.width * 3;
        left.encoding = "bgr8";
        left.set_data_size((int)req.width*(int)req.height*3);

        right.width  = (int)req.width;
        right.height = (int)req.height;
        right.step = (int)req.width * 3;
        right.encoding = "bgr8";
        right.set_data_size((int)req.width*(int)req.height*3);

        fps = (int)req.fps;

        res.ack = 1;
    }
    else {
        ROS_WARN("Can't change image properties whilst the cameras are running");
        res.ack = -1;
    }
    return(true);
} 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereocam_broadcast");
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);
  image_transport::Publisher left_pub = it.advertise("stereo/left/image_raw", 1);
  image_transport::Publisher right_pub = it.advertise("stereo/right/image_raw", 1);
  ros::Rate loop_rate(20);
  int count = 0;

  IplImage *l=NULL;
  IplImage *r=NULL;
  unsigned char *l_=NULL;
  unsigned char *r_=NULL;

  // start service which can be used to start and stop the stereo camera
  ros::ServiceServer service_active = n.advertiseService("camera_active", camera_active);

  // start service which can be used to change camera parameters
  ros::ServiceServer service_params = n.advertiseService("stereocam_params", request_params);

  ROS_INFO("Stereo camera node running");
  ROS_INFO("Waiting for subscribers...");

  while (ros::ok())
  {
    // request to turn camera on or off
    if (cam_active_request != cam_active) {
        if (cam_active_request == false) {
            stop_cameras(c1,c2);
        }
        else {
            stop_cameras(c1, c2);

            // create appropriately sized images
            if (l_ != NULL) {
                cvReleaseImage(&l);
                cvReleaseImage(&r);
            }
            l=cvCreateImage(cvSize(left.width, left.height), 8, 3);
            r=cvCreateImage(cvSize(right.width, right.height), 8, 3);

            l_=(unsigned char *)l->imageData;
            r_=(unsigned char *)r->imageData;

            // start the cameras
            c1 = new Camera(dev0.c_str(), left.width, left.height, fps);
            c2 = new Camera(dev1.c_str(), right.width, right.height, fps);
            cam_active = true;
        }
    }

    if (cam_active) {

        // Read image data
        while(c1->Get()==0 || c2->Get()==0) usleep(100);

        // Convert to IplImage
        c1->toIplImage(l);
        c2->toIplImage(r);

        // Convert to sensor_msgs::Image
        memcpy ((void*)(&left.data[0]), (void*)l_, left.width*left.height*3);
        memcpy ((void*)(&right.data[0]), (void*)r_, right.width*right.height*3);

        // Publish
        left_pub.publish(left);
        right_pub.publish(right);

        ROS_INFO("Stereo images published");
    }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  if (l_ != NULL) {
      cvReleaseImage(&l);
      cvReleaseImage(&r);
  }
  stop_cameras(c1,c2);
}

