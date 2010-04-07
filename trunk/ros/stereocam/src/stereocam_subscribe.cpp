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
#include <cv.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "stereocam/camera_active.h"
#include "stereocam/stereocam_params.h"
#include "cv_bridge/CvBridge.h"
#include <image_transport/image_transport.h>

IplImage* left = NULL;
IplImage* right = NULL;
sensor_msgs::CvBridge bridge_;
ros::ServiceClient client_camera_params;
ros::ServiceClient client_camera_active;

void process_images(
    IplImage* left_image,
    IplImage* right_image)
{
}

void leftImageCallback(const sensor_msgs::ImageConstPtr& ptr)
{
  ROS_INFO("Received left image");

  try
  {
      left = bridge_.imgMsgToCv(ptr, "bgr8");
  }
  catch (sensor_msgs::CvBridgeException error)
  {
      ROS_ERROR("Error converting left image to IplImage");
  }
}

void rightImageCallback(const sensor_msgs::ImageConstPtr& ptr)
{
  ROS_INFO("Received right image");

  try
  {
      right = bridge_.imgMsgToCv(ptr, "bgr8");
      if (left != NULL) process_images(left, right);
  }
  catch (sensor_msgs::CvBridgeException error)
  {
      ROS_ERROR("Error converting right image to IplImage");
  }
}

void camera_on()
{
  stereocam::camera_active srv;
  srv.request.camera_active = 1;
  if (client_camera_active.call(srv)) {
      if ((int)srv.response.ack == 1) {
          ROS_INFO("Camera On");
      }
      else {
          if ((int)srv.response.ack == -1) {
              ROS_WARN("Camera parameters must be specified using set_stereo_camera_params before starting the camera");
          }
          else {
              ROS_ERROR("Failed to activate camera");
          }
      }
  }
  else {
      ROS_ERROR("Failed to activate camera");
  }
}

void camera_off()
{
  stereocam::camera_active srv;
  srv.request.camera_active = 0;
  if (client_camera_active.call(srv)) {
      if ((int)srv.response.ack == 1) {
          ROS_INFO("Camera Off");
      }
      else {
          ROS_ERROR("Failed to deactivate camera");
      }
  }
  else {
      ROS_ERROR("Failed to deactivate camera");
  }
}

void set_stereo_camera_params(
    std::string left_device,
    std::string right_device,
    int width,
    int height,
    int fps)
{
  stereocam::stereocam_params srv;
  srv.request.left_device = left_device;
  srv.request.right_device = right_device;
  srv.request.width = width;
  srv.request.height = height;
  srv.request.fps = fps;
  if (client_camera_params.call(srv)) {
      if ((int)srv.response.ack == -1) {
          ROS_WARN("Can't change image properties whilst the cameras are running");
      }
      else {
          ROS_INFO("Changed stereo camera parameters: %d", (int)srv.response.ack);
      }
  }
  else {
      ROS_ERROR("Failed to call service stereocam_params");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereocam_subscribe");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber left_sub = it.subscribe("stereo/left/image_raw", 1, leftImageCallback);
  image_transport::Subscriber right_sub = it.subscribe("stereo/right/image_raw", 1, rightImageCallback);
  client_camera_active = n.serviceClient<stereocam::camera_active>("camera_active");
  client_camera_params = n.serviceClient<stereocam::stereocam_params>("stereocam_params");

  set_stereo_camera_params("/dev/video1","/dev/video0",320,240,30);
  camera_on();

  ros::spin();
}

