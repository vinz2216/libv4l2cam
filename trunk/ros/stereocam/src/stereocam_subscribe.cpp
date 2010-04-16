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
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include "stereocam/camera_active.h"
#include "stereocam/stereocam_params.h"
#include "stereocam/densestereo_params.h"
#include "cv_bridge/CvBridge.h"
#include <image_transport/image_transport.h>

// whether to display the stereo images
bool show_images = true;

IplImage* left = NULL;
IplImage* right = NULL;
IplImage* disparity = NULL;
sensor_msgs::CvBridge bridge_;
ros::ServiceClient client_camera_params;
ros::ServiceClient client_densestereo_params;
ros::ServiceClient client_camera_active;
std::string left_image_title = "Left Image";
std::string right_image_title = "Right Image";
std::string disparity_image_title = "Disparity Image";

/*!
 * \brief callback when a disparity image is received
 * \param ptr pointer to the image
 */
void disparityImageCallback(const sensor_msgs::ImageConstPtr& ptr)
{
  ROS_INFO("Received disparity image");

  try
  {
      disparity = bridge_.imgMsgToCv(ptr, "mono8");
      if (show_images) cvShowImage(disparity_image_title.c_str(), disparity);
  }
  catch (sensor_msgs::CvBridgeException error)
  {
      ROS_ERROR("Error converting disparity image to IplImage");
  }

}

/*!
 * \brief callback when the left image is received
 * \param ptr pointer to the image
 */
void leftImageCallback(const sensor_msgs::ImageConstPtr& ptr)
{
  ROS_INFO("Received left image");

  try
  {
      left = bridge_.imgMsgToCv(ptr, "bgr8");
      if (show_images) cvShowImage(left_image_title.c_str(), left);
  }
  catch (sensor_msgs::CvBridgeException error)
  {
      ROS_ERROR("Error converting left image to IplImage");
  }
}

/*!
 * \brief callback when the right image is received
 * \param ptr pointer to the image
 */
void rightImageCallback(const sensor_msgs::ImageConstPtr& ptr)
{
  ROS_INFO("Received right image");

  try
  {
      right = bridge_.imgMsgToCv(ptr, "bgr8");
      if (show_images) cvShowImage(right_image_title.c_str(), right);
  }
  catch (sensor_msgs::CvBridgeException error)
  {
      ROS_ERROR("Error converting right image to IplImage");
  }
}

/*!
 * \brief callback when a point cloud is received
 */
void PointCloudImageCallback(const sensor_msgs::PointCloudConstPtr& ptr)
{
  ROS_INFO("Received point cloud");

  sensor_msgs::PointCloud point_cloud = *ptr;
}

/*!
 * \brief turn the stereo camera on
 */
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

/*!
 * \brief turn the stereo camera off
 */
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

/*!
 * \brief set the stereo camera parameters
 * \param left_device left camera device, eg. /dev/video1
 * \param left_device right camera device, eg. /dev/video0
 * \param width width of the image
 * \param height height of the image
 * \param fps frames per second
 * \param baseline_mm stereo baseline in millimetres
 * \param focal_length_pixels focal length in pixels
 */
void set_stereo_camera_params(
    std::string left_device,
    std::string right_device,
    int width,
    int height,
    int fps,
    int baseline_mm,
    int focal_length_pixels)
{
  stereocam::stereocam_params srv;
  srv.request.left_device = left_device;
  srv.request.right_device = right_device;
  srv.request.width = width;
  srv.request.height = height;
  srv.request.fps = fps;
  srv.request.baseline_mm = baseline_mm;
  srv.request.focal_length_pixels = focal_length_pixels;
  if (client_camera_params.call(srv)) {
      if ((int)srv.response.ack == -1) {
          ROS_WARN("Can't change image properties whilst the cameras are running");
      }
      else {
          ROS_INFO("Changed stereo camera parameters");
      }
  }
  else {
      ROS_ERROR("Failed to call service stereocam_params");
  }
}

/*!
 * \brief set the parameters used for dense stereo
 * \param offset_x calibration x offset
 * \param offset_y calibration y offset
 * \param vertical_sampling vertical sampling rate
 * \param max_disparity_percent maximum disparity as a percentage of image width
 * \param correlation_radius radius used for SAD correlation
 * \param smoothing_radius radius used to smooth the disparity space
 * \param disparity_step disparity step size
 * \param disparity_threshold_percent used to apply a threshold to the disparity map
 * \param despeckle whether to despeckle the disparity map
 * \param cross_checking_threshold threshold used to check pixel values between images
 */
void set_dense_stereo_params(
    int offset_x,
    int offset_y,
    int vertical_sampling,
    int max_disparity_percent,
    int correlation_radius,
    int smoothing_radius,
    int disparity_step,
    int disparity_threshold_percent,
    bool despeckle,
    int cross_checking_threshold)
{
  stereocam::densestereo_params srv;
  srv.request.offset_x = offset_x;
  srv.request.offset_y = offset_y;
  srv.request.vertical_sampling = vertical_sampling;
  srv.request.max_disparity_percent = max_disparity_percent;
  srv.request.correlation_radius = correlation_radius;
  srv.request.smoothing_radius = smoothing_radius;
  srv.request.disparity_step = disparity_step;
  srv.request.disparity_threshold_percent = disparity_threshold_percent;
  srv.request.despeckle = despeckle;
  srv.request.cross_checking_threshold = cross_checking_threshold;
  if (client_densestereo_params.call(srv)) {
      ROS_INFO("Changed dense stereo parameters");
  }
  else {
      ROS_ERROR("Failed to call service densestereo_params");
  }
}

int main(int argc, char** argv)
{
  if (show_images) {
      cvNamedWindow(left_image_title.c_str(), CV_WINDOW_AUTOSIZE);
      cvNamedWindow(right_image_title.c_str(), CV_WINDOW_AUTOSIZE);
  }

  ros::init(argc, argv, "stereocam_subscribe");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber left_sub = it.subscribe("stereo/left/image_raw", 1, leftImageCallback);
  image_transport::Subscriber right_sub = it.subscribe("stereo/right/image_raw", 1, rightImageCallback);
  image_transport::Subscriber disparity_sub = it.subscribe("stereo/image_disparity", 1, disparityImageCallback);
  ros::Subscriber point_cloud_sub = n.subscribe("stereo/point_cloud", 1, PointCloudImageCallback);
  client_camera_active = n.serviceClient<stereocam::camera_active>("camera_active");
  client_camera_params = n.serviceClient<stereocam::stereocam_params>("stereocam_params");
  client_densestereo_params = n.serviceClient<stereocam::densestereo_params>("densestereo_params");

  int offset_x = -16;
  int offset_y = 2;
  int max_disparity_percent = 30;
  set_stereo_camera_params("/dev/video1","/dev/video0",320,240,30,60,150);
  set_dense_stereo_params(offset_x,offset_y,2,max_disparity_percent,1,2,8,0,true,30);

  camera_on();

  if (show_images) {
      ros::Rate loop_rate(30);
      while(1) {    
        ros::spinOnce();
        loop_rate.sleep();

        int wait = cvWaitKey(10) & 255;
        if( wait == 27 ) break;
      }
  }
  else {
      ros::spin();
  }
  return 0;
}

