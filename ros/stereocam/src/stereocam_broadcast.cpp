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
#include "stereocam/densestereo_params.h"
#include "libcam.h"
#include "libcam.cpp"
#include "stereodense.h"
#include "stereodense.cpp"

// if you wish the cameras to begin broadcasting immediately
// instead of waiting for subscribers then set this flag
bool broadcast_immediately = false;

std::string dev0 = "";
std::string dev1 = "";

int fps = 30;
Camera *c1 = NULL;
Camera *c2 = NULL;
sensor_msgs::Image left_image;
sensor_msgs::Image right_image;
sensor_msgs::Image disparity;
bool cam_active = false;
bool cam_active_request = false;

// dense stereo parameters
int offset_x = 0;
int offset_y = 0;
int vertical_sampling = 2;
int max_disparity_percent = 50;
int correlation_radius = 1;
int smoothing_radius = 2;
int disparity_step = 8;
int disparity_threshold_percent = 0;
bool despeckle = true;
int cross_checking_threshold = 50;

float baseline_mm = 60;
float focal_length_pixels = 150;

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
        ROS_INFO("Left camera:  %s", req.left_device.c_str());
        ROS_INFO("Right camera: %s", req.right_device.c_str());
        ROS_INFO("Resolution: %dx%d", (int)req.width, (int)req.height);

        dev0 = req.left_device;
        dev1 = req.right_device;

        left_image.width  = (int)req.width;
        left_image.height = (int)req.height;
        left_image.step = (int)req.width * 3;
        left_image.encoding = "bgr8";
        left_image.set_data_size((int)req.width*(int)req.height*3);

        right_image.width  = (int)req.width;
        right_image.height = (int)req.height;
        right_image.step = (int)req.width * 3;
        right_image.encoding = "bgr8";
        right_image.set_data_size((int)req.width*(int)req.height*3);

        fps = (int)req.fps;

        disparity.width = left_image.width;
        disparity.height = left_image.height;
        disparity.encoding = "mono8";
        disparity.set_data_size(disparity.width*disparity.height);

        baseline_mm = (float)req.baseline_mm;
        focal_length_pixels = (float)req.focal_length_pixels;

        res.ack = 1;
    }
    else {
        ROS_WARN("Can't change image properties whilst the cameras are running");
        res.ack = -1;
    }
    return(true);
} 

/*!
 * \brief service requests dense stereo parameters to be changed
 * \param req requested parametersflo
 * \param res returned parameters
 */ 
bool request_dense_stereo_params(
  stereocam::densestereo_params::Request &req,
  stereocam::densestereo_params::Response &res)
{
    offset_x = (int)req.offset_x;
    offset_y = (int)req.offset_y;
    vertical_sampling = (int)req.vertical_sampling;
    max_disparity_percent = (int)max_disparity_percent;
    correlation_radius = (int)req.correlation_radius;
    smoothing_radius = (int)req.smoothing_radius;
    disparity_step = (int)req.disparity_step;
    disparity_threshold_percent = (int)req.disparity_threshold_percent;
    despeckle = (bool)req.despeckle;
    cross_checking_threshold = (int)req.cross_checking_threshold;

    res.ack = 1;
    return(true);
} 

/*!
 * \brief convert the disparity map to stereo_msgs::DisparityImage
 * \param disparity_map disparity map data
 * \param disp_image DisparityImage to be updated
 * \param vertical_sampling vertical sampling rate
 * \param smoothing_radius radius used to smooth the disparity space
 * \param max_disparity_percent maximum disparity as a percent of image width
 * \param sub_pixel_multiplier multiplier used to convert floating point disparity to an integer value
 */
void disparity_map_to_DisparityImage(
    unsigned int* disparity_map,
    sensor_msgs::Image &disp_image,
    int vertical_sampling,
    int smoothing_radius,
    int max_disparity_percent,
    int sub_pixel_multiplier,
    float baseline_mm,
    float focal_length_pixels)
{
    int smooth_vert = 2;
    int img_width = disp_image.width;
    int img_height = disp_image.height;
    int max_disparity_pixels = max_disparity_percent * disp_image.width * sub_pixel_multiplier / 100;
    int width2 = img_width / smoothing_radius;

    unsigned char* data = (unsigned char*)(&disp_image.data[0]);

    for (int y = 0; y < img_height; y++) {
        int n2 = ((y / vertical_sampling) / smooth_vert) * width2;
	for (int x = 0; x < img_width; x++) {
	    int n = y*img_width + x;
	    int n2b = (n2 + (x / smoothing_radius)) * 2;
            data[n] = (unsigned char)(disparity_map[n2b + 1] * 255 / max_disparity_pixels);
	}
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereocam_broadcast");
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);
  image_transport::Publisher left_pub = it.advertise("stereo/left/image_raw", 1);
  image_transport::Publisher right_pub = it.advertise("stereo/right/image_raw", 1);
  image_transport::Publisher disparity_pub = it.advertise("stereo/image_disparity", 1);
  ros::Rate loop_rate(30);
  int count = 0;

  IplImage *l=NULL;
  IplImage *r=NULL;
  unsigned char *l_=NULL;
  unsigned char *r_=NULL;

  // start service which can be used to start and stop the stereo camera
  ros::ServiceServer service_active = n.advertiseService("camera_active", camera_active);

  // start service which can be used to change camera parameters
  ros::ServiceServer service_params = n.advertiseService("stereocam_params", request_params);

  // start service which can be used to change camera parameters
  ros::ServiceServer service_densestereo = n.advertiseService("densestereo_params", request_dense_stereo_params);

  if (!broadcast_immediately) {
      // wait for subscribers
      ROS_INFO("Stereo camera node running");
      ROS_INFO("Waiting for subscribers...");
  }
  else {
      // set some default values
      dev0 = "/dev/video1";
      dev1 = "/dev/video0";

      left_image.width  = 320;
      left_image.height = 240;
      left_image.step = left_image.width * 3;
      left_image.encoding = "bgr8";
      left_image.set_data_size(left_image.width*left_image.height*3);

      right_image.width  = left_image.width;
      right_image.height = left_image.height;
      right_image.step = right_image.width * 3;
      right_image.encoding = "bgr8";
      right_image.set_data_size(right_image.width*right_image.height*3);

      disparity.width = left_image.width;
      disparity.height = left_image.height;
      disparity.encoding = "mono8";
      disparity.set_data_size(disparity.width*disparity.height);

      fps = 30;
      cam_active_request = true;
  }

  const int MAX_IMAGE_WIDTH = 640;
  const int MAX_IMAGE_HEIGHT = 480;
  const int VERTICAL_SAMPLING = 2;
  int max_disparity_pixels = MAX_IMAGE_WIDTH * max_disparity_percent / 100;
  int disparity_space_length = (max_disparity_pixels / disparity_step) * MAX_IMAGE_WIDTH * ((MAX_IMAGE_HEIGHT/VERTICAL_SAMPLING)/smoothing_radius) * 2;
  int disparity_map_length = MAX_IMAGE_WIDTH * ((MAX_IMAGE_HEIGHT/VERTICAL_SAMPLING)/smoothing_radius) * 2;
  unsigned int* disparity_space = new unsigned int[disparity_space_length];
  unsigned int* disparity_map = new unsigned int[disparity_map_length];

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
            l=cvCreateImage(cvSize(left_image.width, left_image.height), 8, 3);
            r=cvCreateImage(cvSize(right_image.width, right_image.height), 8, 3);

            l_=(unsigned char *)l->imageData;
            r_=(unsigned char *)r->imageData;

            // start the cameras
            c1 = new Camera(dev0.c_str(), left_image.width, left_image.height, fps);
            c2 = new Camera(dev1.c_str(), right_image.width, right_image.height, fps);
            cam_active = true;
        }
    }

    if (cam_active) {

        // Read image data
        while(c1->Get()==0 || c2->Get()==0) usleep(100);

        // Convert to IplImage
        c1->toIplImage(l);
        c2->toIplImage(r);

        // create disparity map
	stereodense::update_disparity_map(
	    l_,r_,left_image.width,left_image.height,
  	    offset_x, offset_y,
            vertical_sampling,
	    max_disparity_percent,
	    correlation_radius,
	    smoothing_radius,
	    disparity_step,
	    disparity_threshold_percent,
	    despeckle,
	    cross_checking_threshold,
	    disparity_space,
	    disparity_map);

        // Convert to sensor_msgs::Image
        memcpy ((void*)(&left_image.data[0]), (void*)l_, left_image.width*left_image.height*3);
        memcpy ((void*)(&right_image.data[0]), (void*)r_, right_image.width*right_image.height*3);

        // convert to stereo_msgs::DisparityImage format
        disparity_map_to_DisparityImage(
            disparity_map,
            disparity,
            vertical_sampling,
            smoothing_radius,
            max_disparity_percent,
            16,
            baseline_mm,
            focal_length_pixels);

        // Publish
        left_pub.publish(left_image);
        right_pub.publish(right_image);
        disparity_pub.publish(disparity);

        ROS_INFO("Stereo images published");
    }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  delete[] disparity_space;
  delete[] disparity_map;
  if (l_ != NULL) {
      cvReleaseImage(&l);
      cvReleaseImage(&r);
  }
  stop_cameras(c1,c2);
}

