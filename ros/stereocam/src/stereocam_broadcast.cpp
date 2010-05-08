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
#include <sensor_msgs/PointCloud.h>
#include <image_transport/image_transport.h>
#include <sstream>

#include <omp.h>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

#include "stereocam/camera_active.h"
#include "stereocam/stereocam_params.h"
#include "stereocam/densestereo_params.h"
#include "stereocam/featurestereo_params.h"
#include "libcam.h"
#include "libcam.cpp"
#include "polynomial.h"
#include "polynomial.cpp"
#include "stereo.h"
#include "stereo.cpp"
#include "stereodense.h"
#include "stereodense.cpp"

// if you wish the cameras to begin broadcasting immediately
// instead of waiting for subscribers then set this flag
bool broadcast_immediately = false;

bool flip_left_image = false;
bool flip_right_image = false;

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

// an index number assigned to the stereo camera
int stereo_camera_index = 0;

bool publish_feature_matches = false;
bool publish_disparity_map = false;

// feature based stereo parameters
svs* lcam = NULL;
svs* rcam = NULL;
int inhibition_radius = 6;
unsigned int minimum_response = 25;
int learnDesc = 18*5;  /* weight associated with feature descriptor match */
int learnLuma = 7*5;   /* weight associated with luminance match */
int learnDisp = 1;     /* weight associated with disparity (bias towards smaller disparities) */
int learnGrad = 4;     /* weight associated with horizontal gradient */
int groundPrior = 200; /* weight for ground plane prior */
int use_priors = 1;

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

        flip_left_image = req.flip_left;
        flip_right_image = req.flip_right;

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
 * \param req requested parameters
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
    publish_disparity_map = true;
    return(true);
}

/*!
 * \brief service requests feature stereo parameters to be changed
 * \param req requested parameters
 * \param res returned parameters
 */
bool request_feature_stereo_params(
    stereocam::featurestereo_params::Request &req,
    stereocam::featurestereo_params::Response &res)
{
    offset_x = (int)req.offset_x;
    offset_y = (int)req.offset_y;
    max_disparity_percent = (int)max_disparity_percent;
    res.ack = 1;
    publish_feature_matches = true;
    return(true);
}

/*!
 * \brief convert the disparity map to stereo_msgs::Image
 * \param disparity_map disparity map data
 * \param disp_image Image to be updated
 * \param vertical_sampling vertical sampling rate
 * \param smoothing_radius radius used to smooth the disparity space
 * \param max_disparity_percent maximum disparity as a percent of image width
 * \param sub_pixel_multiplier multiplier used to convert floating point disparity to an integer value
 */
void disparity_map_to_Image(
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

/*!
 * \brief does the line intersect with the given line?
 * \param x0 first line top x
 * \param y0 first line top y
 * \param x1 first line bottom x
 * \param y1 first line bottom y
 * \param x2 second line top x
 * \param y2 second line top y
 * \param x3 second line bottom x
 * \param y3 second line bottom y
 * \param xi intersection x coordinate
 * \param yi intersection y coordinate
 * \return true if the lines intersect
 */
bool intersection(
    float x0,
    float y0,
    float x1,
    float y1,
    float x2,
    float y2,
    float x3,
    float y3,
    float& xi,
    float& yi)
{
    float a1, b1, c1,         //constants of linear equations
    a2, b2, c2,
    det_inv,            //the inverse of the determinant of the coefficient
    m1, m2, dm;         //the gradients of each line
    bool insideLine = false;  //is the intersection along the lines given, or outside them
    float tx, ty, bx, by;

    //compute gradients, note the cludge for infinity, however, this will
    //be close enough
    if ((x1 - x0) != 0)
        m1 = (y1 - y0) / (x1 - x0);
    else
        m1 = (float)1e+10;   //close, but no cigar

    if ((x3 - x2) != 0)
        m2 = (y3 - y2) / (x3 - x2);
    else
        m2 = (float)1e+10;   //close, but no cigar

    dm = fabs(m1 - m2);
    if (dm > 0.000001f)
    {
        //compute constants
        a1 = m1;
        a2 = m2;

        b1 = -1;
        b2 = -1;

        c1 = (y0 - m1 * x0);
        c2 = (y2 - m2 * x2);

        //compute the inverse of the determinate
        det_inv = 1 / (a1 * b2 - a2 * b1);

        //use Kramers rule to compute xi and yi
        xi = ((b1 * c2 - b2 * c1) * det_inv);
        yi = ((a2 * c1 - a1 * c2) * det_inv);

        //is the intersection inside the line or outside it?
        if (x0 < x1) {
            tx = x0;
            bx = x1;
        }
        else {
            tx = x1;
            bx = x0;
        }
        if (y0 < y1) {
            ty = y0;
            by = y1;
        }
        else {
            ty = y1;
            by = y0;
        }
        if ((xi >= tx) && (xi <= bx) && (yi >= ty) && (yi <= by))
        {
            if (x2 < x3) {
                tx = x2;
                bx = x3;
            }
            else {
                tx = x3;
                bx = x2;
            }
            if (y2 < y3) {
                ty = y2;
                by = y3;
            }
            else {
                ty = y3;
                by = y2;
            }
            if ((xi >= tx) && (xi <= bx) && (yi >= ty) && (yi <= by))
            {
                insideLine = true;
            }
        }
    }
    else
    {
        //parallel (or parallelish) lines, return some indicative value
        xi = 9999;
    }

    return (insideLine);
}

/*!
 * \brief turns the disparity map into a point cloud.  Multiple points are generated per disparity in order to try to represent the uncertainty
 * \param disparity_map disparity map
 * \param img_left left colour image
 * \param img_width width of the image
 * \param img_height height of the image
 * \param vertical_sampling vertical sampling rate
 * \param smoothing_radius radius used to smooth the disparity space
 * \param max_disparity_percent maximum disparity as a percent of image width
 * \param sub_pixel_multiplier multiplier used to convert floating point disparity to an integer value
 * \param baseline_mm stereo camera baseline
 * \param focal_length_pixels focal length in pixels
 * \param uncertainty_pixels position uncertainty for each observed disparity
 * \param stereo_camera_index an index number for the stereo camera
 * \param point_cloud returned point cloud
 */
void disparity_map_to_PointCloud(
    unsigned int* disparity_map,
    unsigned char* img_left,
    int img_width,
    int img_height,
    int vertical_sampling,
    int smoothing_radius,
    int max_disparity_percent,
    int sub_pixel_multiplier,
    float baseline_mm,
    float focal_length_pixels,
    float uncertainty_pixels,
    std::string stereo_camera_index_str,
    sensor_msgs::PointCloud &point_cloud)
{
    // each disparity is represented by this number of points in the cloud
    const int points_per_disparity = 4*4;

    // dimensions of the disparity map
    int disparity_map_width = img_width / smoothing_radius;
    int disparity_map_height = (img_height / vertical_sampling) / STEREO_DENSE_SMOOTH_VERTICAL;

    // find the number of non-zero disparity entries
    int no_of_points = 0;
    for (int i = disparity_map_width * disparity_map_height - 1; i >= 0; i--) {
        int n = i*2 + 1;
        if (disparity_map[n] > 0) {
            no_of_points += points_per_disparity;
        }
    }

    point_cloud.points.resize (no_of_points);
    point_cloud.channels.resize (3);
    point_cloud.channels[0].name = "r";
    point_cloud.channels[0].values.resize(no_of_points);
    point_cloud.channels[1].name = "g";
    point_cloud.channels[1].values.resize(no_of_points);
    point_cloud.channels[2].name = "b";
    point_cloud.channels[2].values.resize(no_of_points);

    // multiplier used to convert to metres
    float mult = 1.0f / 1000.0f;

    int baseline_x = baseline_mm / 2.0;
    float cx = img_width / 2.0f;
    float cy = img_height / 2.0f;
    float uncertainty = uncertainty_pixels * 2.0f / 4.0f;
    float uncertainty2 = uncertainty_pixels * 2.0f / 64.0f;
    no_of_points = 0;
    srand(0);
    for (int y = 0; y < disparity_map_height; y++)
    {

        int n = y*disparity_map_width*2 + 1;
        int img_y = y * (img_height-1) / disparity_map_height;
        float dz = cy - (img_y / (float)img_height);

        for (int x = 0; x < disparity_map_width; x++, n += 2)
        {
            int disparity = disparity_map[n];
            if (disparity > 0) {
                float disp = disparity / (float)sub_pixel_multiplier;
                int img_x = x * (img_width-1) / disparity_map_width;
                int n2 = (img_y*img_width + img_x)*3;

                int r = img_left[n2+2];
                int g = img_left[n2+1];
                int b = img_left[n2];

                // horizontal uncertainty
                for (int i = 0; i < 4; i++) {

                    float left_x = img_x - uncertainty_pixels + (i * uncertainty) - cx;
                    float x0 = -baseline_x;
                    float y0 = 0;
                    float x1 = left_x + x0;
                    float y1 = focal_length_pixels;

                    // vertical uncertainty
                    for (int j = 0; j < 4; j++) {

                        float right_x = img_x - disp - uncertainty_pixels + (j * uncertainty) - cx;
                        float x2 = baseline_x;
                        float y2 = 0;
                        float x3 = right_x + x2;
                        float y3 = focal_length_pixels;

                        float point_x = 0, point_y = 0;
                        intersection(x0,y0,x1,y1, x2,y2,x3,y3, point_x,point_y);
                        if (point_x != 9999) {

                            float fraction = point_y / focal_length_pixels;
                            // randomly adjust the z pixel position, to avoid generating too many points
                            float point_z = (dz - uncertainty_pixels + (((rand() % 128)-64)*uncertainty2)) * fraction;

                            // set the point position
                            point_cloud.points[no_of_points].x = point_x * mult;
                            point_cloud.points[no_of_points].y = point_y * mult;
                            point_cloud.points[no_of_points].z = point_z * mult;

                            // set the colour
                            point_cloud.channels[0].values[no_of_points] = r;
                            point_cloud.channels[1].values[no_of_points] = g;
                            point_cloud.channels[2].values[no_of_points] = b;

                        }

                        no_of_points++;
                    }
                }
            }
        }
    }

    point_cloud.header.stamp = ros::Time::now();
    point_cloud.header.frame_id = "stereo_cloud" + stereo_camera_index_str;
}

/*!
 * \brief turns feature matches into a point cloud.
 * \param no_of_matches number of feature matches
 * \param img_left left colour image
 * \param img_width width of the image
 * \param img_height height of the image
 * \param baseline_mm stereo camera baseline
 * \param focal_length_pixels focal length in pixels
 * \param stereo_camera_index an index number for the stereo camera
 * \param point_cloud returned point cloud
 */
void feature_matches_to_PointCloud(
    int no_of_matches,
    unsigned char* img_left,
    int img_width,
    int img_height,
    float baseline_mm,
    float focal_length_pixels,
    std::string stereo_camera_index_str,
    sensor_msgs::PointCloud &point_cloud)
{
    point_cloud.points.resize (no_of_matches);
    point_cloud.channels.resize (3);
    point_cloud.channels[0].name = "r";
    point_cloud.channels[0].values.resize(no_of_matches);
    point_cloud.channels[1].name = "g";
    point_cloud.channels[1].values.resize(no_of_matches);
    point_cloud.channels[2].name = "b";
    point_cloud.channels[2].values.resize(no_of_matches);

    int baseline_x = baseline_mm / 2.0;
    float cx = img_width / 2.0f;
    float cy = img_height / 2.0f;

    float x0 = -baseline_x;
    float y0 = 0;
    float x2 = baseline_x;
    float y2 = 0;
    float y1 = focal_length_pixels;
    float y3 = focal_length_pixels;

    // multiplier used to convert to metres
    float mult = 1.0f / 1000.0f;

    for (int i = 0; i < no_of_matches; i++) {
        if ((lcam->svs_matches[i*5] > 0) &&
                (lcam->svs_matches[i*5+4] != 9999)) {
            float x = lcam->svs_matches[i*5 + 1]/(float)SVS_SUB_PIXEL;
            float y = lcam->svs_matches[i*5 + 2];
            float dz = cy - (y / (float)img_height);
            float disp = lcam->svs_matches[i*5 + 3]/(float)SVS_SUB_PIXEL;

            int n = ((int)y*img_width + (int)x)*3;
            int r = img_left[n+2];
            int g = img_left[n+1];
            int b = img_left[n];

            float left_x = x - cx;
            float x1 = left_x + x0;

            float right_x = x - disp - cx;
            float x3 = right_x + x2;

            float point_x = 0, point_y = 0;
            intersection(x0,y0,x1,y1, x2,y2,x3,y3, point_x,point_y);
            if (point_x != 9999) {
                float fraction = point_y / focal_length_pixels;
                float point_z = dz * fraction;

                // set the point position in metres
                point_cloud.points[i].x = point_x * mult;
                point_cloud.points[i].y = point_y * mult;
                point_cloud.points[i].z = point_z * mult;

                // set the colour
                point_cloud.channels[0].values[i] = r;
                point_cloud.channels[1].values[i] = g;
                point_cloud.channels[2].values[i] = b;
            }
        }
    }

    point_cloud.header.stamp = ros::Time::now();
    point_cloud.header.frame_id = "stereo_feature_cloud" + stereo_camera_index_str;
    ROS_INFO("No of feature matches %d", no_of_matches);
}

/*!
 * \brief extracts edge features and stereo matches them
 * \param left_img left image data
 * \param right_img right image data
 * \param image_width width of the image
 * \param image_height height of the image
 * \param ideal_no_of_matches desired number of stereo matches to be returned
 * \param max_disparity_percent maximum stereo disparity as a percentage of image width
 * \return number of matched features
 */
int update_feature_matches(
    unsigned char* left_img,
    unsigned char* right_img,
    int image_width,
    int image_height,
    int ideal_no_of_matches,
    int max_disparity_percent)
{
    if (lcam == NULL) {
        lcam = new svs(image_width, image_height);
        rcam = new svs(image_width, image_height);
    }

#pragma omp parallel for
    for (int cam = 1; cam >= 0; cam--) {

        int calib_offset_x = 0;
        int calib_offset_y = 0;
        unsigned char* rectified_frame_buf = NULL;
        svs* stereo_cam = NULL;
        if (cam == 0) {
            rectified_frame_buf = left_img;
            stereo_cam = lcam;
            calib_offset_x = 0;
            calib_offset_y = 0;
        }
        else {
            rectified_frame_buf = right_img;
            stereo_cam = rcam;
            calib_offset_x = offset_x;
            calib_offset_y = offset_y;
        }

        int no_of_feats = stereo_cam->get_features_vertical(
                          rectified_frame_buf,
                          inhibition_radius,
                          minimum_response,
                          calib_offset_x,
                          calib_offset_y,
                          0);

        ROS_INFO("number of edges %d", no_of_feats);

        if (cam == 0) {
            stereo_cam->get_features_horizontal(
                rectified_frame_buf,
                inhibition_radius,
                minimum_response,
                calib_offset_x,
                calib_offset_y,
                0);
        }
    }

    lcam->enable_ground_priors = false;
    lcam->ground_y_percent = 0;

    int matches = lcam->match(
                      rcam,
                      ideal_no_of_matches,
                      max_disparity_percent,
                      learnDesc,
                      learnLuma,
                      learnDisp,
                      learnGrad,
                      groundPrior,
                      use_priors);

    return(matches);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereocam_broadcast");
    ros::NodeHandle n;

    // get the index number of the stereo camera
    ros::NodeHandle nh("~");
    nh.getParam("index", stereo_camera_index);
    char stereo_camera_index2[20];
    sprintf(stereo_camera_index2,"%d", stereo_camera_index);
    std::string stereo_camera_index_str = "";
    if (stereo_camera_index > -1) {
        stereo_camera_index_str += stereo_camera_index2;
        ROS_INFO("Stereo camera index: %s", stereo_camera_index_str.c_str());
    }

    image_transport::ImageTransport it(n);

    std::string topic_str = "stereo/left/image_raw" + stereo_camera_index_str;
    image_transport::Publisher left_pub = it.advertise(topic_str.c_str(), 1);

    topic_str = "stereo/right/image_raw" + stereo_camera_index_str;
    image_transport::Publisher right_pub = it.advertise(topic_str, 1);

    topic_str = "stereo/image_disparity" + stereo_camera_index_str;
    image_transport::Publisher disparity_pub = it.advertise(topic_str, 1);

    topic_str = "stereo/point_cloud" + stereo_camera_index_str;
    ros::Publisher point_cloud_pub = n.advertise<sensor_msgs::PointCloud>(topic_str, 1);
    ros::Rate loop_rate(30);

    IplImage *l=NULL;
    IplImage *r=NULL;
    unsigned char *l_=NULL;
    unsigned char *r_=NULL;

    // start service which can be used to start and stop the stereo camera
    std::string service_str = "camera_active" + stereo_camera_index_str;
    ros::ServiceServer service_active = n.advertiseService(service_str, camera_active);

    // start service which can be used to change camera parameters
    service_str = "stereocam_params" + stereo_camera_index_str;
    ros::ServiceServer service_params = n.advertiseService(service_str, request_params);

    // start service which can be used to change dense stereo parameters
    service_str = "densestereo_params" + stereo_camera_index_str;
    ros::ServiceServer service_densestereo = n.advertiseService(service_str, request_dense_stereo_params);

    // start service which can be used to change feature based stereo parameters
    service_str = "featurestereo_params" + stereo_camera_index_str;
    ros::ServiceServer service_featurestereo = n.advertiseService(service_str, request_feature_stereo_params);

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
    const int sub_pixel_multiplier = 16;
    int max_disparity_pixels = MAX_IMAGE_WIDTH * max_disparity_percent / 100;
    int disparity_space_length = (max_disparity_pixels / disparity_step) * MAX_IMAGE_WIDTH * ((MAX_IMAGE_HEIGHT/VERTICAL_SAMPLING)/smoothing_radius) * 2;
    int disparity_map_length = MAX_IMAGE_WIDTH * ((MAX_IMAGE_HEIGHT/VERTICAL_SAMPLING)/smoothing_radius) * 2;
    unsigned int* disparity_space = new unsigned int[disparity_space_length];
    unsigned int* disparity_map = new unsigned int[disparity_map_length];

    unsigned char* rectification_buffer = NULL;

    int no_matches = 0;
    bool restart_cameras = false;

    while (n.ok())
    {
        // request to turn camera on or off
        if ((cam_active_request != cam_active) || (restart_cameras)) {
            if ((cam_active_request == false) && (!restart_cameras)) {
                stop_cameras(c1,c2);
            }
            else {
                // create appropriately sized images
                if (l_ != NULL) {
                    cvReleaseImage(&l);
                    cvReleaseImage(&r);
                }
                l=cvCreateImage(cvSize(left_image.width, left_image.height), 8, 3);
                r=cvCreateImage(cvSize(right_image.width, right_image.height), 8, 3);

                l_=(unsigned char *)l->imageData;
                r_=(unsigned char *)r->imageData;

                if (lcam != NULL) {
                    delete lcam;
                    delete rcam;
                    lcam = NULL;
                    rcam = NULL;
                }

                // start the cameras
                if (c1 != NULL) {
                    delete c1;
                    delete c2;
                }

                c1 = new Camera(dev0.c_str(), left_image.width, left_image.height, fps);
                c2 = new Camera(dev1.c_str(), right_image.width, right_image.height, fps);

                cam_active = true;
                restart_cameras = false;
            }
        }

        if (cam_active) {

            // Read image data
            while ((c1->Get() == 0) || (c2->Get() == 0)) {
                usleep(100);
            }

            // Convert to IplImage
            c1->toIplImage(l);
            c2->toIplImage(r);

            // flip images
            if (flip_left_image) {
                if (rectification_buffer == NULL) {
                    rectification_buffer = new unsigned char[left_image.width * left_image.height * 3];
                }
                lcam->flip(l_, rectification_buffer);
            }
            if (flip_right_image) {
                if (rectification_buffer == NULL) {
                    rectification_buffer = new unsigned char[left_image.width * left_image.height * 3];
                }
                rcam->flip(r_, rectification_buffer);
            }

            if (publish_feature_matches) {
                // match features
                int matches = update_feature_matches(l_,r_,left_image.width,left_image.height, 400, max_disparity_percent);

                // convert features into a point cloud
                sensor_msgs::PointCloud point_cloud;
                feature_matches_to_PointCloud(
                    matches,
                    l_, left_image.width,left_image.height,
                    baseline_mm,
                    focal_length_pixels,
                    stereo_camera_index_str,
                    point_cloud);
                point_cloud_pub.publish(point_cloud);

                if (matches == 0) {
                    no_matches++;
                    if (no_matches > 3) {
                        restart_cameras = true;
                        no_matches = 0;
                    }
                }
            }

            if (publish_disparity_map) {
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

                // convert to stereo_msgs::Image format
                disparity_map_to_Image(
                    disparity_map,
                    disparity,
                    vertical_sampling,
                    smoothing_radius,
                    max_disparity_percent,
                    sub_pixel_multiplier,
                    baseline_mm,
                    focal_length_pixels);

                // create a point cloud from the disparity map
                float uncertainty_pixels = 1;
                sensor_msgs::PointCloud point_cloud;
                disparity_map_to_PointCloud(
                    disparity_map,
                    l_, left_image.width, left_image.height,
                    vertical_sampling,
                    smoothing_radius,
                    max_disparity_percent,
                    sub_pixel_multiplier,
                    baseline_mm,
                    focal_length_pixels,
                    uncertainty_pixels,
                    stereo_camera_index_str,
                    point_cloud);

                disparity_pub.publish(disparity);
                point_cloud_pub.publish(point_cloud);
            }

            // Convert to sensor_msgs::Image
            memcpy ((void*)(&left_image.data[0]), (void*)l_, left_image.width*left_image.height*3);
            memcpy ((void*)(&right_image.data[0]), (void*)r_, right_image.width*right_image.height*3);

            left_pub.publish(left_image);
            right_pub.publish(right_image);

            ROS_INFO("Stereo images published");
        }

        ros::spinOnce();
        int wait = cvWaitKey(10) & 255;
        if ( wait == 27 ) break;
    }

    if (l_ != NULL) {
        cvReleaseImage(&l);
        cvReleaseImage(&r);
    }

    if (lcam != NULL) {
        delete lcam;
        delete rcam;
    }
    if (rectification_buffer != NULL) delete[] rectification_buffer;
    if (disparity_space != NULL) delete[] disparity_space;
    if (disparity_map != NULL) delete[] disparity_map;

    stop_cameras(c1,c2);
    ROS_INFO("Exit Success");
}

