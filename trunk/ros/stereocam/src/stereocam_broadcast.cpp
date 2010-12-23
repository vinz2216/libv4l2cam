/*
    ROS driver to broadcast point cloud data from one or more stereo cameras
    Copyright (C) 2010 Bob Mottram
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
#include <sensor_msgs/PointCloud.h>
#include <sstream>
#include <iostream>
#include <stdio.h>

#include "stereocam/camera_active.h"

// enable flags for each stereo camera
bool enable[4];

bool load_point_cloud(
     char * point_cloud_filename,
     float * &translation,
     float * &rotation,
     int &image_width,
     int &image_height,
     float &baseline,
     sensor_msgs::PointCloud &point_cloud,
     ros::Time t)
{
    bool success = true;
    FILE * fp = fopen(point_cloud_filename,"rb");
    if (fp==NULL) return false;
    float * header = new float[11];
    size_t retval = fread(header, sizeof(float), 11, fp);
    if (retval != 11) return false;

    if (translation == NULL) translation = new float[3];
    if (rotation == NULL) rotation = new float[3];

    int no_of_points = 0, elem_bytes = 0;
    image_width = 0;
    image_height = 0;

    point_cloud.header.stamp = t;
    
    elem_bytes = (sizeof(float)*3) + 3;
    no_of_points = (int)header[1];
    image_width = (int)header[2];
    image_height = (int)header[3];
    translation[0] = header[4];
    translation[1] = header[5];
    translation[2] = header[6];
    rotation[0] = header[7];
    rotation[1] = header[8];
    rotation[2] = header[9];
    baseline = header[10];

    if (no_of_points > 0) {
        point_cloud.points.resize (no_of_points);
        point_cloud.channels.resize (3);
        point_cloud.channels[0].name = "r";
        point_cloud.channels[0].values.resize(no_of_points);
        point_cloud.channels[1].name = "g";
        point_cloud.channels[1].values.resize(no_of_points);
        point_cloud.channels[2].name = "b";
        point_cloud.channels[2].values.resize(no_of_points);

        ROS_INFO("Loading point cloud data");
        unsigned char * data_bytes = new unsigned char[elem_bytes * no_of_points];
        retval = fread(data_bytes,elem_bytes,no_of_points,fp);
        ROS_INFO("Point cloud data loaded");

        for (int i = 0; i < no_of_points; i++) {
            // set the point position
            float * data = (float*)&data_bytes[i*elem_bytes];
            point_cloud.points[i].x = data[0];
            point_cloud.points[i].y = data[1];
            point_cloud.points[i].z = data[2];

            // set the colour
            int n = (i*elem_bytes) + (3*sizeof(float));
            point_cloud.channels[0].values[i] = data_bytes[n];
            point_cloud.channels[1].values[i] = data_bytes[n+1];
            point_cloud.channels[2].values[i] = data_bytes[n+2];
        }
        delete [] data_bytes;
    }
    else {
        success = false;
    }

    delete [] header;
    return success;
}


/*!
 * \brief received a request to start or stop stereo cameras
 * \param req requested parameters
 * \param res returned parameters
 */
bool camera_active(
    stereocam::camera_active::Request &req,
    stereocam::camera_active::Response &res)
{
    enable[0] = req.camera1_active;
    enable[1] = req.camera2_active;
    enable[2] = req.camera3_active;
    enable[3] = req.camera4_active;

    return true;
}

bool file_exists(char * filename)
{
    FILE * fp = fopen(filename,"rb");
    if (fp != NULL) {
        fclose(fp);
        return true;
    }
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereocam_broadcast");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    bool en;
    int glimpsetime = 3;
    std::string output_directory = "";
    std::string s;
    std::string glimpse_command[4];

    for (int i = 0; i < 4; i++) enable[i] = true;

    nh.getParam("glimpsetime", glimpsetime);
    nh.getParam("outputdirectory", output_directory);
    nh.getParam("enable1", en);
    enable[0] = en;
    nh.getParam("enable2", en);
    enable[1] = en;
    nh.getParam("enable3", en);
    enable[2] = en;
    nh.getParam("enable4", en);
    enable[3] = en;
    nh.getParam("glimpsecommand1", s);
    glimpse_command[0] = s;
    nh.getParam("glimpsecommand2", s);
    glimpse_command[1] = s;
    nh.getParam("glimpsecommand3", s);
    glimpse_command[2] = s;
    nh.getParam("glimpsecommand4", s);
    glimpse_command[3] = s;

    // start service which can be used to start and stop the stereo camera
    std::string service_str = "camera_active";
    ros::ServiceServer service_active = n.advertiseService(service_str, camera_active);

    std::string topic_str = "stereo/point_cloud";
    ros::Publisher point_cloud_pub = n.advertise<sensor_msgs::PointCloud>(topic_str, 1);
    ros::Rate loop_rate(30);

    int retval, stereo_camera_index = 0;
    char str[256],filename[256],command_str[1000];

    while (n.ok())
    {
        if ((enable[stereo_camera_index]) &&
            (glimpse_command[stereo_camera_index] != "")) {
            // acquire point cloud data
            sprintf((char*)command_str,glimpse_command[stereo_camera_index].c_str(),output_directory.c_str(),stereo_camera_index+1);
            ROS_INFO("%s", command_str);
            ros::Time timestamp = ros::Time::now();
            retval = system(command_str);            

            ros::spinOnce();
            if (glimpsetime > 0) sleep(glimpsetime);

            sprintf((char*)filename,"%sglimpse%d.dat",output_directory.c_str(),stereo_camera_index+1);
            if (file_exists(filename)) {

                ros::spinOnce();
                float * translation = NULL;
                float * rotation = NULL;
                int image_width=0,image_height=0;
                float baseline=0;

                sensor_msgs::PointCloud point_cloud;
                sprintf((char*)str,"stereo_camera%d", stereo_camera_index);
                point_cloud.header.frame_id = str;

                if (load_point_cloud(
                    filename, translation, rotation,
                    image_width, image_height,
                    baseline, point_cloud, timestamp)) {

                    point_cloud_pub.publish(point_cloud);

                    ROS_INFO("Point cloud published from stereo camera %d", stereo_camera_index);
                }
                if (translation != NULL) delete [] translation;
                if (rotation != NULL) delete [] rotation;

                // delete point cloud data
                sprintf((char*)str,"rm %s",filename);
                retval = system(str);
            }
            else {
                ROS_INFO("File not found %s", filename);
            }
        }

        stereo_camera_index++;
        if (stereo_camera_index >= 4) stereo_camera_index = 0;
        ros::spinOnce();
    }

    ROS_INFO("Exit Success");
}

