/*
    Point cloud functions
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

#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include <cctype>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <omp.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

class pointcloud {
public:
    static void save(
        unsigned char * img_left,
        IplImage * points_image, 
        int max_range_mm,
        CvMat * pose,
        std::string point_cloud_filename);

    static void disparity_map_to_3d_points(
        float * disparity_map,
        int img_width,
        int img_height,
        CvMat * disparity_to_depth,
        CvMat * pose,
        IplImage * &disparity_image,
        IplImage * &points_image);

    static void show(
        IplImage * points_image,
        float * disparity_map,
        unsigned char * img_left,
        CvMat * pose,
        float max_range_mm,
        float max_height_mm,
        int view_type,
        int output_image_width,
        int output_image_height,
        unsigned char * img_output);

    static void view_from_pose(
        unsigned char * img,
        IplImage * points_image,
        CvMat * pose,
        CvMat * intrinsic_matrix,
        CvMat * distortion_coeffs,
        float max_range_mm,
        unsigned char * img_output);

};

#endif

