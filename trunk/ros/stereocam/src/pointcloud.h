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
#include <math.h>

using namespace std;

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

    static void virtual_camera(
        unsigned char * img,
        IplImage * points_image,
        CvMat * pose,
        CvMat * intrinsic_matrix,
        CvMat * distortion_coeffs,
        float max_range_mm,
        unsigned char * img_output);

    static void virtual_camera(
        unsigned char * img,
        IplImage * points_image,
        CvMat * pose,
        CvMat * intrinsic_matrix,
        CvMat * distortion_coeffs,
        float max_range_mm,
        float * &depth,
        CvMat * &rotation_matrix,
        CvMat * &translation,
        CvMat * &rotation_vector,
        CvMat * &points,
        CvMat * &image_points,
        unsigned char * img_output);

    static void obstacle_map(
        IplImage * points_image,
        int map_dimension,
        int map_cell_size_mm,
        CvMat * pose,
        float relative_x_mm,
        float relative_y_mm,
        int threshold,
        float tilt_degrees,
        int * map);

    static void fill(
        int id,
        int * map,
        int map_dimension,
        int x,
        int y,
        int depth,
        int &ctr,
        int threshold);

    static void find_objects(
        unsigned char * img,
        IplImage * points_image,
        int map_dimension,
        int map_cell_size_mm,
        CvMat * pose,
        float relative_x_mm,
        float relative_y_mm,
        int threshold,
        float tilt_degrees,
        int * map,
        int min_area_mm2,
        int max_area_mm2,
        std::vector<std::vector<float> > &objects);
};

#endif

