/*
    stereo
    Functions for simple sparse stereo
    Copyright (C) 2009 Bob Mottram and Giacomo Spigler
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

#ifndef STEREO_H_
#define STEREO_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "polynomial.h"

#define SVS_MAX_FEATURES         2000
#define SVS_MAX_IMAGE_WIDTH      1024
#define SVS_MAX_IMAGE_HEIGHT     1024
#define SVS_VERTICAL_SAMPLING    2
#define SVS_DESCRIPTOR_PIXELS    30
#define SVS_MAX_LINES            200

#define pixindex(xx, yy)  ((yy * imgWidth + xx) * 3)

class svs {
public:
    unsigned int imgWidth, imgHeight;

    /* array storing x coordinates of detected features */
    short int* feature_x;

    /* array storing the number of features detected on each row */
    unsigned short int* features_per_row;

    /* Array storing a binary descriptor, 32bits in length, for each detected feature.
     * This will be used for matching purposes.*/
    unsigned int* descriptor;

    /* mean luminance for each feature */
    unsigned char* mean;

    /* buffer which stores sliding sum */
    int* row_sum;

    /* buffer used to find peaks in edge space */
    unsigned int* row_peaks;

    /* array stores matching probabilities (prob,x,y,disp) */
    unsigned int* svs_matches;

    /* used during filtering */
    unsigned char* valid_quadrants;

    /* array used to store a disparity histogram */
    int* disparity_histogram;

    /* maps raw image pixels to rectified pixels */
    int* calibration_map;

    int update_sums(int y, unsigned char* rectified_frame_buf);
    void non_max(int inhibition_radius, unsigned int min_response);
    int compute_descriptor(int px, int py, unsigned char* rectified_frame_buf, int no_of_features, int row_mean);
    int get_features(unsigned char* rectified_frame_buf, int inhibition_radius, unsigned int minimum_response, int calibration_offset_x, int calibration_offset_y);
    void filter(int no_of_possible_matches, int max_disparity_pixels, int tolerance);
    int match(svs* other, int ideal_no_of_matches, int max_disparity_percent, int descriptor_match_threshold, int learnDesc, int learnLuma, int learnDisp);

    void calibrate_offsets(unsigned char* left_image, unsigned char* right_image, int x_range, int y_range, int& calibration_offset_x, int& calibration_offset_y);
    void rectify(unsigned char* raw_image, float centre_of_distortion_x, float centre_of_distortion_y, float coeff_0, float coeff_1, float coeff_2, float rotation, float scale, unsigned char* rectified_frame_buf);

    void save_matches(std::string filename, unsigned char* rectified_frame_buf, int no_of_matches, bool colour);

    svs(int width, int height);
    ~svs();
};

#endif

