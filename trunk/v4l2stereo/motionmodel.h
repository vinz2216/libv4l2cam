/*
    motion model for smoothly moving camera
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

#ifndef MOTIONMODEL_H_
#define MOTIONMODEL_H_

#define MOTION_MODEL_SAMPLE_POSES  100
#define MOTION_MODEL_BUFFER_LENGTH 5

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "polynomial.h"
#include "linefit.h"
#include "stereo.h"

class motionmodel {
public:
	int no_of_rows, no_of_cols;

    /* array storing x coordinates of detected features */
    short int* feature_x;
    short int* prev_feature_x;

    /* array storing y coordinates of detected features */
    short int* feature_y;
    short int* prev_feature_y;

    /* array storing the number of features detected on each row */
    unsigned short int* features_per_row;
    unsigned short int* prev_features_per_row;

    /* array storing the number of features detected on each column */
    unsigned short int* features_per_col;
    unsigned short int* prev_features_per_col;

    /* total number of row and column features */
    int row_feats, col_feats;

    /* maximum variance for search */
    int max_variance_pan_degrees;
    int max_variance_tilt_degrees;
    int max_variance_roll_degrees;
    int max_variance_scale_percent;

    /* pose and score */
    int* poses;

    /* recent history of poses */
    int* pose_buffer;
    int pose_buffer_index;

    /* number of sample edges per hypothesis */
    int samples_per_hypothesis;

	int prior_pan_degrees;
	int prior_tilt_degrees;
	int prior_roll_degrees;
	int prior_scale_percent;

    void Update(short int* curr_feature_x, short int* curr_feature_y, unsigned short int* curr_features_per_row, unsigned short int* curr_features_per_col, int FOV_degrees, int rows, int cols, int image_width, int image_height);
    void Survey(int no_of_samples, int samples_per_hypothesis, int image_width, int image_height, int FOV_degrees, int rows, int cols);
    int TestHypothesis(int pan_degrees, int tilt_degrees, int roll_degrees, int scale_percent, int no_of_samples, int image_width, int image_height, int FOV_degrees, int rows, int cols);

	motionmodel();
	virtual ~motionmodel();
};

#endif
