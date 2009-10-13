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

#include "motionmodel.h"

motionmodel::motionmodel() {
	no_of_rows = 0;
	no_of_cols = 0;
	row_feats=0;
	col_feats=0;

	max_variance_pan_degrees = 20;
    max_variance_tilt_degrees = 20;
    max_variance_roll_degrees = 0;
    max_variance_scale_percent = 10;

	prior_pan_degrees = 0;
	prior_tilt_degrees = 0;
	prior_roll_degrees = 0;
	prior_scale_percent = 100;

	poses = new int[MOTION_MODEL_SAMPLE_POSES*5];
	pose_buffer = new int[MOTION_MODEL_BUFFER_LENGTH*4];
	pose_buffer_index = 0;
    samples_per_hypothesis = 100;

	/* array storing x coordinates of detected features */
	feature_x = new short int[SVS_MAX_FEATURES];
	feature_y = new short int[SVS_MAX_FEATURES];
	prev_feature_x = new short int[SVS_MAX_FEATURES];
	prev_feature_y = new short int[SVS_MAX_FEATURES];

	/* array storing the number of features detected on each row */
	features_per_row = new unsigned short int[SVS_MAX_IMAGE_HEIGHT
			/ SVS_VERTICAL_SAMPLING];
	features_per_col = new unsigned short int[SVS_MAX_IMAGE_WIDTH
			/ SVS_HORIZONTAL_SAMPLING];
	prev_features_per_row = new unsigned short int[SVS_MAX_IMAGE_HEIGHT
			/ SVS_VERTICAL_SAMPLING];
	prev_features_per_col = new unsigned short int[SVS_MAX_IMAGE_WIDTH
			/ SVS_HORIZONTAL_SAMPLING];
}

motionmodel::~motionmodel() {
	delete[] feature_x;
	delete[] feature_y;
	delete[] features_per_row;
	delete[] features_per_col;
	delete[] prev_feature_x;
	delete[] prev_feature_y;
	delete[] prev_features_per_row;
	delete[] prev_features_per_col;
	delete[] poses;
	delete[] pose_buffer;
}

void motionmodel::Update(
	short int* curr_feature_x,
	short int* curr_feature_y,
	unsigned short int* curr_features_per_row,
	unsigned short int* curr_features_per_col,
	int FOV_degrees,
	int rows,
	int cols,
	int image_width,
	int image_height)
{
	/* copy current data into the previous buffers */
    int tot_row_feats = 0;
	for (int i = 0; i < no_of_rows; i++) {
		tot_row_feats += this->features_per_row[i];
	}
	memcpy((void*)prev_feature_x, (void*)this->feature_x, tot_row_feats * sizeof(short int));
	memcpy((void*)prev_features_per_row, (void*)this->features_per_row, no_of_rows * sizeof(unsigned short int));

	int tot_col_feats = 0;
	for (int i = 0; i < no_of_cols; i++) {
		tot_col_feats += this->features_per_col[i];
	}
	memcpy((void*)prev_feature_y, (void*)this->feature_y, tot_col_feats * sizeof(short int));
	memcpy((void*)prev_features_per_col, (void*)this->features_per_col, no_of_cols * sizeof(unsigned short int));

	row_feats = tot_row_feats;
	col_feats = tot_col_feats;

	/* update current features */
    tot_row_feats = 0;
	for (int i = 0; i < rows; i++) {
		tot_row_feats += curr_features_per_row[i];
	}
	memcpy((void*)feature_x, (void*)curr_feature_x, tot_row_feats * sizeof(short int));
	memcpy((void*)features_per_row, (void*)curr_features_per_row, rows * sizeof(unsigned short int));
    tot_col_feats = 0;
	for (int i = 0; i < cols; i++) {
		tot_col_feats += curr_features_per_col[i];
	}
	memcpy((void*)feature_y, (void*)curr_feature_y, tot_col_feats * sizeof(short int));
	memcpy((void*)features_per_col, (void*)curr_features_per_col, cols * sizeof(unsigned short int));

    if (no_of_rows > 0) Survey(MOTION_MODEL_SAMPLE_POSES, samples_per_hypothesis, image_width, image_height, FOV_degrees, rows, cols);

	no_of_rows = rows;
	no_of_cols = cols;

}

int motionmodel::TestHypothesis(
    int pan_pixels,
    int tilt_pixels,
    int roll_degrees,
    int scale_percent,
    int no_of_samples,
    int image_width,
    int image_height,
    int FOV_degrees,
    int rows,
    int cols)
{
	const int SinLookup[] = {
	0,174,348,523,697,871,1045,1218,1391,1564,1736,1908,2079,2249,2419,2588,2756,2923,3090,3255,3420,3583,3746,3907,4067,4226,4383,4539,4694,4848,5000,5150,5299,5446,5591,5735,5877,6018,6156,6293,6427,6560,6691,6819,6946,7071,            7193,7313,7431,7547,7660,7771,7880,7986,8090,8191,8290,8386,8480,8571,8660,8746,8829,8910,8987,9063,9135,9205,9271,9335,9396,9455,9510,9563,9612,9659,9702,9743,9781,9816,9848,9876,9902,9925,9945,9961,9975,9986,9993,9998,9999,            9998,9993,9986,9975,9961,9945,9925,9902,9876,9848,9816,9781,9743,9702,9659,9612,9563,9510,9455,9396,9335,9271,9205,9135,9063,8987,8910,8829,8746,8660,8571,8480,8386,8290,8191,8090,7986,7880,7771,7660,7547,7431,7313,7193,7071,           6946,6819,6691,6560,6427,6293,6156,6018,5877,5735,5591,5446,5299,5150,5000,4848,4694,4539,4383,4226,4067,3907,3746,3583,3420,3255,3090,2923,2756,2588,2419,2249,2079,1908,1736,1564,1391,1218,1045,871,697,523,348,174,0,            -174,-348,-523,-697,-871,-1045,-1218,-1391,-1564,-1736,-1908,-2079,-2249,-2419,-2588,-2756,-2923,-3090,-3255,-3420,-3583,-3746,-3907,-4067,-4226,-4383,-4539,-4694,-4848,-5000,-5150,-5299,-5446,-5591,-5735,-5877,-6018,-6156,-6293,-6427,-6560,-6691,-6819,-6946,-7071,            -7193,-7313,-7431,-7547,-7660,-7771,-7880,-7986,-8090,-8191,-8290,-8386,-8480,-8571,-8660,-8746,-8829,-8910,-8987,-9063,-9135,-9205,-9271,-9335,-9396,-9455,-9510,-9563,-9612,-9659,-9702,-9743,-9781,-9816,-9848,-9876,-9902,-9925,-9945,-9961,-9975,-9986,-9993,-9998,-9999,           -9998,-9993,-9986,-9975,-9961,-9945,-9925,-9902,-9876,-9848,-9816,-9781,-9743,-9702,-9659,-9612,-9563,-9510,-9455,-9396,-9335,-9271,-9205,-9135,-9063,-8987,-8910,-8829,-8746,-8660,-8571,-8480,-8386,-8290,-8191,-8090,-7986,-7880,-7771,-7660,-7547,-7431,-7313,-7193,-7071,            -6946,-6819,-6691,-6560,-6427,-6293,-6156,-6018,-5877,-5735,-5591,-5446,-5299,-5150,-4999,-4848,-4694,-4539,-4383,-4226,-4067,-3907,-3746,-3583,-3420,-3255,-3090,-2923,-2756,-2588,-2419,-2249,-2079,-1908,-1736,-1564,-1391,-1218,-1045,-871,-697,-523,-348,-174,
	};

	int score = 0;
	int j,min_dx, min_dy,r,f,idx,fx,fy,dx,dy,c;
	int dx_temp,dy_temp, expected_fx, expected_fy;
	int expected_row, expected_col;
	int cx = image_width/2;
	int cy = image_height/2;

    if (roll_degrees < 0) roll_degrees += 360;
    if (roll_degrees >= 360) roll_degrees -= 360;

    int SinVal = SinLookup[roll_degrees];
    int CosVal = 90 - roll_degrees;
    if (CosVal < 0) CosVal += 360;
    CosVal = SinLookup[CosVal];
    int tries = 0;
    int v, row, col;

    for (int i = 0; i < no_of_samples; i++) {

    	/* randomly select a feature */
    	tries = 0;
    	v = 0;
    	while ((tries < 3) && (v == 0))
    	{
            row = rand() % (no_of_rows-1);
            r=0;
            f=0;
            for (r = 0; r < row; r++) {
        	    f += (int)prev_features_per_row[r];
            }
            v = (int)prev_features_per_row[row]-1;
            tries++;
    	}
    	if (v > 0) {
			idx = f + (rand() % v);

			/* feature position in the previous image */
			fx = (int)prev_feature_x[idx];
			fy = 4 + (row * SVS_VERTICAL_SAMPLING);

			/* transform feature */
			dx = fx - cx;
			dy = fy - cy;

			if (scale_percent != 100) {
				dx = dx * scale_percent / 100;
				dy = dy * scale_percent / 100;
			}

			// rotate by the hypothetical orientation
			if (roll_degrees != 0) {
			    dx_temp = (dx*CosVal - dy*SinVal) / (int)10000;
			    dy_temp = (dx*SinVal + dy*CosVal) / (int)10000;
			    dx = dx_temp + pan_pixels;
			    dy = dy_temp + tilt_pixels;
			}
			else {
			    dx += pan_pixels;
			    dy += tilt_pixels;
			}

			/* expected position in the current image */
			expected_fx = cx + dx;
			expected_row = (((cy + dy)-4) / SVS_VERTICAL_SAMPLING);

			min_dx = image_width;
			if ((expected_row > -1) && (expected_row < rows)) {
				r=0;
				f=0;
				for (r = 0; r < expected_row; r++) {
					f += (int)features_per_row[r];
				}
				for (j = f; j < f + (int)features_per_row[expected_row]; j++) {
					dx = (int)feature_x[j] - expected_fx;
					if (dx < 0) dx = -dx;
					if (dx < min_dx) {
						min_dx = dx;
					}
				}
			}
			score += (image_width - min_dx);
    	}
    }

    for (int i = 0; i < no_of_samples; i++) {

    	// randomly select a feature
    	tries = 0;
    	v = 0;
    	while ((tries < 3) && (v == 0))
    	{
            col = rand() % (no_of_cols-1);
            c=0;
            f=0;
            for (c = 0; c < col; c++) {
        	    f += (int)prev_features_per_col[c];
            }
            v = (int)prev_features_per_col[col]-1;
            tries++;
    	}
    	if (v > 0) {
			idx = f + (rand() % v);

			// feature position in the previous image
			fx = 4 + (col * SVS_HORIZONTAL_SAMPLING);
			fy = (int)prev_feature_y[idx];

			// transform feature
			dx = fx - cx;
			dy = fy - cy;

			if (scale_percent != 100) {
				dx = dx * scale_percent / 100;
				dy = dy * scale_percent / 100;
			}

			// rotate by the hypothetical orientation
			if (roll_degrees != 0) {
			    dx_temp = (dx*CosVal - dy*SinVal) / (int)10000;
			    dy_temp = (dx*SinVal + dy*CosVal) / (int)10000;
			    dx = dx_temp + pan_pixels;
			    dy = dy_temp + tilt_pixels;
			}
			else {
			    dx += pan_pixels;
			    dy += tilt_pixels;
			}

			// expected position in the current image
			expected_fy = cy + dy;
			expected_col = (((cx + dx)-4) / SVS_HORIZONTAL_SAMPLING);

			min_dy = image_height;
			if ((expected_col > -1) && (expected_col < cols)) {
				c=0;
				f=0;
				for (c = 0; c < expected_col; c++) {
					f += (int)features_per_col[c];
				}
				for (j = f; j < f + (int)features_per_col[expected_col]; j++) {
					dy = (int)feature_y[j] - expected_fy;
					if (dy < 0) dy = -dy;
					if (dy < min_dy) {
						min_dy = dy;
					}
				}
			}
			score += (image_height - min_dy);
    	}
    }
    return(score);
}

void motionmodel::Survey(
	int no_of_samples,
	int samples_per_hypothesis,
	int image_width,
	int image_height,
	int FOV_degrees,
	int rows,
	int cols)
{
	int max_variance_pan_pixels = max_variance_pan_degrees * image_width / FOV_degrees;
	int max_variance_tilt_pixels = max_variance_tilt_degrees * image_width / FOV_degrees;

	int best_pose = -1;
	int max_score = 0;
    for (int s = 0; s < no_of_samples; s++) {
    	int pan_pixels = 0;
    	int tilt_pixels = 0;
    	int roll_degrees = 0;
    	int scale_percent = 100;

    	if (max_variance_pan_degrees > 0) pan_pixels = prior_pan_degrees + ((rand() % (max_variance_pan_pixels*2)) - max_variance_pan_pixels);
    	if (max_variance_tilt_degrees > 0) tilt_pixels = prior_tilt_degrees + ((rand() % (max_variance_tilt_pixels*2)) - max_variance_tilt_pixels);
    	if (max_variance_roll_degrees > 0) roll_degrees = prior_roll_degrees + ((rand() % (max_variance_roll_degrees*2)) - max_variance_roll_degrees);
    	if (max_variance_scale_percent > 0) scale_percent = prior_scale_percent + ((rand() % (max_variance_scale_percent*2)) - max_variance_scale_percent);

    	if (s == 0) {
    		pan_pixels = 0;
    		tilt_pixels = 0;
    		roll_degrees = 0;
    		scale_percent = 100;
    	}

    	poses[s*5] = pan_pixels;
    	poses[s*5+1] = tilt_pixels;
    	poses[s*5+2] = roll_degrees;
    	poses[s*5+3] = scale_percent;

    	int score = TestHypothesis(
    	    pan_pixels,
    	    tilt_pixels,
    	    roll_degrees,
    	    scale_percent,
    	    samples_per_hypothesis,
    	    image_width,
    	    image_height,
    	    FOV_degrees,
    	    rows, cols);

    	poses[s*5+4] = score;
    	if (score > 0) {
    		if (score > max_score) {
    			max_score = score;
    			best_pose = s;
    		}
    	}
    }

    int threshold = max_score * 99/100;
    int interpolated_pan_pixels = 0;
    int interpolated_tilt_pixels = 0;
    int interpolated_roll_degrees = 0;
    int interpolated_scale_percent = 0;
    int tot = 0;
    for (int s = 0; s < no_of_samples; s++) {
    	int score = poses[s*5+4];
    	if (score > threshold) {
    		//score *= score;
    		interpolated_pan_pixels += (poses[s*5] * image_width / FOV_degrees) * score;
    		interpolated_tilt_pixels += (poses[s*5+1] * image_width / FOV_degrees) * score;
    		interpolated_roll_degrees += poses[s*5+2] * score;
    		interpolated_scale_percent += poses[s*5+3] * score;
    		tot += score;
    	}
    }
    if (tot > 0) {
		interpolated_pan_pixels /= tot;
		interpolated_tilt_pixels /= tot;
		interpolated_roll_degrees /= tot;
		interpolated_scale_percent /= tot;

    	pose_buffer[pose_buffer_index*4] = interpolated_pan_pixels;
    	pose_buffer[pose_buffer_index*4+1] = interpolated_tilt_pixels;
    	pose_buffer[pose_buffer_index*4+2] = interpolated_roll_degrees;
    	pose_buffer[pose_buffer_index*4+3] = interpolated_scale_percent;
    	pose_buffer_index++;
    	if (pose_buffer_index == MOTION_MODEL_BUFFER_LENGTH) {
    		pose_buffer_index = 0;
    	}

    	interpolated_pan_pixels = 0;
    	interpolated_tilt_pixels = 0;
    	interpolated_roll_degrees = 0;
    	interpolated_scale_percent = 0;
    	for (int b = 0; b < MOTION_MODEL_BUFFER_LENGTH; b++) {
    		interpolated_pan_pixels += pose_buffer[b*4];
    		interpolated_tilt_pixels += pose_buffer[b*4+1];
    		interpolated_roll_degrees += pose_buffer[b*4+2];
    		interpolated_scale_percent += pose_buffer[b*4+3];
    	}
    	interpolated_pan_pixels /= MOTION_MODEL_BUFFER_LENGTH;
    	interpolated_tilt_pixels /= MOTION_MODEL_BUFFER_LENGTH;
    	interpolated_roll_degrees /= MOTION_MODEL_BUFFER_LENGTH;
    	interpolated_scale_percent /= MOTION_MODEL_BUFFER_LENGTH;

		printf("pan: %d  ", interpolated_pan_pixels);
    	printf("tilt: %d\n", interpolated_tilt_pixels);
    }

    if (best_pose > -1) {

    	//prior_pan_degrees += (poses[best_pose*5] - prior_pan_degrees)/2;
    	//prior_tilt_degrees += (poses[best_pose*5 + 1] - prior_tilt_degrees)/2;
    	//prior_roll_degrees += (poses[best_pose*5 + 2] - prior_roll_degrees)/2;
    	//prior_scale_percent += (poses[best_pose*5 + 3] - prior_scale_percent)/2;
    }
}
