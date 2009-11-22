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

motionmodel::motionmodel()
{
	bf = new bool*[3];
	bf_motion = new int*[3];
	for (int i = 0; i < 3; i++) {
		bf[i] = new bool[MOTIONMODEL_NO_OF_FEATURES];
		bf_motion[i] = new int[MOTIONMODEL_NO_OF_FEATURES];
	}
	bf_buffer = new bool[MOTIONMODEL_NO_OF_FEATURES];

	average_pan = 0;
	average_tilt = 0;
	average_roll = 0;
}

motionmodel::~motionmodel()
{
	delete[] bf_buffer;
	for (int i = 0; i < 3; i++) {
		delete[] bf[i];
		delete[] bf_motion[i];
	}
	delete[] bf;
	delete[] bf_motion;
}

int motionmodel::matchbuffer(bool* curr, bool *prev, int* motion, int window_size, int max_offset)
{
	int i,offset,matches,average_offset=0;
	int max_matches = 0;

	for (offset = -max_offset; offset < max_offset; offset++) {
		matches = 0;
		for (i = 0; i < MOTIONMODEL_NO_OF_FEATURES; i++) {
			if ((i+offset > -1) && (i+offset < MOTIONMODEL_NO_OF_FEATURES)) {
				if (curr[i+offset] == prev[i]) matches++;
			}
		}
		if (matches > max_matches) {
			max_matches = matches;
			average_offset = offset;
		}
	}

	for (i = 0; i < MOTIONMODEL_NO_OF_FEATURES; i++) {
		prev[i] = curr[i];
	}

	return(average_offset);
}

void motionmodel::update(unsigned char* img, int img_width, int img_height)
{
	int window_size = 10;
	int max_offset = 50;

	// horizontal
	int n,n0 = ((img_height/2) * img_width * 3)+2;
	for (int x = 0; x < MOTIONMODEL_NO_OF_FEATURES; x++) {
        n = n0 + (x * img_width * 3 / MOTIONMODEL_NO_OF_FEATURES);
        if (img[n+3]+img[n+4]+img[n+5] > img[n]+img[n+1]+img[n+2]) {
        	bf_buffer[x] = true;
        }
        else {
        	bf_buffer[x] = false;
        }
	}
	average_pan = (average_pan + matchbuffer(bf_buffer, bf[0], bf_motion[0], window_size, max_offset)) / 2;
    //printf("pan %d\n", average_pan);

	// vertical
	n0 = ((img_width/2) * 3) + 2;
	for (int y = 0; y < MOTIONMODEL_NO_OF_FEATURES; y++) {
        n = n0 + (y * img_height / MOTIONMODEL_NO_OF_FEATURES)*img_width*3;
        if (img[n+3]+img[n+4]+img[n+5] > img[n]+img[n+1]+img[n+2]) {
        	bf_buffer[y] = true;
        }
        else {
        	bf_buffer[y] = false;
        }
	}
	average_tilt = (average_tilt + matchbuffer(bf_buffer, bf[1], bf_motion[1], window_size, max_offset)) / 2;
	//printf("tilt %d\n", average_tilt);
}

void motionmodel::show(unsigned char* img, int img_width, int img_height)
{
	for (int x = 0; x < img_width; x++) {
		bool state = bf[0][x*MOTIONMODEL_NO_OF_FEATURES/img_width];
		for (int y = img_height/2; y < (img_height/2)+5; y++) {
            int n = ((y*img_width)+x)*3;
            if (state) {
            	img[n] = 255;
            	img[n+1] = 255;
            	img[n+2] = 255;
            }
            else {
            	img[n] = 0;
            	img[n+1] = 0;
            	img[n+2] = 0;
            }
		}
	}

	for (int y = 0; y < img_height; y++) {
		bool state = bf[1][y*MOTIONMODEL_NO_OF_FEATURES/img_height];
		for (int x = img_width/2; x < (img_width/2)+5; x++) {
            int n = ((y*img_width)+x)*3;
            if (state) {
            	img[n] = 255;
            	img[n+1] = 255;
            	img[n+2] = 255;
            }
            else {
            	img[n] = 0;
            	img[n+1] = 0;
            	img[n+2] = 0;
            }
		}
	}
}
