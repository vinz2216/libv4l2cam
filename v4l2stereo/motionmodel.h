/*
 motion model for smoothly moving camera
 Based upon code originally written by Justin Domke
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

#define MOTIONMODEL_NO_OF_FEATURES 300
#define MOTIONMODEL_STEP 10

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stereo.h"

class motionmodel {
private:
	int matchbuffer(bool* curr, bool *prev, int* motion, int window_size, int max_offset);
public:

	bool** bf;
	int** bf_motion;
	bool* bf_buffer;
	int average_pan, average_tilt, average_roll;

	void update(unsigned char* img, int img_width, int img_height);
	void show(unsigned char* img, int img_width, int img_height);

	motionmodel();
	virtual ~motionmodel();
};

#endif
