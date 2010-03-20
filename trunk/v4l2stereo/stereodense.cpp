/*
    Dense stereo correspondence
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

#include "stereodense.h"

void stereodense::get_sums(
	unsigned char* img,
	int img_width,
	int img_height,
	int y,
	int* sums)
{
	int n = y*img_width*3;
	int stride = img_width*3;
	sums[0] = 0;
	for (int x = 1; x < img_width; x++, n += 3) {
		sums[x] = sums[x-1] + img[n] + img[n+1] + img[n+2];
		sums[x] += img[n-stride] + img[n+1-stride] + img[n+2-stride];
		sums[x] += img[n+stride] + img[n+1+stride] + img[n+2+stride];
	}
}

void stereodense::update_disparity_map(
	unsigned char* img_left,
	unsigned char* img_right,
	int img_width,
	int img_height,
	int offset_x,
	int offset_y,
	int vertical_sampling,
	int max_disparity_percent,
	int correlation_radius,
	int smoothing_radius,
	int disparity_step,
	int *disparity_space,
	int *disparity_map)
{
	const int smoothing_radius_vertical = 2;
	int correlation_radius_inner = correlation_radius/2;
	int max_disparity = max_disparity_percent * img_width / 100;
	int sums_left[img_width];
	int sums_right[img_width];

	int img_height2 = img_height / vertical_sampling;
	int width2 = img_width/smoothing_radius;
	int height2 = img_height2/smoothing_radius_vertical;

	int ty = 0;
	int by = img_height;
	if (offset_y >= 0)
		by = img_height - offset_y;
	else
		ty = -offset_y;

	int disparity_space_pixels = width2*(img_height2/smoothing_radius_vertical);

	for (int disparity = 0; disparity < max_disparity; disparity += disparity_step) {

		memset((void*)disparity_space,'\0',disparity_space_pixels*sizeof(int));

		// insert correlation values into the buffer
		int y2 = 0;
		for (int y = ty; y < by; y += vertical_sampling, y2++) {

			get_sums(img_left, img_width, img_height, y, sums_left);
			get_sums(img_right, img_width, img_height, y + offset_y, sums_right);

			int x_right = offset_x + disparity;
			for (int x_left = 0; x_left < img_width - offset_x; x_left++, x_right++) {

				if ((x_left - correlation_radius > -1) &&
					(x_left + correlation_radius < img_width) &&
					(x_right - correlation_radius > -1) &&
					(x_right + correlation_radius < img_width)) {

					int left_response =
						(sums_left[x_left + correlation_radius] - sums_left[x_left - correlation_radius]) -
						((sums_left[x_left + correlation_radius_inner] - sums_left[x_left - correlation_radius_inner])*4);

					int right_response =
						(sums_right[x_right + correlation_radius] - sums_right[x_right - correlation_radius]) -
						((sums_right[x_right + correlation_radius_inner] - sums_right[x_right - correlation_radius_inner])*4);

					int xx = x_left / smoothing_radius;
					int yy = y2 / smoothing_radius_vertical;
					if ((xx > 0) && (xx < width2-1) &&
						(yy > 0) && (yy < height2-1)) {
						int n = yy*width2 + xx;
						int v = 30000 - abs(left_response - right_response);
						disparity_space[n] += v;
						v /= 2;
						disparity_space[n-1] += v;
						disparity_space[n+1] += v;
						disparity_space[n-width2] += v;
						disparity_space[n-width2-1] += v;
						disparity_space[n-width2+1] += v;
						disparity_space[n+width2] += v;
						disparity_space[n+width2-1] += v;
						disparity_space[n+width2+1] += v;
					}
				}
			}
		}

		// update the disparity map
		for (int i = disparity_space_pixels-1; i >= 0; i--) {
			if ((disparity_space[i] > disparity_map[i*2]) || (disparity == 0)) {
				disparity_map[i*2] = disparity_space[i];
				disparity_map[i*2 + 1] = disparity;
			}
		}
	}
}

void stereodense::show(
	unsigned char* img,
	int img_width,
	int img_height,
	int vertical_sampling,
	int smoothing_radius,
	int max_disparity_percent,
	int *disparity_map)
{
	const int smoothing_radius_vertical = 2;
	int max_disparity = img_width * max_disparity_percent / 100;
	int width2 = img_width/smoothing_radius;

	for (int y = 0; y < img_height; y++) {
		for (int x = 0; x < img_width; x++) {
			int n = ((y*img_width) + x)*3;
			int n2 = ((((y/vertical_sampling)/smoothing_radius_vertical)*width2) + (x/smoothing_radius))*2;
            int disparity = disparity_map[n2 + 1] * 255  / max_disparity;
            img[n] = (unsigned char)disparity;
            img[n+1] = (unsigned char)disparity;
            img[n+2] = (unsigned char)disparity;
		}
	}
}

