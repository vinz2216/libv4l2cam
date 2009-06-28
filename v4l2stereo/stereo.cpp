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

#include "stereo.h"

/* offsets of pixels to be compared within the patch region
 * arranged into a roughly rectangular structure */
const int pixel_offsets[] = { -2, -4, -1, -4, 1, -4, 2, -4, -5, -2, -4, -2, -3,
		-2, -2, -2, -1, -2, 0, -2, 1, -2, 2, -2, 3, -2, 4, -2, 5, -2, -5, 2,
		-4, 2, -3, 2, -2, 2, -1, 2, 0, 2, 1, 2, 2, 2, 3, 2, 4, 2, 5, 2, -2, 4,
		-1, 4, 1, 4, 2, 4 };

/* lookup table used for counting the number of set bits */
const unsigned char BitsSetTable256[] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3,
		2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3,
		2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
		4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4,
		3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
		4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 1, 2, 2, 3,
		2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
		4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5,
		4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5,
		4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 3, 4, 4, 5,
		4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7,
		6, 7, 7, 8 };

svs::svs(int width, int height) {

	imgWidth = width;
	imgHeight = height;

	/* array storing x coordinates of detected features */
	feature_x = new short int[SVS_MAX_FEATURES];
	feature_y = new short int[SVS_MAX_FEATURES];

	disparity_histogram = NULL;
	calibration_map = NULL;

	/* array storing the number of features detected on each row */
	features_per_row = new unsigned short int[SVS_MAX_IMAGE_HEIGHT
			/ SVS_VERTICAL_SAMPLING];
	features_per_col = new unsigned short int[SVS_MAX_IMAGE_WIDTH
			/ SVS_VERTICAL_SAMPLING];

	/* Array storing a binary descriptor, 32bits in length, for each detected feature.
	 * This will be used for matching purposes.*/
	descriptor = new unsigned int[SVS_MAX_FEATURES];

	/* mean luminance for each feature */
	mean = new unsigned char[SVS_MAX_FEATURES];

	/* buffer which stores sliding sum */
	row_sum = new int[SVS_MAX_IMAGE_WIDTH];

	/* buffer used to find peaks in edge space */
	row_peaks = new unsigned int[SVS_MAX_IMAGE_WIDTH];

	/* array stores matching probabilities (prob,x,y,disp) */
	svs_matches = NULL;

	/* array used during filtering */
	valid_quadrants = NULL;

	/* priors */
	disparity_priors = NULL;
}

svs::~svs() {
	delete[] feature_x;
	delete[] features_per_row;
	delete[] descriptor;
	delete[] mean;
	delete[] row_sum;
	delete[] row_peaks;
	if (svs_matches != NULL)
		delete[] svs_matches;
	if (valid_quadrants != NULL)
		delete[] valid_quadrants;
	if (disparity_priors != NULL)
		delete[] disparity_priors;
	if (disparity_histogram != NULL)
		delete[] disparity_histogram;
	if (calibration_map != NULL)
		delete[] calibration_map;
}

/* Updates sliding sums and edge response values along a single row
 * Returns the mean luminance along the row */
int svs::update_sums(int y, /* row index */
unsigned char* rectified_frame_buf) { /* image data */

	int x, idx, mean = 0;
	unsigned int v;

	/* compute sums along the row */
	int stride = pixindex(imgWidth, 0);
	idx = stride * y;
	row_sum[0] = rectified_frame_buf[idx + 2] + rectified_frame_buf[idx + 1]
			+ rectified_frame_buf[idx + 0];
	for (x = 1; x < (int) imgWidth; x++) {
		idx = pixindex(x, y);
		v = rectified_frame_buf[idx + 2] + rectified_frame_buf[idx + 1]
				+ rectified_frame_buf[idx];
		row_sum[x] = row_sum[x - 1] + v;
	}

	/* row mean luminance */
	mean = row_sum[x - 1] / ((int) imgWidth * 3);

	/* compute peaks */
	int p0, p1;
	for (x = 4; x < (int) imgWidth - 4; x++) {

		/* edge using 2 pixel radius */
		p0 = (row_sum[x] - row_sum[x - 2]) - (row_sum[x + 2] - row_sum[x]);
		if (p0 < 0)
			p0 = -p0;

		/* edge using 4 pixel radius */
		p1 = (row_sum[x] - row_sum[x - 4]) - (row_sum[x + 4] - row_sum[x]);
		if (p1 < 0)
			p1 = -p1;

		/* overall edge response */
		row_peaks[x] = p0 + p1;
	}

	return (mean);
}

/* Updates sliding sums and edge response values along a single column
 * Returns the mean luminance along the column */
int svs::update_sums_vertical(int x, /* col index */
unsigned char* rectified_frame_buf) { /* image data */

	int y, idx, mean = 0;
	unsigned int v;

	/* compute sums along the column */
	idx = x;
	row_sum[0] =
		rectified_frame_buf[idx + 2] +
		rectified_frame_buf[idx + 1] +
		rectified_frame_buf[idx + 0];
	for (y = 1; y < (int) imgHeight; y++) {
		idx = pixindex(x, y);
		v =
		    rectified_frame_buf[idx + 2] +
		    rectified_frame_buf[idx + 1] +
			rectified_frame_buf[idx];
		row_sum[y] = row_sum[y - 1] + v;
	}

	/* column mean luminance */
	mean = row_sum[y - 1] / ((int) imgHeight * 3);

	/* compute peaks */
	int p0, p1;
	for (y = 4; y < (int) imgHeight - 4; y++) {

		/* edge using 2 pixel radius */
		p0 = (row_sum[y] - row_sum[y - 2]) - (row_sum[y + 2] - row_sum[y]);
		if (p0 < 0)
			p0 = -p0;

		/* edge using 4 pixel radius */
		p1 = (row_sum[y] - row_sum[y - 4]) - (row_sum[y + 4] - row_sum[y]);
		if (p1 < 0)
			p1 = -p1;

		/* overall edge response */
		row_peaks[y] = p0 + p1;
	}

	return (mean);
}

/* performs non-maximal suppression on the given row */
void svs::non_max(int inhibition_radius, /* radius for non-maximal suppression */
unsigned int min_response) { /* minimum threshold as a percent in the range 0-200 */

	int x, r;
	unsigned int v;

	/* average response */
	unsigned int av_peaks = 0;
	for (x = 4; x < (int) imgWidth - 4; x++) {
		av_peaks += row_peaks[x];
	}
	av_peaks /= (imgWidth - 8);

	/* adjust the threshold */
	av_peaks = av_peaks * min_response / 100;

	for (x = 4; x < (int) imgWidth - inhibition_radius; x++) {

		if (row_peaks[x] < av_peaks)
			row_peaks[x] = 0;
		v = row_peaks[x];
		if (v > 0) {
			for (r = 1; r < inhibition_radius; r++) {
				if (row_peaks[x + r] < v) {
					row_peaks[x + r] = 0;
				} else {
					row_peaks[x] = 0;
					r = inhibition_radius;
				}
			}
		}
	}
}

/* performs non-maximal suppression on the given row */
void svs::non_max_vertical(int inhibition_radius, /* radius for non-maximal suppression */
unsigned int min_response) { /* minimum threshold as a percent in the range 0-200 */

	int y, r;
	unsigned int v;

	/* average response */
	unsigned int av_peaks = 0;
	for (y = 4; y < (int) imgHeight - 4; y++) {
		av_peaks += row_peaks[y];
	}
	av_peaks /= (imgHeight - 8);

	/* adjust the threshold */
	av_peaks = av_peaks * min_response / 100;

	for (y = 4; y < (int) imgHeight - inhibition_radius; y++) {

		if (row_peaks[y] < av_peaks)
			row_peaks[y] = 0;
		v = row_peaks[y];
		if (v > 0) {
			for (r = 1; r < inhibition_radius; r++) {
				if (row_peaks[y + r] < v) {
					row_peaks[y + r] = 0;
				} else {
					row_peaks[y] = 0;
					r = inhibition_radius;
				}
			}
		}
	}
}

/* creates a binary descriptor for a feature at the given coordinate
 which can subsequently be used for matching */
int svs::compute_descriptor(int px, int py, unsigned char* rectified_frame_buf,
		int no_of_features, int row_mean) {

	unsigned char bit_count = 0;
	int pixel_offset_idx, ix, bit;
	int meanval = 0;
	unsigned int desc = 0;

	/* find the mean luminance for the patch */
	for (pixel_offset_idx = 0; pixel_offset_idx < SVS_DESCRIPTOR_PIXELS * 2; pixel_offset_idx
			+= 2) {

		ix
				= rectified_frame_buf[pixindex((px + pixel_offsets[pixel_offset_idx]), (py + pixel_offsets[pixel_offset_idx + 1]))];
		meanval += rectified_frame_buf[ix + 2] + rectified_frame_buf[ix + 1]
				+ rectified_frame_buf[ix];
	}
	meanval /= SVS_DESCRIPTOR_PIXELS;

	/* binarise */
	bit = 1;
	for (pixel_offset_idx = 0; pixel_offset_idx < SVS_DESCRIPTOR_PIXELS * 2; pixel_offset_idx
			+= 2, bit *= 2) {

		ix
				= rectified_frame_buf[pixindex((px + pixel_offsets[pixel_offset_idx]), (py + pixel_offsets[pixel_offset_idx + 1]))];
		if (rectified_frame_buf[ix + 2] + rectified_frame_buf[ix + 1]
				+ rectified_frame_buf[ix] > meanval) {
			desc |= bit;
			bit_count++;
		}
	}

	if ((bit_count > 3) && (bit_count < SVS_DESCRIPTOR_PIXELS - 3)) {
		meanval /= 3;

		/* adjust the patch luminance relative to the mean
		 * luminance for the entire row.  This helps to ensure
		 * that comparisons between left and right images are
		 * fair even if there are exposure/illumination differences. */
		meanval = meanval - row_mean + 127;
		if (meanval < 0)
			meanval = 0;
		if (meanval > 255)
			meanval = 255;

		mean[no_of_features] = (unsigned char) (meanval / 3);
		descriptor[no_of_features] = desc;
		return (0);
	} else {
		/* probably just noise */
		return (-1);
	}
}

/* returns a set of features suitable for stereo matching */
int svs::get_features(unsigned char* rectified_frame_buf, /* image data */
int inhibition_radius, /* radius for non-maximal supression */
unsigned int minimum_response, /* minimum threshold */
int calibration_offset_x, int calibration_offset_y) {

	unsigned short int no_of_feats;
	int x, y, row_mean, start_x;
	int no_of_features = 0;
	int row_idx = 0;

	memset(features_per_row, 0, SVS_MAX_IMAGE_HEIGHT / SVS_VERTICAL_SAMPLING
			* sizeof(unsigned short));

	start_x = imgWidth - 15;
	if ((int) imgWidth - inhibition_radius - 1 < start_x)
		start_x = (int) imgWidth - inhibition_radius - 1;

	for (y = 4 + calibration_offset_y; y < (int) imgHeight - 4; y
			+= SVS_VERTICAL_SAMPLING) {

		/* reset number of features on the row */
		no_of_feats = 0;

		if ((y >= 4) && (y <= (int) imgHeight - 4)) {

			row_mean = update_sums(y, rectified_frame_buf);
			non_max(inhibition_radius, minimum_response);

			/* store the features */
			for (x = start_x; x > 15; x--) {
				if (row_peaks[x] > 0) {

					if (compute_descriptor(x, y, rectified_frame_buf,
							no_of_features, row_mean) == 0) {
						feature_x[no_of_features++] = (short int) (x
								+ calibration_offset_x);
						no_of_feats++;
						if (no_of_features == SVS_MAX_FEATURES) {
							y = imgHeight;
							printf("stereo feature buffer full\n");
							break;
						}
					}
				}
			}
		}

		features_per_row[row_idx++] = no_of_feats;
	}
	return (no_of_features);
}

/* returns a set of features suitable for stereo matching */
int svs::get_features_vertical(unsigned char* rectified_frame_buf, /* image data */
int inhibition_radius, /* radius for non-maximal supression */
unsigned int minimum_response, /* minimum threshold */
int calibration_offset_x, int calibration_offset_y) {

	unsigned short int no_of_feats;
	int x, y, row_mean, start_y;
	int no_of_features = 0;
	int col_idx = 0;

	memset(features_per_col, 0, SVS_MAX_IMAGE_WIDTH / SVS_VERTICAL_SAMPLING
			* sizeof(unsigned short));

	start_y = imgHeight - 15;
	if ((int) imgHeight - inhibition_radius - 1 < start_y)
		start_y = (int) imgHeight - inhibition_radius - 1;

	for (x = 4 + calibration_offset_x; x < (int) imgWidth - 4; x
			+= SVS_VERTICAL_SAMPLING) {

		/* reset number of features on the row */
		no_of_feats = 0;

		if ((x >= 4) && (x <= (int) imgWidth - 4)) {

			row_mean = update_sums_vertical(x, rectified_frame_buf);
			non_max_vertical(inhibition_radius, minimum_response);

			/* store the features */
			for (y = start_y; y > 15; y--) {
				if (row_peaks[y] > 0) {

					if (compute_descriptor(x, y, rectified_frame_buf,
							no_of_features, row_mean) == 0) {
						feature_y[no_of_features++] = (short int) (y
								+ calibration_offset_y);
						no_of_feats++;
						if (no_of_features == SVS_MAX_FEATURES) {
							x = imgWidth;
							printf("stereo feature buffer full\n");
							break;
						}
					}
				}
			}
		}

		features_per_col[col_idx++] = no_of_feats;
	}
	return (no_of_features);
}

/* Match features from this camera with features from the opposite one.
 * It is assumed that matching is performed on the left camera CPU */
int svs::match(svs* other, int ideal_no_of_matches, /* ideal number of matches to be returned */
int max_disparity_percent, /* max disparity as a percent of image width */
int descriptor_match_threshold, /* minimum no of descriptor bits to be matched, in the range 1 - SVS_DESCRIPTOR_PIXELS */
int learnDesc, /* descriptor match weight */
int learnLuma, /* luminance match weight */
int learnDisp, /* disparity weight */
int use_priors) { /* if non-zero then use priors, assuming time between frames is small */

	int x, xL=0, xR, L, R, y, no_of_feats, no_of_feats_left, no_of_feats_right, row, col=0, bit;
	int luma_diff, disp_prior, min_disp, max_disp = 0, max_disp_pixels, meanL,
			meanR, disp=0, fL = 0, fR = 0, bestR = 0;
	unsigned int descL, descLanti, descR, desc_match;
	unsigned int correlation, anticorrelation, total, n;
	unsigned int match_prob, best_prob;
	int idx, max, curr_idx=0, search_idx, winner_idx = 0;
	int no_of_possible_matches = 0, matches = 0;
	int itt, prev_matches;

	unsigned int meandescL, meandescR;
	short meandesc[SVS_DESCRIPTOR_PIXELS];

	/* create arrays */
	if (svs_matches == NULL) {
		svs_matches = new unsigned int[SVS_MAX_MATCHES * 4];
		valid_quadrants = new unsigned char[SVS_MAX_MATCHES];
	}

	/* create array to store disparity priors */
	if ((use_priors != 0) && (disparity_priors == NULL)) {
		disparity_priors = new int[SVS_MAX_IMAGE_WIDTH * SVS_MAX_IMAGE_HEIGHT
				/ (16*SVS_VERTICAL_SAMPLING)];
	}

	/* convert max disparity from percent to pixels */
	max_disp_pixels = max_disparity_percent * imgWidth / 100;
	min_disp = -10;
	max_disp = max_disp_pixels;

	row = 0;
	for (y = 4; y < (int) imgHeight - 4; y += SVS_VERTICAL_SAMPLING, row++) {

		/* number of features on left and right rows */
		no_of_feats_left = features_per_row[row];
		no_of_feats_right = other->features_per_row[row];

		/* compute mean descriptor for the left row
		 * this will be used to create eigendescriptors */
		meandescL = 0;
		memset(meandesc, 0, (SVS_DESCRIPTOR_PIXELS) * sizeof(short));
		for (L = 0; L < no_of_feats_left; L++) {
			descL = descriptor[fL + L];
			n = 1;
			for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2) {
				if (descL & n)
					meandesc[bit]++;
				else
					meandesc[bit]--;
			}
		}
		n = 1;
		for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2) {
			if (meandesc[bit] >= 0)
				meandescL |= n;
		}

		/* compute mean descriptor for the right row
		 * this will be used to create eigendescriptors */
		meandescR = 0;
		memset(meandesc, 0, (SVS_DESCRIPTOR_PIXELS) * sizeof(short));
		for (R = 0; R < no_of_feats_right; R++) {
			descR = other->descriptor[fR + R];
			n = 1;
			for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2) {
				if (descR & n)
					meandesc[bit]++;
				else
					meandesc[bit]--;
			}
		}
		n = 1;
		for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2) {
			if (meandesc[bit] > 0)
				meandescR |= n;
		}

		/* features along the row in the left camera */
		for (L = 0; L < no_of_feats_left; L++) {

			/* x coordinate of the feature in the left camera */
			xL = feature_x[fL + L];

			if (use_priors != 0) {
				disp_prior = disparity_priors[(row * imgWidth + xL) / 16];

				if (disp_prior == 0) {
					/* no prior available - use default search */
					min_disp = -10;
					max_disp = max_disp_pixels;
				} else {
					/* narrow the range of search based upon prior */
					min_disp = disp_prior - 3;
					max_disp = disp_prior + 3;
				}
			}

			/* mean luminance and eigendescriptor for the left camera feature */
			meanL = mean[fL + L];
			descL = descriptor[fL + L] & meandescL;

			/* invert bits of the descriptor for anti-correlation matching */
			n = descL;
			descLanti = 0;
			for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++) {
				/* Shift result vector to higher significance. */
				descLanti <<= 1;
				/* Get least significant input bit. */
				descLanti |= n & 1;
				/* Shift input vector to lower significance. */
				n >>= 1;
			}

			total = 0;

			/* features along the row in the right camera */
			for (R = 0; R < no_of_feats_right; R++) {

				/* set matching score to zero */
				row_peaks[R] = 0;

				/* x coordinate of the feature in the right camera */
				xR = other->feature_x[fR + R];

				/* compute disparity */
				disp = xL - xR;

				/* is the disparity within range? */
				if ((disp >= min_disp) && (disp < max_disp)) {
					if (disp < 0)
						disp = 0;

					/* mean luminance for the right camera feature */
					meanR = other->mean[fR + R];

					/* is the mean luminance similar? */
					luma_diff = meanR - meanL;

					/* right camera feature eigendescriptor */
					descR = other->descriptor[fR + R] & meandescR;

					/* bitwise descriptor correlation match */
					desc_match = descL & descR;

					/* count the number of correlation bits */
					correlation = BitsSetTable256[desc_match & 0xff]
							+ BitsSetTable256[(desc_match >> 8) & 0xff]
							+ BitsSetTable256[(desc_match >> 16) & 0xff]
							+ BitsSetTable256[desc_match >> 24];

					/* were enough bits matched ? */
					if ((int) correlation > descriptor_match_threshold) {

						/* bitwise descriptor anti-correlation match */
						desc_match = descLanti & descR;

						/* count the number of anti-correlation bits */
						anticorrelation = BitsSetTable256[desc_match & 0xff]
								+ BitsSetTable256[(desc_match >> 8) & 0xff]
								+ BitsSetTable256[(desc_match >> 16) & 0xff]
								+ BitsSetTable256[desc_match >> 24];

						if (luma_diff < 0)
							luma_diff = -luma_diff;
						int score =
								10000 + (max_disp * learnDisp)
										+ (((int) correlation
												+ (int) (SVS_DESCRIPTOR_PIXELS
														- anticorrelation))
												* learnDesc) - (luma_diff
										* learnLuma) - (disp * learnDisp);
						if (score < 0)
							score = 0;

						/* store overall matching score */
						row_peaks[R] = (unsigned int) score;
						total += row_peaks[R];
					}
				} else {
					if ((disp < min_disp) && (disp > -max_disp)) {
						row_peaks[R] = (unsigned int) ((max_disp - disp)
								* learnDisp);
						total += row_peaks[R];
					}
				}
			}

			/* non-zero total matching score */
			if (total > 0) {

				/* convert matching scores to probabilities */
				best_prob = 0;
				for (R = 0; R < no_of_feats_right; R++) {
					if (row_peaks[R] > 0) {
						match_prob = row_peaks[R] * 1000 / total;
						if (match_prob > best_prob) {
							best_prob = match_prob;
							bestR = R;
						}
					}
				}

				if ((best_prob > 0) && (best_prob < 1000)
						&& (no_of_possible_matches < SVS_MAX_FEATURES)) {

					/* x coordinate of the feature in the right camera */
					xR = other->feature_x[fR + bestR];

					/* possible disparity */
					disp = xL - xR;

					if (disp >= -10) {
						if (disp < 0)
							disp = 0;
						/* add the best result to the list of possible matches */
						svs_matches[no_of_possible_matches * 4] = best_prob;
						svs_matches[no_of_possible_matches * 4 + 1]
								= (unsigned int) xL;
						svs_matches[no_of_possible_matches * 4 + 2]
								= (unsigned int) y;
						svs_matches[no_of_possible_matches * 4 + 3]
								= (unsigned int) disp;
						no_of_possible_matches++;
					}
				}
			}
		}

		/* increment feature indexes */
		fL += no_of_feats_left;
		fR += no_of_feats_right;
	}

	// clear priors
	memset(disparity_priors, 0, imgWidth * imgHeight / (16*
			SVS_VERTICAL_SAMPLING ) * sizeof(int));

	if (no_of_possible_matches > 1) {

		/* filter the results */
		filter(no_of_possible_matches, max_disp, 3);

		/* sort matches in descending order of probability */
		if (no_of_possible_matches < ideal_no_of_matches) {
			ideal_no_of_matches = no_of_possible_matches;
		}
		curr_idx = 0;
		search_idx = 0;
		for (matches = 0; matches < ideal_no_of_matches; matches++, curr_idx
				+= 4) {

			match_prob = svs_matches[curr_idx];
			winner_idx = -1;

			search_idx = curr_idx + 4;
			max = no_of_possible_matches * 4;
			while (search_idx < max) {
				if (svs_matches[search_idx] > match_prob) {
					match_prob = svs_matches[search_idx];
					winner_idx = search_idx;
				}
				search_idx += 4;
			}
			if (winner_idx > -1) {

				/* swap */
				best_prob = svs_matches[winner_idx];
				xL = svs_matches[winner_idx + 1];
				y = svs_matches[winner_idx + 2];
				disp = svs_matches[winner_idx + 3];

				svs_matches[winner_idx] = svs_matches[curr_idx];
				svs_matches[winner_idx + 1] = svs_matches[curr_idx + 1];
				svs_matches[winner_idx + 2] = svs_matches[curr_idx + 2];
				svs_matches[winner_idx + 3] = svs_matches[curr_idx + 3];

				svs_matches[curr_idx] = best_prob;
				svs_matches[curr_idx + 1] = xL;
				svs_matches[curr_idx + 2] = y;
				svs_matches[curr_idx + 3] = disp;

				/* store prior */
				row = y / SVS_VERTICAL_SAMPLING;
			    disparity_priors[(row * imgWidth + xL) / 16] = disp;
			    if (row > 2) {
			    	idx = ((row-1) * imgWidth + xL) / 16;
			    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    	if (xL > 6) {
				    	idx = ((row-1) * imgWidth + xL - 4) / 16;
				    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
				    	idx = ((row-1) * imgWidth + xL - 8) / 16;
				    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    	}
			    	if (xL < (int)imgWidth-9) {
			    	    idx = ((row-1) * imgWidth + xL + 4) / 16;
			    	    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    	    idx = ((row-1) * imgWidth + xL + 8) / 16;
			    	    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    	}
			    	idx = ((row-2) * imgWidth + xL) / 16;
			    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    	idx = ((row-3) * imgWidth + xL) / 16;
			    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    }
			    idx = ((row+1) * imgWidth + xL) / 16;
			    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    idx = ((row+2) * imgWidth + xL) / 16;
			    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    idx = ((row+3) * imgWidth + xL) / 16;
			    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
		    	if (xL > 6) {
			    	idx = ((row+1) * imgWidth + xL - 4) / 16;
			    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
			    	idx = ((row+1) * imgWidth + xL - 8) / 16;
			    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
		    	}
		    	if (xL < (int)imgWidth-9) {
		    	    idx = ((row+1) * imgWidth + xL + 4) / 16;
		    	    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
		    	    idx = ((row+1) * imgWidth + xL + 8) / 16;
		    	    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
		    	}
			}

			if (svs_matches[curr_idx] == 0) {
				break;
			}

		}
	}

	/* attempt to assign disparities to vertical features */
	memset(valid_quadrants, 0, SVS_MAX_MATCHES * sizeof(unsigned char));
	itt = 0;
	prev_matches = matches;
	for (itt = 0; itt < 10; itt++) {
		fL = 0;
		col = 0;
		for (x = 4; x < (int) imgWidth - 4; x += SVS_VERTICAL_SAMPLING, col++) {

			no_of_feats = features_per_col[col];

			/* features along the row in the left camera */
			for (L = 0; L < no_of_feats; L++) {

				if (valid_quadrants[fL + L] == 0) {
					/* y coordinate of the feature in the left camera */
					y = feature_y[fL + L];

					/* lookup disparity from priors */

					row = y / SVS_VERTICAL_SAMPLING;
					disp_prior = disparity_priors[(row * imgWidth + x) / 16];

					if ((disp_prior > 0) &&
						(matches < SVS_MAX_MATCHES)) {
						curr_idx = matches * 4;
						svs_matches[curr_idx] = 1000;
						svs_matches[curr_idx + 1] = x;
						svs_matches[curr_idx + 2] = y;
						svs_matches[curr_idx + 3] = disp_prior;
						matches++;
						if (row > 2) {
							idx = ((row-1) * imgWidth + x) / 16;
							if (disparity_priors[idx] == 0) disparity_priors[idx] = disp_prior;
							idx = ((row-2) * imgWidth + x) / 16;
							if (disparity_priors[idx] == 0) disparity_priors[idx] = disp_prior;
							idx = ((row-3) * imgWidth + x) / 16;
							if (disparity_priors[idx] == 0) disparity_priors[idx] = disp_prior;
					    	if (xL > 6) {
						    	idx = ((row-1) * imgWidth + xL - 4) / 16;
						    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
						    	idx = ((row-1) * imgWidth + xL - 8) / 16;
						    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
					    	}
					    	if (xL < (int)imgWidth-9) {
					    	    idx = ((row-1) * imgWidth + xL + 4) / 16;
					    	    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
					    	    idx = ((row-1) * imgWidth + xL + 8) / 16;
					    	    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
					    	}
						}
						idx = ((row+1) * imgWidth + x) / 16;
						if (disparity_priors[idx] == 0) disparity_priors[idx] = disp_prior;
						idx = ((row+2) * imgWidth + x) / 16;
						if (disparity_priors[idx] == 0) disparity_priors[idx] = disp_prior;
						idx = ((row+3) * imgWidth + x) / 16;
						if (disparity_priors[idx] == 0) disparity_priors[idx] = disp_prior;
				    	if (xL > 6) {
					    	idx = ((row+1) * imgWidth + xL - 4) / 16;
					    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
					    	idx = ((row+1) * imgWidth + xL - 8) / 16;
					    	if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
				    	}
				    	if (xL < (int)imgWidth-9) {
				    	    idx = ((row+1) * imgWidth + xL + 4) / 16;
				    	    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
				    	    idx = ((row+1) * imgWidth + xL + 8) / 16;
				    	    if (disparity_priors[idx] == 0) disparity_priors[idx] = disp;
				    	}
						valid_quadrants[fL + L] = 1;
					}
				}
			}
			fL += no_of_feats;
		}
	    if (prev_matches == matches) break;
	    prev_matches = matches;
	}
	return (matches);
}

/* filtering function removes noise by searching for a peak in the disparity histogram */
void svs::filter(int no_of_possible_matches, /* the number of stereo matches */
int max_disparity_pixels, /*maximum disparity in pixels */
int tolerance) { /* tolerance around the peak in pixels of disparity */

	int i, hf;
	unsigned int tx = 0, ty = 0, bx = 0, by = 0;

	/* create the histogram */
	if (disparity_histogram == NULL) {
		/* more than half of the image width is overkill */
		disparity_histogram = new int[SVS_MAX_IMAGE_WIDTH / 2];
	}

	/* clear quadrants */
	memset(valid_quadrants, 0, no_of_possible_matches * sizeof(unsigned char));

	/* create disparity histograms within different
	 * zones of the image */
	for (hf = 0; hf < 4; hf++) {

		switch (hf) {
		/* left hemifield */
		case 0: {
			tx = 0;
			ty = 0;
			bx = imgWidth / 2;
			by = imgHeight;
			break;
		}
			/* right hemifield */
		case 1: {
			tx = bx;
			bx = imgWidth;
			break;
		}
			/* upper hemifield */
		case 2: {
			tx = 0;
			ty = 0;
			bx = imgWidth;
			by = imgHeight / 2;
			break;
		}
			/* lower hemifield */
		case 3: {
			ty = by;
			by = imgHeight;
			break;
		}
		}

		/* clear the histogram */
		memset(disparity_histogram, 0, max_disparity_pixels * sizeof(int));
		int hist_max = 0;

		/* update the disparity histogram */
		for (i = 0; i < no_of_possible_matches; i++) {
			unsigned int x = svs_matches[i * 4 + 1];
			if ((x > tx) && (x < bx)) {
				unsigned int y = svs_matches[i * 4 + 2];
				if ((y > ty) && (y < by)) {
					int disp = svs_matches[i * 4 + 3];
					disparity_histogram[disp]++;
					if (disparity_histogram[disp] > hist_max)
						hist_max = disparity_histogram[disp];
				}
			}
		}

		/* locate the histogram peak */
		int mass = 0;
		int disp2 = 0;
		int hist_thresh = hist_max / 4;
		int hist_mean = 0;
		int hist_mean_hits = 0;
		for (int d = 3; d < max_disparity_pixels - 1; d++) {
			if (disparity_histogram[d] > hist_thresh) {
				int m = disparity_histogram[d] + disparity_histogram[d - 1]
						+ disparity_histogram[d + 1];
				mass += m;
				disp2 += m * d;
			}
			if (disparity_histogram[d] > 0) {
				hist_mean += disparity_histogram[d];
				hist_mean_hits++;
			}
		}
		if (mass > 0) {
			disp2 /= mass;
			hist_mean /= hist_mean_hits;
		}

		/* simple near/far classification adjusts
		 * the peak disparity that we're interested in */
		int near = 1;
		if (hist_mean * 4 > disparity_histogram[0]) {
			near = 0;
		}

		/* remove matches too far away from the peak by setting
		 * their probabilities to zero */
		unsigned int min_disp = disp2 - tolerance;
		unsigned int max_disp = disp2 + tolerance;
		for (i = 0; i < no_of_possible_matches; i++) {
			unsigned int x = svs_matches[i * 4 + 1];
			if ((x > tx) && (x < bx)) {
				unsigned int y = svs_matches[i * 4 + 2];
				if ((y > ty) && (y < by)) {
					unsigned int disp = svs_matches[i * 4 + 3];
					if (near == 1) {
						if (!((disp < min_disp) || (disp > max_disp))) {
							/* near - within stereo ranging resolution */
							valid_quadrants[i]++;
						}
					} else {
						if (disp <= 2) {
							/* far out man */
							valid_quadrants[i]++;
						}
					}
				}
			}
		}
	}

	for (i = 0; i < no_of_possible_matches; i++) {
		if (valid_quadrants[i] == 0) {
			/* set probability to zero */
			svs_matches[i * 4] = 0;
		}
	}
}

/* calculate offsets assuming that the cameras are looking at some distant object */
void svs::calibrate_offsets(unsigned char* left_image, /* left image data */
unsigned char* right_image, /* right image data */
int x_range, /* horizontal search range in pixels */
int y_range, /* vertical search range in pixels */
int& calibration_offset_x, /* returned horizontal offset in pixels */
int& calibration_offset_y) { /* returned vertical offset in pixels */

	int tx = imgWidth * 25 / 100;
	int ty = imgHeight * 25 / 100;
	int bx = imgWidth - tx;
	int by = imgHeight - ty;

	int min_diff = (bx - tx) * (by - ty) * 3* 255 ;
	for (int offset_y = -y_range; offset_y < y_range; offset_y++) {
		for (int offset_x = -x_range; offset_x < x_range; offset_x++) {

			int diff = 0;
			for (int y = ty; y < by; y++) {
				int n = ((y * imgWidth) + tx) * 3;
				int n2 = (((y + offset_y) * imgWidth) + (tx + offset_x)) * 3;
				for (int x = tx; x < bx; x++, n += 3, n2 += 3) {

					for (int col = 0; col < 3; col++) {
						int v = left_image[n + col] - right_image[n2 + col];
						if (v < 0)
							v = -v;
						diff += v;
					}

				}
			}
			if (diff < min_diff) {
				min_diff = diff;
				calibration_offset_x = offset_x;
				calibration_offset_y = offset_y;
			}
		}
	}
}

/* creates a calibration map */
void svs::make_map(
float centre_of_distortion_x, /* centre of distortion x coordinate in pixels */
float centre_of_distortion_y, /* centre of distortion y coordinate in pixels */
float coeff_0, /* lens distortion polynomial coefficient 0 */
float coeff_1, /* lens distortion polynomial coefficient 1 */
float coeff_2, /* lens distortion polynomial coefficient 2 */
float rotation, /* camera rotation (roll angle) in radians */
float scale) { /* scaling applied */

	/* free existing map */
	if (calibration_map != NULL) free(calibration_map);

	polynomial* distortion_curve = new polynomial();
	distortion_curve->SetDegree(3);
	distortion_curve->SetCoeff(0, 0);
	distortion_curve->SetCoeff(1, coeff_0);
	distortion_curve->SetCoeff(2, coeff_1);
	distortion_curve->SetCoeff(3, coeff_2);

	int half_width = imgWidth / 2;
	int half_height = imgHeight / 2;
	calibration_map = new int[imgWidth * imgHeight];
	for (int x = 0; x < (int) imgWidth; x++) {

		float dx = x - centre_of_distortion_x;

		for (int y = 0; y < (int) imgHeight; y++) {

			float dy = y - centre_of_distortion_y;

			float radial_dist_rectified = (float) sqrt(dx * dx + dy * dy);
			if (radial_dist_rectified >= 0.01f) {

				double radial_dist_original = distortion_curve->RegVal(
						radial_dist_rectified);
				if (radial_dist_original > 0) {

					double ratio = radial_dist_original
							/ radial_dist_rectified;
					float x2 = (float) round(centre_of_distortion_x + (dx
							* ratio));
					x2 = (x2 - (imgWidth / 2)) * scale;
					float y2 = (float) round(centre_of_distortion_y + (dy
							* ratio));
					y2 = (y2 - (imgHeight / 2)) * scale;

					// apply rotation
					double x3 = x2, y3 = y2;
					double hyp;
					if (rotation != 0) {
						hyp = sqrt(x2 * x2 + y2 * y2);
						if (hyp > 0) {
							double rot_angle = acos(y2 / hyp);
							if (x2 < 0)
								rot_angle = (3.1415927 * 2) - rot_angle;
							double new_angle = rotation + rot_angle;
							x3 = hyp * sin(new_angle);
							y3 = hyp * cos(new_angle);
						}
					}

					x3 += half_width;
					y3 += half_height;

					if (((int) x3 > -1) && ((int) x3 < (int) imgWidth)
							&& ((int) y3 > -1) && ((int) y3
							< (int) imgHeight)) {

						int n = (y * imgWidth) + x;
						int n2 = ((int) y3 * imgWidth) + (int) x3;

						calibration_map[n] = n2;
					}
				}
			}
		}
	}
	delete distortion_curve;
}

/* takes the raw image and camera calibration parameters and returns a rectified image */
void svs::rectify(unsigned char* raw_image, /* raw image grabbed from camera */
unsigned char* rectified_frame_buf) {       /* returned rectified image */

	int max, n, i, idx;

	if (calibration_map != NULL) {

	    n = 0;
	    max = imgWidth * imgHeight * 3;
	    for (i = 0; i < max; i += 3, n++) {
		    idx = calibration_map[n] * 3;
		    rectified_frame_buf[i] = raw_image[idx];
		    rectified_frame_buf[i + 1] = raw_image[idx + 1];
		    rectified_frame_buf[i + 2] = raw_image[idx + 2];
	    }
	}
}


/* takes the raw image and camera calibration parameters and returns a rectified image */
void svs::make_map_int(long centre_of_distortion_x, /* centre of distortion x coordinate in pixels xSVS_MULT */
long centre_of_distortion_y, /* centre of distortion y coordinate in pixels xSVS_MULT */
long* coeff, /* three lens distortion polynomial coefficients xSVS_MULT_COEFF */
long scale_num, /* scaling numerator */
long scale_denom) { /* scaling denominator */

	const long SVS_MULT = 1;
	const long SVS_MULT_COEFF = 10000000;

	long v, powr, radial_dist_rectified, radial_dist_original;
	long i, j, x, y, dx, dy;
	long n, n2, x2, y2;
	long ww = imgWidth;
	long hh = imgHeight;
	long half_width = ww / 2;
	long half_height = hh / 2;
	scale_denom *= SVS_MULT;
	calibration_map = new int[imgWidth * imgHeight];
	for (x = 0; x < ww; x++) {

		dx = (x * SVS_MULT) - centre_of_distortion_x;

		for (y = 0; y < hh; y++) {

			dy = (y * SVS_MULT) - centre_of_distortion_y;

			v = dx * dx + dy * dy;
			/* integer square root */
			for (radial_dist_rectified = 0; v >= (2* radial_dist_rectified )
					+ 1; v -= (2* radial_dist_rectified ++) + 1)
				;

			//float radial_dist_rectified2 = (float) sqrt(dx * dx + dy * dy);

			//printf("radial_dist_rectified %ld %f\n", radial_dist_rectified, radial_dist_rectified2);

			if (radial_dist_rectified >= 0) {

				/* for each polynomial coefficient */
				radial_dist_original = 0;
				double radial_dist_original2 = 0;
				for (i = 0; i < 4; i++) {

					/* pow(radial_dist_rectified, i) */
					powr = 1;
					if (i != 0) {
						j = i;
						while (j != 0) {
							powr *= radial_dist_rectified;
							j--;
						}
					} else {
						powr = 1;
					}

					//printf("powr %ld %lf\n", powr, pow(radial_dist_rectified, i));

					/*
					if (coeff[i] >= 0) {
					    radial_dist_original += (unsigned long)coeff[i] * powr;
					}
					else {
						radial_dist_original -= (unsigned long)(-coeff[i]) * powr;
					}
					*/
					radial_dist_original += coeff[i] * powr;
					radial_dist_original2 += coeff[i] * pow(radial_dist_rectified, i);

					//printf("powr %ld %lf\n", coeff[i] * powr, (double)coeff[i] * pow(radial_dist_rectified, i));
				}
				//printf("radial_dist_original %ld %lf\n", radial_dist_original, radial_dist_original2);

				if (radial_dist_original > 0) {

					radial_dist_original /= SVS_MULT_COEFF;
					radial_dist_original2 /= SVS_MULT_COEFF;
					//printf("radial_dist_original %ld %lf\n", radial_dist_original, radial_dist_original2);

					//printf("radial_dist_rectified: %d\n", (int)(radial_dist_rectified/SVS_MULT));
					//printf("radial_dist_original:  %d\n", (int)(radial_dist_original/SVS_MULT));

					x2 = centre_of_distortion_x + (dx * radial_dist_original
							/ radial_dist_rectified);
					x2 = (x2 - (half_width * SVS_MULT)) * scale_num
							/ scale_denom;
					y2 = centre_of_distortion_y + (dy * radial_dist_original
							/ radial_dist_rectified);
					y2 = (y2 - (half_height * SVS_MULT)) * scale_num
							/ scale_denom;

					x2 += half_width;
					y2 += half_height;

					if ((x2 > -1) && (x2 < ww) && (y2 > -1) && (y2 < hh)) {

						n = y * imgWidth + x;
						n2 = y2 * imgWidth + x2;

						//int diff = calibration_map[(int) n] - (int) n2;
						//if (diff < 0)
						//	diff = -diff;
						//printf("diff: %d\n", diff);

						calibration_map[(int)n] = (int)n2;
					}
				}
			}
		}
	}
}

/* saves stereo matches to file for use by other programs */
void svs::save_matches(std::string filename, /* filename to save as */
unsigned char* rectified_frame_buf, /* left image data */
int no_of_matches, /* number of stereo matches */
bool colour) { /* whether to additionally save colour of each match */

	FILE *file = fopen(filename.c_str(), "wb");
	if (file != NULL) {

		if (!colour) {

			struct MatchData {
				float x;
				float y;
				float disparity;
			};

			MatchData *m = new MatchData[no_of_matches];
			for (int i = 0; i < no_of_matches; i++) {
				m[i].x = svs_matches[i * 4 + 1];
				m[i].y = svs_matches[i * 4 + 2];
				m[i].disparity = svs_matches[i * 4 + 3];
			}

			fwrite(m, sizeof(MatchData), no_of_matches, file);
			delete[] m;
		} else {
			struct MatchDataColour {
				float x;
				float y;
				float disparity;
				unsigned char r, g, b;
				unsigned char pack;
			};

			int n;
			MatchDataColour *m = new MatchDataColour[no_of_matches];
			for (int i = 0; i < no_of_matches; i++) {
				m[i].x = svs_matches[i * 4 + 1];
				m[i].y = svs_matches[i * 4 + 2];
				m[i].disparity = svs_matches[i * 4 + 3];
				n = ((m[i].y * imgWidth) + m[i].x) * 3;
				m[i].r = rectified_frame_buf[n + 2];
				m[i].g = rectified_frame_buf[n + 1];
				m[i].b = rectified_frame_buf[n];
			}

			fwrite(m, sizeof(MatchDataColour), no_of_matches, file);
			delete[] m;
		}

		fclose(file);
	}
}

/* experimental plane fitting */
int svs::fit_plane(int no_of_matches, int max_deviation, int no_of_samples) {
	int x_x0 = 0, x_y0 = 0, x_x1 = 0, x_y1 = 0;
	int y_x0 = 0, y_y0 = 0, y_x1 = 0, y_y1 = 0;
	int max_hits = 0;
	int idx0, idx1, xx0, yy0, xx1, yy1, dx, dy, abs_dx, abs_dy, hits;
	int grad_x, grad_y, index0, index1, sample, horizontal, deviation_sum;
	int best_index0, best_index1, min_deviation, min_deviation_hits = 0,
			edge_sample;
	int axis, edge_x, edge_y, deviation, predicted_edge_x, predicted_edge_y;

	if (no_of_matches > 40) {
		/* fit to x and y axes */
		for (axis = 0; axis < 2; axis++) {
			min_deviation = 999999;
			best_index0 = -1;
			best_index1 = -1;

			/* try a number of baselines */
			for (sample = 0; sample < no_of_samples; sample++) {
				// pick the baseline
				index0 = rand() % no_of_matches;
				index1 = rand() % no_of_matches;
				if (index0 != index1) {
					hits = 0;
					deviation_sum = 0;
					idx0 = index0 * 4;
					idx1 = index1 * 4;
					if (axis == 0) {
						/* oriented along the x axis */
						xx0 = svs_matches[idx0 + 1];
						xx1 = svs_matches[idx1 + 1];
					} else {
						/* oriented along the y axis */
						xx0 = svs_matches[idx0 + 2];
						xx1 = svs_matches[idx1 + 2];
					}
					yy0 = svs_matches[idx0 + 3];
					yy1 = svs_matches[idx1 + 3];
					dx = xx1 - xx0;
					dy = yy1 - yy0;
					if (dx >= 0)
						abs_dx = dx;
					else
						abs_dx = -dx;
					if (dy >= 0)
						abs_dy = dy;
					else
						abs_dy = -dy;

					// is the baseline horizontally oriented ?
					horizontal = 1;
					if (abs_dy > abs_dx) {
						horizontal = 0;
						grad_x = dx;
						grad_y = dy;
					} else {
						grad_x = dy;
						grad_y = dx;
					}

					if (grad_y != 0) {
						for (edge_sample = 0; edge_sample < no_of_matches; edge_sample
								+= 2) {
							edge_x = svs_matches[edge_sample * 4 + 1];
							edge_y = svs_matches[edge_sample * 4 + 2];
							deviation = 0;

							if (horizontal == 1) {
								predicted_edge_y = yy0 + ((edge_x - xx0)
										* grad_x / grad_y);
								deviation = predicted_edge_y - edge_y;
							} else {
								predicted_edge_x = xx0 + ((edge_y - yy0)
										* grad_x / grad_y);
								deviation = predicted_edge_x - edge_x;
							}

							if ((deviation > -max_deviation) && (deviation
									< max_deviation)) {
								hits++;
								if (deviation < 0)
									deviation = -deviation;
								deviation_sum += deviation;
							}
						}

						if (hits > 0) {
							// pick the line with the maximum number of edges within the max deviation range
							if (hits > max_hits) {
								best_index0 = index0;
								best_index1 = index1;
								max_hits = hits;
								min_deviation = deviation_sum;
							} else {
								// if there is a tie choose the result with the lowest deviation
								if (hits == max_hits) {
									if (deviation_sum < min_deviation) {
										best_index0 = index0;
										best_index1 = index1;
										min_deviation = deviation_sum;
										min_deviation_hits = hits;
									}
								}
							}
						}
					}
				}
			}

			if (min_deviation_hits > 3) {

				min_deviation /= min_deviation_hits;

				if (best_index0 > -1) {
					if (axis == 0) {
						x_x0 = svs_matches[best_index0 + 1];
						x_y0 = svs_matches[best_index0 + 3];
						x_x1 = svs_matches[best_index1 + 1];
						x_y1 = svs_matches[best_index1 + 3];
					} else {
						y_x0 = svs_matches[best_index0 + 2];
						y_y0 = svs_matches[best_index0 + 3];
						y_x1 = svs_matches[best_index1 + 2];
						y_y1 = svs_matches[best_index1 + 3];
					}
					printf("min deviation %d\n", min_deviation);
				} else {
					break;
				}
			} else {
				break;
			}
		}
	}

	return (max_hits);
}

