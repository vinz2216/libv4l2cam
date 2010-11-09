/*
    Stereo camera calibration using OpenCV
    Functions for simple sparse stereo
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

#ifndef CAMCALIB_H_
#define CAMCALIB_H_

#include <cctype>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <omp.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

#include "calibration/mcv.h"
#include "calibration/CvexStereoCameraCalibration.h"

#include "rectify.h"
#include "libcam.h"

class camcalib {
private:
    Rectify rectification[2];

    CvMat* matMul(const CvMat* A, const CvMat* B);

    void rotationMatrixFromEuler(
        double xAngle, double yAngle, double zAngle,
        CvMat * rotation);

    void SetIntrinsic(
        double * intrinsic,
        int camera_right);

    void SetExtrinsicRotation(
        double * extrinsic);

    void SetDisparityToDepth(
        double * disparity_to_depth);

    void SetExtrinsicTranslation(
        double * extrinsic);

    void editHomographyGUI(
        IplImage* leftim,
        IplImage* rightim,
        CvMat* hl,
        CvMat* hr);

    void getShiftRectificationParameterGUI(
        IplImage* leftim,
        IplImage* rightim,
        int *_v);

    void getShiftRectificationParameter(
        IplImage* leftim,
        IplImage* rightim,
        int *_v);

    void cvexMakeStereoImageSidebySide(
        IplImage* left,
        IplImage* right,
        IplImage* dest,
        int shift,
        int mode = CVEX_CONNECT_HORIZON);

    void cvexMakeStereoImageInterlace(
        IplImage* left,
        IplImage* right,
        IplImage* dest,
        int shift);

    void flip(
        int image_width,
        int image_height,
        unsigned char* raw_image,
        unsigned char* flipped_frame_buf);

    std::string lowercase(
        std::string str);

public:
    CvMat intrinsicCalibration_right;
    CvMat intrinsicCalibration_left;
    CvMat extrinsicRotation;
    CvMat extrinsicTranslation;
    CvMat disparityToDepth;

    void stereo_camera_calibrate(
        int image_width,
        int image_height,
        int pattern_squares_x,
        int pattern_squares_y,
        int square_size_mm,
        std::string dev0,
        std::string dev1,
        bool flip_left_image,
        bool flip_right_image,
        int calibration_images,
        bool headless);

    int ParseCalibrationParameters(
        char * calibration_str,
        int &pattern_squares_x,
        int &pattern_squares_y,
        int &square_size_mm);

    int ParseExtrinsicTranslation(
        char * extrinsic_str);

    int ParseExtrinsicRotation(
        char * extrinsic_str);

    int ParseIntrinsic(
        char * intrinsic_str,
        int camera_right);

    void SetRectification(
        double * params,
        int camera_right);

    int ParseRectification(
        char * rectification_str,
        int camera_right);

    void RectifyImage(
        int right_image,
        int image_width,
        int image_height,
        unsigned char * image_data,
        int v_shift);

    void RectifyImage(
        int right_image,
        int image_width,
        int image_height,
        int16_t *image_data,
        int v_shift);

    void SetStereoCamera(
        std::string camera_type);

    camcalib();
    ~camcalib();
};

#endif

