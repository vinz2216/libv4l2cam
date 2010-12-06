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

class camcalibbase {
private:
    CvMat* matMul(const CvMat* A, const CvMat* B);

    void SetIntrinsic(
        double * intrinsic,
        int camera_right);

    void SetExtrinsicRotation(
        double * extrinsic);

    void SetExtrinsicTranslation(
        double * extrinsic);

    std::string lowercase(
        std::string str);

    int ParseCalibrationFileMatrix(
        std::string calibration_filename,
        std::string title,
        double * matrix_data,
        int rows);

    void matSet(CvMat * m, double * data);

public:
    CvMat * intrinsicCalibration_left;
    CvMat * intrinsicCalibration_right;
    CvMat * distortion_left;
    CvMat * distortion_right;
    CvMat * extrinsicRotation;
    CvMat * extrinsicTranslation;
    CvMat * disparityToDepth;
    CvMat * fundamentalMatrix;
    CvMat * essentialMatrix;
    CvMat * pose;
    int v_shift;

    void SetFundamentalMatrix(
        double * matrix);

    void SetEssentialMatrix(
        double * matrix);

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

    void SetPose(
        double * pose_matrix);

    void SetPoseRotation(
        double * pose_vector);

    int ParsePose(
        char * pose_str);

    int ParsePoseRotation(
        char * pose_str);

    int ParseDistortion(
        char * distortion_str,
        int camera_right);

    int ParsePoseTranslation(
        char * pose_str);

    void SetDistortion(
        double * distortion_vector,
        int camera_right);

    void GetPoseRotation(
        double * rotation_vector);

    void SetPoseTranslation(
        double * pose_matrix);

    void ParseCalibrationFile(
        std::string calibration_filename);

    void translate_pose(double distance_mm, int axis);
    void rotate_pose(double angle_degrees, int axis);

    camcalibbase();
    ~camcalibbase();
};

#endif

