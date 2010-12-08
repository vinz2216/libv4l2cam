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

#include "camcalibbase.h"


camcalibbase::camcalibbase()
{
    v_shift = 0;
    double intCalib[] = {
        340,   0, 330,
          0, 340,  94,
          0,   0,   1
    };
    intrinsicCalibration_right = cvCreateMat(3,3,CV_64F);
    matSet(intrinsicCalibration_right,intCalib);
    intrinsicCalibration_left = cvCreateMat(3,3,CV_64F);
    matSet(intrinsicCalibration_left,intCalib);

    double distortion[] = {
        -0.38407,  0.13186,  0.00349,  -0.00392
    };
    distortion_left = cvCreateMat(1,4,CV_64F);
    matSet(distortion_left, distortion);
    distortion_right = cvCreateMat(1,4,CV_64F);
    matSet(distortion_right, distortion);

    double trans[] = { -0.575, 0, 0 };
    extrinsicTranslation = cvCreateMat(3,1,CV_64F);
    matSet(extrinsicTranslation, trans);

    disparityToDepth = cvCreateMat(4,4,CV_64F);
    extrinsicRotation = cvCreateMat(3,1,CV_64F);
    fundamentalMatrix = cvCreateMat(3,3,CV_64F);
    essentialMatrix = cvCreateMat(3,3,CV_64F);

    double initPose[] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 0
    };
    pose = cvCreateMat(4,4,CV_64F);
    matSet(pose, initPose);
}

camcalibbase::~camcalibbase()
{
    cvReleaseMat(&intrinsicCalibration_right);
    cvReleaseMat(&intrinsicCalibration_left);
    cvReleaseMat(&extrinsicRotation);
    cvReleaseMat(&extrinsicTranslation);
    cvReleaseMat(&disparityToDepth);
    cvReleaseMat(&fundamentalMatrix);
    cvReleaseMat(&essentialMatrix);
    cvReleaseMat(&pose);
    cvReleaseMat(&distortion_left);
    cvReleaseMat(&distortion_right);
}

void camcalibbase::matSet(CvMat * m, double * data)
{
    int i=0;
    for (int y = 0; y < m->rows; y++) {
        for (int x = 0; x < m->cols; x++,i++) {
            cvmSet(m,y,x,data[i]);
        }
    }
}

CvMat* camcalibbase::matMul(const CvMat* A, const CvMat* B) 
{
    assert(A->cols == B->rows);

    CvMat* M = cvCreateMat(A->rows, B->cols, A->type);
    cvMatMul(A, B, M);
    return M;
}

int camcalibbase::ParseIntrinsic(
    char * intrinsic_str,
    int camera_right)
{
    char str[256];
    double params[3*3];
    int i=0,index=0,p=0,success=0;
    while (intrinsic_str[i]!=0) {
        if ((index > 0) &&
            (intrinsic_str[i]==' ')) {
            str[index]=0;
            params[p++] = atof(str);
            index=0;   
        }
        else {
            str[index++] = intrinsic_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atof(str);
    }
    if (p==9) {
        SetIntrinsic(params,camera_right);
        success=1;
    }
    return success;
}

int camcalibbase::ParsePoseRotation(
    char * pose_str)
{
    char str[256];
    double params[3];
    int i=0,index=0,p=0,success=0;
    while (pose_str[i]!=0) {
        if ((index > 0) &&
            (pose_str[i]==' ')) {
            str[index]=0;
            params[p++] = atof(str);
            index=0;   
        }
        else {
            str[index++] = pose_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atof(str);
    }
    if (p==3) {
        SetPoseRotation(params);
        success=1;
    }
    return success;
}

int camcalibbase::ParsePoseTranslation(
    char * pose_str)
{
    char str[256];
    double params[3];
    int i=0,index=0,p=0,success=0;
    while (pose_str[i]!=0) {
        if ((index > 0) &&
            (pose_str[i]==' ')) {
            str[index]=0;
            params[p++] = atof(str);
            index=0;   
        }
        else {
            str[index++] = pose_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atof(str);
    }
    if (p==3) {
        SetPoseTranslation(params);
        success=1;
    }
    return success;
}

int camcalibbase::ParseDistortion(
    char * distortion_str,
    int camera_right)
{
    char str[256];
    double params[4];
    int i=0,index=0,p=0,success=0;
    while (distortion_str[i]!=0) {
        if ((index > 0) &&
            (distortion_str[i]==' ')) {
            str[index]=0;
            params[p++] = atof(str);
            index=0;
        }
        else {
            str[index++] = distortion_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atof(str);
    }
    if (p==4) {
        SetDistortion(params,camera_right);
        success=1;
    }
    return success;
}


int camcalibbase::ParsePose(
    char * pose_str)
{
    char str[256];
    double params[4*4];
    int i=0,index=0,p=0,success=0;
    while (pose_str[i]!=0) {
        if ((index > 0) &&
            (pose_str[i]==' ')) {
            str[index]=0;
            params[p++] = atof(str);
            index=0;   
        }
        else {
            str[index++] = pose_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atof(str);
    }
    if (p==16) {
        SetPose(params);
        success=1;
    }
    return success;
}

int camcalibbase::ParseExtrinsicRotation(
    char * extrinsic_str)
{
    char str[256];
    double params[3*3];
    int i=0, index=0, p=0, success=0;
    while (extrinsic_str[i]!=0) {
        if ((index > 0) &&
            (extrinsic_str[i]==' ')) {
            str[index]=0;
            params[p++] = atof(str);
            index=0;   
        }
        else {
            str[index++] = extrinsic_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atof(str);
    }
    if (p==9) {
        SetExtrinsicRotation(params);
        success=1;
    }
    return success;
}

int camcalibbase::ParseExtrinsicTranslation(
    char * extrinsic_str)
{
    char str[256];
    double params[3];
    int i=0,index=0,p=0,success=0;

    while (extrinsic_str[i]!=0) {
        if ((index > 0) &&
            (extrinsic_str[i]==' ')) {
            str[index]=0;
            params[p++] = atof(str);
            index=0;   
        }
        else {
            str[index++] = extrinsic_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atof(str);
    }
    if (p==3) {
        SetExtrinsicTranslation(params);
        success=1;
    }
    return success;
}

int camcalibbase::ParseCalibrationParameters(
    char * calibration_str,
    int &pattern_squares_x,
    int &pattern_squares_y,
    int &square_size_mm)
{
    char str[256];
    int params[3];
    int i=0,index=0,p=0,success=0;

    while (calibration_str[i]!=0) {
        if ((index > 0) &&
            (calibration_str[i]==' ')) {
            str[index]=0;
            params[p++] = atoi(str);
            index=0;   
        }
        else {
            str[index++] = calibration_str[i];
        }
        if (i==255) break;
        i++;
    }
    if (index > 0) {
        str[index]=0;
        params[p++] = atoi(str);
    }
    if (p==3) {
        pattern_squares_x = params[0];
        pattern_squares_y = params[1];
        square_size_mm = params[2];
        success=1;
    }
    return success;
}

void camcalibbase::SetExtrinsicTranslation(
    double * extrinsic)
{
    matSet(extrinsicTranslation, extrinsic);
}

void camcalibbase::SetExtrinsicRotation(
    double * extrinsic)
{
    matSet(extrinsicRotation, extrinsic);
}

void camcalibbase::SetIntrinsic(
    double * intrinsic,
    int camera_right)
{
    if (camera_right==0) {
        matSet(intrinsicCalibration_left,intrinsic);
    }
    else {
        matSet(intrinsicCalibration_right,intrinsic);
    }
}

void camcalibbase::SetPose(
    double * pose_matrix)
{
    matSet(pose,pose_matrix);
}

void camcalibbase::SetDistortion(
    double * distortion_vector,
    int camera_right)
{
    if (camera_right==0) {
        matSet(distortion_left, distortion_vector);
    }
    else {
        matSet(distortion_right, distortion_vector);
    }
}

void camcalibbase::SetPoseRotation(
    double * pose_vector)
{
    for (int i = 0; i < 3; i++) {
        if (pose_vector[i]!=0) rotate_pose(pose_vector[i], i);
    }
}

void camcalibbase::GetPoseRotation(
    double * rotation_vector)
{
    CvMat * rot = cvCreateMat(3, 1, CV_64F);
    CvMat * rotmat = cvCreateMat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            cvmSet(rotmat, i, j, cvmGet(pose,i,j));
        }
    }
    // convert from 3x3 matrix to 3x1 rotation vector
    cvRodrigues2(rotmat, rot);
    for (int i = 0; i < 3; i++) {
        rotation_vector[i] = cvmGet(rot, i, 0)*180.0/3.1415927;
    }
    cvReleaseMat(&rot);
    cvReleaseMat(&rotmat);
}


void camcalibbase::SetPoseTranslation(
    double * pose_matrix)
{
    for (int i = 0;i < 3; i++) {
        cvmSet(pose,i,3,pose_matrix[i]);
    }
}

void camcalibbase::SetFundamentalMatrix(
    double * matrix)
{
    matSet(fundamentalMatrix, matrix);
}

void camcalibbase::SetEssentialMatrix(
    double * matrix)
{
    matSet(essentialMatrix, matrix);
}

int camcalibbase::ParseCalibrationFileMatrix(
    std::string calibration_filename,
    std::string title,
    double * matrix_data,
    int rows)
{
    FILE * fp;
    int matrix_index=0;
    char * retval;
    const char * title_str = title.c_str();
    int i,length1 = title.length();
    fp = fopen(calibration_filename.c_str(),"r");
    if (fp != NULL) {
        char str[100];
        while (!feof(fp)) {
            retval = fgets (str, 100, fp);
            for (i = 0; i < length1; i++) {
                if (str[i]!=title_str[i]) break;
                if (i>=(int)strlen(str)) break;
            }
            if (i > length1-3) {
                for (int row = 0; row < rows; row++) {
                    char c=1;
                    int index=0;
                    while ((c != '\n') && (!feof(fp)) && (index < 100)) {
                        c = fgetc(fp);
                        if (((c>='0') && (c<='9')) || (c=='-') || (c=='.') || (c==',')) {
                            str[index++] = c;
                        }
                        else {
                            if (index > 0) {
                                str[index]=0;
                                matrix_data[matrix_index++] = atof(str);
                            }
                            index = 0;
                        }
                    }
                }
                break;
            }
        }
        fclose(fp);
    }
    return matrix_index;
}

void camcalibbase::translate_pose(double distance_mm, int axis)
{
    cvmSet(pose,axis,3,cvmGet(pose,axis,3)+distance_mm);
}

void camcalibbase::rotate_pose(double angle_degrees, int axis)
{
    float angle_radians = angle_degrees*3.1415927f/180.0f;
    CvMat * rotation_matrix = cvCreateMat(4, 4, CV_64F);

    // identity
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            if (x != y) {
                cvmSet(rotation_matrix,y,x,0);
            }
            else {
                cvmSet(rotation_matrix,y,x,1);
            }
        }
    }
    
    switch(axis) {
        case 0: { // rotate around X axis
            cvmSet(rotation_matrix,1,1,cos(angle_radians));
            cvmSet(rotation_matrix,1,2,sin(angle_radians));
            cvmSet(rotation_matrix,2,1,-sin(angle_radians));
            cvmSet(rotation_matrix,2,2,cos(angle_radians));
            break;
        }
        case 1: { // rotate around Y axis
            cvmSet(rotation_matrix,0,0,cos(angle_radians));
            cvmSet(rotation_matrix,0,2,-sin(angle_radians));
            cvmSet(rotation_matrix,2,0,sin(angle_radians));
            cvmSet(rotation_matrix,2,2,cos(angle_radians));
            break;
        }
        case 2: { // rotate around Z axis
            cvmSet(rotation_matrix,0,0,cos(angle_radians));
            cvmSet(rotation_matrix,0,1,sin(angle_radians));
            cvmSet(rotation_matrix,1,0,-sin(angle_radians));
            cvmSet(rotation_matrix,1,1,cos(angle_radians));
            break;
        }
    }

    CvMat * new_pose = matMul(rotation_matrix, pose);

    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            cvmSet(pose,y,x,cvmGet(new_pose,y,x));
        }
    }

    cvReleaseMat(&rotation_matrix);
    cvReleaseMat(&new_pose);
}

void camcalibbase::ParseCalibrationFile(
    std::string calibration_filename)
{
    double matrix_data[4*4];

    FILE * fp = fopen(calibration_filename.c_str(),"r");
    if (fp==NULL) return;
    fclose(fp);

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Left Distortion Parameters:",
        (double*)matrix_data, 1) == 4) {
        SetDistortion(matrix_data, 0);
    }

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Right Distortion Parameters:",
        (double*)matrix_data, 1) == 4) {
        SetDistortion(matrix_data, 1);
    }

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Relative Translation:",
        (double*)matrix_data, 3) == 3) {
        SetExtrinsicTranslation(matrix_data);
    }

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Relative Rotation:",
        (double*)matrix_data, 3) == 3) {
        SetExtrinsicRotation(matrix_data);
    }

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Left Intrinsic Parameters:",
        (double*)matrix_data, 3) == 9) {
        SetIntrinsic(matrix_data, 0);
    }

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Right Intrinsic Parameters:",
        (double*)matrix_data, 3) == 9) {
        SetIntrinsic(matrix_data, 1);
    }

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Essensial Matrix:",
        (double*)matrix_data, 3) == 9) {
        SetEssentialMatrix(matrix_data);
    }

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Fundamental Matrix:",
        (double*)matrix_data, 3) == 9) {
        SetFundamentalMatrix(matrix_data);
    }

    if (ParseCalibrationFileMatrix(
        calibration_filename,
        "Vertical shift:",
        (double*)matrix_data, 1) == 1) {
        v_shift = (int)matrix_data[0];
    }
}

