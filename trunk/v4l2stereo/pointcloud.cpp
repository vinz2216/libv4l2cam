/*
    Point cloud functions
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

#include "pointcloud.h"

/*
  Format: floating point array with 4 float values per point
  The first array value is the number of subsequent points,
  followed by 16 floats containing the 4x4 pose matrix.
  For each point the first three floats are the point coordinates,
  and the subsequent three bytes are the RGB values
*/
void pointcloud::save(
    unsigned char * img_left,
    IplImage * points_image, 
    int max_range_mm,
    CvMat * pose,
    std::string point_cloud_filename)
{
    FILE * fp = fopen(point_cloud_filename.c_str(),"w");
    if (fp != NULL) {
        float * points_image_data = (float*)points_image->imageData;
        int pixels = points_image->width*points_image->height;
        int n = 0, ctr = 0;
        for (int i = 0; i < pixels; i++, n += 3) {
            if ((fabs(points_image_data[n]) < max_range_mm) &&
                (fabs(points_image_data[n+1]) < max_range_mm) &&
                (fabs(points_image_data[n+2]) < max_range_mm)) {
                ctr++;
            }
        }
        if (ctr > 0) {
            float * data = new float[1+16+(ctr*4)];
            unsigned char * data_bytes = (unsigned char*)data;
            data[0] = ctr;
            ctr = 1;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++, ctr++) {
                   data[ctr] = cvmGet(pose,i,j); 
                }
            }
            n = 0;
            for (int i = 0; i < pixels; i++, n += 3) {
                if ((fabs(points_image_data[n]) < max_range_mm) &&
                    (fabs(points_image_data[n+1]) < max_range_mm) &&
                    (fabs(points_image_data[n+2]) < max_range_mm)) {
                    data[ctr] = points_image_data[n];
                    data[ctr+1] = points_image_data[n+1];
                    data[ctr+2] = points_image_data[n+2];
                    int n2 = (ctr*sizeof(float)) + (3*sizeof(float));
                    data_bytes[n2] = img_left[n];
                    data_bytes[n2+1] = img_left[n+1];
                    data_bytes[n2+2] = img_left[n+2];
                    ctr+=4;
                }
            }
            fwrite(data,sizeof(float),ctr,fp);
            printf("%d points saved\n", (int)data[0]);
            delete [] data;
        }
        fclose(fp);
    }
}

void pointcloud::disparity_map_to_3d_points(
    float * disparity_map,
    int img_width,
    int img_height,
    CvMat * disparity_to_depth,
    CvMat * pose,
    IplImage * &disparity_image,
    IplImage * &points_image)
{
    if (disparity_to_depth != NULL) {
        // create image objects
        if (points_image == NULL) {
            points_image = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_32F, 3);
            disparity_image = cvCreateImage(cvSize(img_width, img_height), IPL_DEPTH_32F, 1);
        }

        // insert disparities into the image object
        float * disparity_image_data = (float*)disparity_image->imageData;
        for (int i = 0; i < img_width*img_height; i++) {
            disparity_image_data[i] = disparity_map[i];
        }

        // reproject the points in an egocentric coordinate frame
        cvReprojectImageTo3D(disparity_image, points_image, disparity_to_depth);

        // reposition the points according to the camera pose
        float * points_image_data = (float*)points_image->imageData;
        int n = 0;
        float xx = cvmGet(pose,0,3);
        float yy = cvmGet(pose,1,3);
        float zz = cvmGet(pose,2,3);
        for (int i = 0; i < img_width*img_height; i++, n += 3) {
            if (disparity_map[i] > 0) {
                float px = (cvmGet(pose, 0, 0)*points_image_data[n] + cvmGet(pose, 0, 1)*points_image_data[n+1] + cvmGet(pose, 0, 2)*points_image_data[n+2]);
                float py = (cvmGet(pose, 1, 0)*points_image_data[n] + cvmGet(pose, 1, 1)*points_image_data[n+1] + cvmGet(pose, 1, 2)*points_image_data[n+2]);
                float pz = (cvmGet(pose, 2, 0)*points_image_data[n] + cvmGet(pose, 2, 1)*points_image_data[n+1] + cvmGet(pose, 2, 2)*points_image_data[n+2]);
                points_image_data[n] = xx + px;
                points_image_data[n+1] = yy + py;
                points_image_data[n+2] = zz + pz;
            }
            else {
                points_image_data[n] = xx;
                points_image_data[n+1] = yy;
                points_image_data[n+2] = zz;
            }
        }
    }
}

void pointcloud::show(
    IplImage * points_image,
    float * disparity_map,
    unsigned char * img_left,
    CvMat * pose,
    float max_range_mm,
    float max_height_mm,
    int view_type,
    int output_image_width,
    int output_image_height,
    unsigned char * img_output)
{
    float cx = cvmGet(pose,0,3);
    float cy = cvmGet(pose,1,3);
    float cz = cvmGet(pose,2,3);
    float px,py,pz;
    int ix=0,iy=0,prev_ix,prev_iy,dx,dy,length;
    int n = 0;
    float * points_image_data = (float*)points_image->imageData;

    memset((void*)img_output,'\0',output_image_width*output_image_height*3);

    for (int y = 0; y < points_image->height; y++) {
        prev_ix = prev_iy = 0;
        for (int x = 0; x < points_image->width; x++, n++) {
            if (disparity_map[n] > 1) {
                px = points_image_data[n*3] - cx;
                py = points_image_data[n*3 + 1] - cy;
                pz = points_image_data[n*3 + 2] - cz;
                switch(view_type) {
                    case 0: {
                        ix = (int)((output_image_width>>1) + (px * output_image_width / (max_range_mm*2)));
                        iy = (int)((output_image_height>>1) - (pz * output_image_width / (max_height_mm*2)));
                        break;
                    }
                    case 1: {
                        ix = (int)(py * output_image_width / max_range_mm);
                        iy = (int)((output_image_height>>1) - (pz * output_image_width / max_height_mm));
                        break;
                    }
                    case 2: {
                        ix = (int)((output_image_width>>1) + (px * output_image_width / max_range_mm));
                        iy = (int)(py * output_image_width / max_range_mm);
                        break;
                    }
                }
                if ((!((prev_ix == 0) && (prev_iy == 0))) &&
                    ((ix > 0) && (ix < output_image_width) &&
                     (iy > 0) && (iy < output_image_height))) {
                    dx = ix - prev_ix;
                    dy = iy - prev_iy;
                    length = (int)sqrt(dx*dx + dy*dy);
                    if (length>2) length=2;
                    for (int i = 0; i < length; i++) {
                        int ixx = prev_ix + (i * dx / length);
                        int iyy = prev_iy + (i * dy / length);
                        if ((ixx > 0) && (ixx < output_image_width) &&
                            (iyy > 0) && (iyy < output_image_height)) {
                            int nn = (iyy*output_image_width + ixx)*3;
                            img_output[nn] = img_left[n*3];
                            img_output[nn+1] = img_left[n*3+1];
                            img_output[nn+2] = img_left[n*3+2];
                        }
                    }
                }
                prev_ix = ix;
                prev_iy = iy;
            }
        }
    }
}



