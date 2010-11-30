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
        }
        if (disparity_image == NULL) {
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


void pointcloud::virtual_camera(
    unsigned char * img,
    IplImage * points_image,
    CvMat * pose,
    CvMat * intrinsic_matrix,
    CvMat * distortion_coeffs,
    float max_range_mm,
    bool view_point_cloud,
    unsigned char * img_output)
{
    int pixels = points_image->width*points_image->height;
    float * depth = new float[pixels];
    CvMat * rotation_matrix = cvCreateMat(3, 3, CV_32F);
    CvMat * translation = cvCreateMat(3, 1, CV_32F);
    CvMat * rotation_vector = cvCreateMat(3, 1, CV_32F);
    CvMat * points = cvCreateMat(pixels, 3, CV_32F);
    CvMat * image_points = cvCreateMat(pixels, 2, CV_32F);

    virtual_camera(
        img, points_image, pose,
        intrinsic_matrix, distortion_coeffs,
        max_range_mm, depth,
        rotation_matrix, translation, rotation_vector,
        points, image_points,
        view_point_cloud,
        img_output);

    cvReleaseMat(&rotation_matrix);
    cvReleaseMat(&translation);
    cvReleaseMat(&rotation_vector);
    cvReleaseMat(&points);
    cvReleaseMat(&image_points);
    delete [] depth;
}



void pointcloud::virtual_camera(
    unsigned char * img,
    IplImage * points_image,
    CvMat * pose,
    CvMat * intrinsic_matrix,
    CvMat * distortion_coeffs,
    float max_range_mm,
    float * &depth,
    CvMat * &rotation_matrix,
    CvMat * &translation,
    CvMat * &rotation_vector,
    CvMat * &points,
    CvMat * &image_points,
    bool view_point_cloud,
    unsigned char * img_output)
{
    int pixels = points_image->width*points_image->height;
    max_range_mm *= max_range_mm;

    if (depth == NULL) {
        depth = new float[pixels];
        rotation_matrix = cvCreateMat(3, 3, CV_32F);
        translation = cvCreateMat(3, 1, CV_32F);
        rotation_vector = cvCreateMat(3, 1, CV_32F);
        points = cvCreateMat(pixels, 3, CV_32F);
        image_points = cvCreateMat(pixels, 2, CV_32F);
    }

    // view rotation
    for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 3; x++) {
            cvmSet(rotation_matrix,y,x,cvmGet(pose,y,x));
        }
    }
    // view translation
    for (int i = 0; i < 3; i++) {
        cvmSet(translation,i,0,cvmGet(pose,i,3));
    }
    // convert from 3x3 to 3x1
    cvRodrigues2(rotation_matrix, rotation_vector);

    float * points_image_data = (float*)points_image->imageData;

    // set 3D points
    for (int i = 0; i < pixels; i++) {
        cvmSet(points,i,0,points_image_data[i*3]);
        cvmSet(points,i,1,points_image_data[i*3+1]);
        cvmSet(points,i,2,points_image_data[i*3+2]);
    }

    // project points
    cvProjectPoints2(
        points,
        rotation_vector, translation,
        intrinsic_matrix,
        distortion_coeffs,
        image_points);

    // draw
    memset((void*)img_output,'\0',pixels*3);
    memset((void*)depth,'\0',pixels*sizeof(float));
    int prev_i=-1,prev_x=-1,prev_y=-1,length;
    float prev_dist=0;
    int w = points_image->width;
    int h = points_image->height;
    int step = 1;
    if (view_point_cloud) step = 2;
    for (int py = 0; py < h; py += step) {
        for (int px = 0; px < w; px += step) {
            int i = py*w + px;
            int x = (int)cvmGet(image_points,i,0);
            if ((x >=0) && (x < w)) {
                int y = (int)cvmGet(image_points,i,1);
                if ((y >=0) && (y < points_image->height-2)) {
                    int n = y*w + (w-1-x);

                    float dx = points_image_data[i*3] - cvmGet(translation,0,0);
                    float dy = points_image_data[i*3+1] - cvmGet(translation,1,0);
                    float dz = points_image_data[i*3+2] - cvmGet(translation,2,0);
                    float dist = dx*dx + dy*dy + dz*dz;
                    if ((dist > 0) && (dist < max_range_mm)) {
                        if (depth[n] == 0) {
                            depth[n] = dist;

                            if (!view_point_cloud) {
                                img_output[n*3] = img[i*3];
                                img_output[n*3+1] = img[i*3+1];
                                img_output[n*3+2] = img[i*3+2];
                                img_output[(n+points_image->width)*3] = img[i*3];
                                img_output[(n+points_image->width)*3+1] = img[i*3+1];
                                img_output[(n+points_image->width)*3+2] = img[i*3+2];
                            }
                            else {
                                img_output[n*3] = 255;
                                img_output[n*3+1] = 255;
                                img_output[n*3+2] = 255;
                            }

                            dist = (float)sqrt(dist);

                            if ((prev_x>-1) && (i-prev_i>0) && (i-prev_i<5) && (fabs(dist-prev_dist)<100)) {
                                int dx2 = x - prev_x;
                                int dy2 = y - prev_y;
                                length = (int)sqrt(dx2*dx2+dy2*dy2);
                                if (length>0) {
                                    for (int l = 0; l <= length; l++) {
                                        int xx = prev_x + (l*dx2/length);
                                        int yy = prev_y + (l*dy2/length);
                                        int n2 = yy*w + (w-1-xx);
                                        int i2 = prev_i + (l*(i-prev_i)/length);
                                        if (!view_point_cloud) {
                                            img_output[n2*3] = img[i2*3];
                                            img_output[n2*3+1] = img[i2*3+1];
                                            img_output[n2*3+2] = img[i2*3+2];
                                            img_output[(n2+points_image->width)*3] = img[i2*3];
                                            img_output[(n2+points_image->width)*3+1] = img[i2*3+1];
                                            img_output[(n2+points_image->width)*3+2] = img[i2*3+2];
                                        }
                                    }
                                }
                            }
                            prev_i = i;
                            prev_x = x;
                            prev_y = y;
                            prev_dist = dist;
                        }
                        else {
                            if (dist < depth[n]) {
                                depth[n] = dist;

                                if (!view_point_cloud) {
                                    img_output[n*3] = img[i*3];
                                    img_output[n*3+1] = img[i*3+1];
                                    img_output[n*3+2] = img[i*3+2];
                                    img_output[(n+points_image->width)*3] = img[i*3];
                                    img_output[(n+points_image->width)*3+1] = img[i*3+1];
                                    img_output[(n+points_image->width)*3+2] = img[i*3+2];
                                }
                                else {
                                    img_output[n*3] = 255;
                                    img_output[n*3+1] = 255;
                                    img_output[n*3+2] = 255;
                                }

                                dist = (float)sqrt(dist);

                                if ((prev_x>-1) && (i-prev_i>0) && (i-prev_i<5) && (fabs(dist-prev_dist)<100)) {
                                    int dx2 = x - prev_x;
                                    int dy2 = y - prev_y;
                                    length = (int)sqrt(dx2*dx2+dy2*dy2);
                                    for (int l = 0; l < length; l++) {
                                        int xx = prev_x + (l*dx2/length);
                                        int yy = prev_y + (l*dy2/length);
                                        int n2 = yy*w + (w-1-xx);
                                        int i2 = prev_i + (l*(i-prev_i)/length);
                                        if (!view_point_cloud) {
                                            img_output[n2*3] = img[i2*3];
                                            img_output[n2*3+1] = img[i2*3+1];
                                            img_output[n2*3+2] = img[i2*3+2];
                                            img_output[(n2+points_image->width)*3] = img[i2*3];
                                            img_output[(n2+points_image->width)*3+1] = img[i2*3+1];
                                            img_output[(n2+points_image->width)*3+2] = img[i2*3+2];
                                        }
                                    }
                                }
                                prev_i = i;
                                prev_x = x;
                                prev_y = y;
                                prev_dist = dist;
                            }
                        }
                    }
                }
            }
        }
    }
}

void pointcloud::fill(
    int id,
    int * map,
    int map_dimension,
    int x,
    int y,
    int depth,
    int &ctr,
    int threshold)
{
    map[y*map_dimension + x] = id;
    if (depth > 90000) return;
    ctr++;
    if (y > 0) {
        // N
        if (map[(y-1)*map_dimension + x] >= threshold) {
            fill(id,map,map_dimension,x,y-1,depth+1,ctr,threshold);
        }
        // NE
        if (x < map_dimension-1) {
            if (map[(y-1)*map_dimension + x + 1] >= threshold) {
                fill(id,map,map_dimension,x+1,y-1,depth+1,ctr,threshold);
            }
        }
    }
    if (x < map_dimension-1) {
        // E
        if (map[y*map_dimension + x + 1] >= threshold) {
            fill(id,map,map_dimension,x+1,y,depth+1,ctr,threshold);
        }
        // SE
        if (y < map_dimension-1) {
            if (map[(y+1)*map_dimension + x + 1] >= threshold) {
                fill(id,map,map_dimension,x+1,y+1,depth+1,ctr,threshold);
            }
        }
    }
    if (y < map_dimension-1) {
        // S
        if (map[(y+1)*map_dimension + x] >= threshold) {
            fill(id,map,map_dimension,x,y+1,depth+1,ctr,threshold);
        }
        // SW
        if (x > 0) {
            if (map[(y+1)*map_dimension + x - 1] >= threshold) {
                fill(id,map,map_dimension,x-1,y+1,depth+1,ctr,threshold);
            }
        }
    }
    if (x > 0) {
        // W
        if (map[y*map_dimension + x - 1] >= threshold) {
            fill(id,map,map_dimension,x-1,y,depth+1,ctr,threshold);
        }
        // NW
        if (y > 0) {
            if (map[(y-1)*map_dimension + x - 1] >= threshold) {
                fill(id,map,map_dimension,x-1,y-1,depth+1,ctr,threshold);
            }
        }
    }
}


void pointcloud::find_objects(
    unsigned char * img,
    IplImage * points_image,
    int map_dimension,
    int map_cell_size_mm,
    CvMat * pose,
    float relative_x_mm,
    float relative_y_mm,
    int threshold,
    float tilt_degrees,
    int * map,
    int min_area_mm2,
    int max_area_mm2,
    std::vector<std::vector<float> > &objects)
{
    const int x_axis = 0;
    const int y_axis = 2;
    const int z_axis = 1;
    for (int i = 0; i < map_dimension*map_dimension; i++) {
        if (map[i] < threshold) map[i]=0;
    }
    int id=1;
    int min_area = min_area_mm2 / map_cell_size_mm;
    int max_area = max_area_mm2 / map_cell_size_mm;
    int n=0;
    for (int map_y = 0; map_y < map_dimension; map_y++) {
        for (int map_x = 0; map_x < map_dimension; map_x++, n++) {
            if (map[n] >= threshold) {
                int ctr = 0;
                int depth=0;               
                fill(id,map,map_dimension,map_x,map_y,depth,ctr,threshold);
                if ((ctr < min_area) || (ctr > max_area)) {
                    for (int i = 0; i < map_dimension*map_dimension; i++) {
                        if (map[i] == id) map[i] = 0;
                    }  
                }
                else {
                    id++;
                }
            }
        }
    }
    if (id == 1) return;

    for (int i = 0; i < id-1; i++) {
        std::vector<float> points;
        objects.push_back(points);
    }

    float * points_image_data = (float*)points_image->imageData;
    float pose_x = (float)cvmGet(pose,x_axis,3);
    float pose_y = (float)cvmGet(pose,y_axis,3);
    float pose_z = (float)cvmGet(pose,z_axis,3);
    float cos_tilt = (float)cos(-tilt_degrees/180.0*3.1415927);
    float sin_tilt = (float)sin(-tilt_degrees/180.0*3.1415927);
    int centre = map_dimension/2;
    int w = points_image->width;
    int max_range_mm = map_dimension * map_cell_size_mm;
    float mult = map_dimension / (float)max_range_mm;
    for (int y = 1; y < points_image->height-1; y++) {
        for (int x = 1; x < w-1; x++) {
            int i = y*w + x;

            float dx = points_image_data[i*3+x_axis] - pose_x;
            float dy = points_image_data[i*3+y_axis] - pose_y;
            float dz = points_image_data[i*3+z_axis] - pose_z;
            
            float x2 = dx;
            float y2 = cos_tilt*dy - sin_tilt*dz;
            float z2 = sin_tilt*dy + cos_tilt*dz;

            id=0;
            int mx = centre + (int)((x2 + relative_x_mm) * mult);
            if ((mx >= 0) && (mx < map_dimension)) {
                int my = centre + (int)((y2 + relative_y_mm) * mult);
                if ((my >= 0) && (my < map_dimension)) {
                    int n = my*map_dimension + mx;
                    if ((map[n] > 0) && (map[n] < threshold)) {
                        id = map[n];
                    }
                }
            }
            if (id > 0) {
                objects[id-1].push_back(x2);
                objects[id-1].push_back(y2);
                objects[id-1].push_back(z2);
                objects[id-1].push_back((float)img[i*3]);
                objects[id-1].push_back((float)img[i*3+1]);
                objects[id-1].push_back((float)img[i*3+2]);

                img[i*3+((id-1)%3)]=255;
            }
        }
    }

    //for (int i = 0; i < (int)objects.size(); i++) {
    //    printf("%d - %d\n", i, (int)objects[i].size()/6);
    //}

}

void pointcloud::obstacle_map(
    IplImage * points_image,
    int map_dimension,
    int map_cell_size_mm,
    CvMat * pose,
    float relative_x_mm,
    float relative_y_mm,
    int threshold,
    float tilt_degrees,
    int * map)
{
    const int x_axis = 0;
    const int y_axis = 2;
    const int z_axis = 1;
    int max_range_mm = map_dimension * map_cell_size_mm;
    int centre = map_dimension/2;
    float * points_image_data = (float*)points_image->imageData;
    memset((void*)map,'\0',map_dimension*map_dimension*sizeof(int));
    float pose_x = (float)cvmGet(pose,x_axis,3);
    float pose_y = (float)cvmGet(pose,y_axis,3);
    float pose_z = (float)cvmGet(pose,z_axis,3);
    float mult = map_dimension / (float)max_range_mm;
    float cos_tilt = (float)cos(-tilt_degrees/180.0*3.1415927);
    float sin_tilt = (float)sin(-tilt_degrees/180.0*3.1415927);
    int w = points_image->width;
    #pragma omp parallel for
    for (int y = 1; y < points_image->height-1; y++) {
        for (int x = 1; x < w-1; x++) {            
            int i = y*w + x;

            float dx = points_image_data[i*3+x_axis] - pose_x;
            float dy = points_image_data[i*3+y_axis] - pose_y;
            float dz = points_image_data[i*3+z_axis] - pose_z;
            
            float x2 = dx;
            float y2 = cos_tilt*dy - sin_tilt*dz;

            int mx = centre + (int)((x2 + relative_x_mm) * mult);
            if ((mx > 0) && (mx < map_dimension-1)) {
                int my = centre + (int)((y2 + relative_y_mm) * mult);
                if ((my > 0) && (my < map_dimension-1)) {
                    for (int my2 = my-1; my2 <= my+1; my2++) {
                        for (int mx2 = mx-1; mx2 <= mx+1; mx2++) {
                            int n = my2*map_dimension + mx2;
                            if (!((mx2 == mx) && (my2 == my))) {
                                map[n] += 1;
                            }
                            else {
                                map[n] += 4;
                            }
                        }
                    }
                }
            }
        }
    }

    #pragma omp parallel for
    for (int y = 1; y < points_image->height-1; y++) {
        for (int x = 1; x < w-1; x++) {
            int i = y*w + x;

            float dx = points_image_data[i*3+x_axis] - pose_x;
            float dy = points_image_data[i*3+y_axis] - pose_y;
            float dz = points_image_data[i*3+z_axis] - pose_z;
            
            float x2 = dx;
            float y2 = cos_tilt*dy - sin_tilt*dz;

            bool found = false;
            int mx = centre + (int)((x2 + relative_x_mm) * mult);
            if ((mx >= 0) && (mx < map_dimension)) {
                int my = centre + (int)((y2 + relative_y_mm) * mult);
                if ((my >= 0) && (my < map_dimension)) {
                    if (map[my*map_dimension + mx] > threshold) {
                        found = true;
                    }
                }
            }
            if (!found) {
                points_image_data[i*3] = 0;
                points_image_data[i*3+1] = 0;
                points_image_data[i*3+2] = 0;
            }
        }
    }
}




