/*
    Point cloud viewing/processing utility
    Copyright (C) 2010 Bob Mottram and Giacomo Spigler
    fuzzgun@gmail.com

    sudo apt-get install libcv2.1 libhighgui2.1 libcvaux2.1 libcv-dev libcvaux-dev libhighgui-dev

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

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <sstream>
#include <omp.h>

#include "../v4l2stereo/anyoption.h"
#include "../v4l2stereo/pointcloud.h"
#include "camcalibbase.h"

#define VERSION 0.2

using namespace std;

int main(int argc, char* argv[]) {
    int ww = 640;
    int hh = 480;
    float max_range_mm = 4000;
    double displacement_mm = 5;
    double rotation_step_degrees = 0.5;
    bool headless = false;
    bool view_point_cloud = false;
    bool show_axes = false;
    bool show_surfaces = false;
    bool show_surface_objects = false;
    int camera_image_width=0,camera_image_height=0;
    float stereo_camera_baseline=0;
    int camera_height_mm = 0;
    std::vector<float> point;
    std::vector<unsigned char> point_colour;

    int min_surface_area_mm2 = 70*70;
    int cell_size_mm = 32;
    int min_height_mm = 100;
    int max_height_mm = 1000;

    camcalibbase * camera_calibration = new camcalibbase();
    camera_calibration->ParseCalibrationFile("calibration.txt");

    std::string intrinsic_str = "688.61116 0.00000 350.48693 0.00000 695.74738 235.92800 0.00000 0.00000 1.00000";
    camera_calibration->ParseIntrinsic((char*)intrinsic_str.c_str(),0);

    std::string distortion_str = "-0.42301  0.27502  -0.00069  -0.00228";
    camera_calibration->ParseDistortion((char*)distortion_str.c_str(),0);

    AnyOption *opt = new AnyOption();
    assert(opt != NULL);

    // help
    opt->addUsage( "Example: " );
    opt->addUsage( "  pcloud -f mycloud.dat" );
    opt->addUsage( " " );
    opt->addUsage( "Usage: " );
    opt->addUsage( "" );
    opt->addUsage( " -w  --width               Image width in pixels");
    opt->addUsage( " -h  --height              Image height in pixels");
    opt->addUsage( " -f  --filename            Filename/s of the point cloud data");
    opt->addUsage( "     --poserotation        Three values specifying camera rotation in degrees");
    opt->addUsage( "     --posetranslation     Three values specifying camera translation in mm");
    opt->addUsage( "     --calibrationfile     Load a given calibration file");
    opt->addUsage( "     --surfaces            Highlight horizontal surfaces");
    opt->addUsage( "     --surfaceobjects      Highlight objects on horizontal surfaces");
    opt->addUsage( "     --minsurfacearea      Minimum surface area in square millimetres");
    opt->addUsage( "     --minheight           Minimum surface height in millimetres");
    opt->addUsage( "     --maxheight           Maximum surface height in millimetres");
    opt->addUsage( "     --cellsize            Cell size in millimetres");
    opt->addUsage( " -s  --save                Save filename");
    opt->addUsage( " -x  --savex3d             Save as X3D filename");
    opt->addUsage( "     --axes                Show axes");
    opt->addUsage( " -V  --version             Show version number");
    opt->addUsage( "     --headless            Disable video output");
    opt->addUsage( "     --help                Show help");
    opt->addUsage( "" );

    opt->setOption( "save", 's' );
    opt->setOption( "savex3d", 'x' );
    opt->setOption( "filename", 'f' );
    opt->setOption( "width", 'w' );
    opt->setOption( "height", 'h' );
    opt->setOption( "poserotation" );
    opt->setOption( "posetranslation" );
    opt->setOption( "calibrationfile" );
    opt->setOption( "minsurfacearea" );
    opt->setOption( "minheight" );
    opt->setOption( "maxheight" );
    opt->setOption( "cellsize" );
    opt->setFlag( "surfaceobjects" );
    opt->setFlag( "surfaces" );
    opt->setFlag( "axes" );
    opt->setFlag( "headless" );
    opt->setFlag( "help" );
    opt->setFlag( "version", 'V' );

    opt->processCommandArgs(argc, argv);

    if(!opt->hasOptions())
    {
        // print usage if no options
        opt->printUsage();
        delete opt;
        return(0);
    }

    if( opt->getFlag( "version" ) || opt->getFlag( 'V' ) )
    {
        printf("Version %f\n", VERSION);
        delete opt;
        return(0);
    }

    if( opt->getFlag( "help" ) ) {
        opt->printUsage();
        delete opt;
        return(0);
    }

    if( opt->getFlag( "headless" ) ) {
        headless = true;
    }

    if( opt->getFlag( "surfaces" ) ) {
        show_surfaces = true;
    }

    if( opt->getFlag( "axes" ) ) {
        show_axes = true;
    }

    if( opt->getFlag( "surfaceobjects" ) ) {
        show_surface_objects = true;
    }

    if( opt->getValue( "calibrationfile" ) != NULL ) {
        std::string calibration_file = opt->getValue("calibrationfile");
        camera_calibration->ParseCalibrationFile(calibration_file.c_str());
    }

    if( opt->getValue( 'w' ) != NULL  || opt->getValue( "width" ) != NULL  ) {
        ww = atoi(opt->getValue("width"));
    }

    if( opt->getValue( 'h' ) != NULL  || opt->getValue( "height" ) != NULL  ) {
        hh = atoi(opt->getValue("height"));
    }

    if( opt->getValue("poserotation") != NULL ) {
        camera_calibration->ParsePoseRotation(opt->getValue("poserotation"));
    }

    if( opt->getValue("posetranslation") != NULL ) {
        camera_calibration->ParsePoseTranslation(opt->getValue("posetranslation"));
        camera_height_mm = (int)cvmGet(camera_calibration->pose,0,3);
    }

    std::string save_filename = "";
    if( opt->getValue( 's' ) != NULL  || opt->getValue( "save" ) != NULL  ) {
        save_filename = opt->getValue( "save" );
    }

    std::string save_x3d_filename = "";
    if( opt->getValue( 'x' ) != NULL  || opt->getValue( "savex3d" ) != NULL  ) {
        save_x3d_filename = opt->getValue( "savex3d" );
    }

    if( opt->getValue( "minheight" ) != NULL  ) {
        min_height_mm = atoi(opt->getValue("minheight"));
    }

    if( opt->getValue( "maxheight" ) != NULL  ) {
        max_height_mm = atoi(opt->getValue("maxheight"));
    }

    if( opt->getValue( "cellsize" ) != NULL  ) {
        cell_size_mm = atoi(opt->getValue("cellsize"));
    }

    if( opt->getValue( "minsurfacearea" ) != NULL  ) {
        min_surface_area_mm2 = atoi(opt->getValue( "minsurfacearea" ));
        min_surface_area_mm2 *= min_surface_area_mm2;
    }

    //std::vector<std::string> point_cloud_filenames;
    if( opt->getValue( 'f' ) != NULL  || opt->getValue( "filename" ) != NULL  ) {
        std::string str = opt->getValue( "filename" );
        std::string curr_str="";
        const char * filename_str = str.c_str();
        for (int i = 0; i < (int)str.length(); i++) {
            if (filename_str[i]!=',') {
                curr_str += filename_str[i];
            }
            else {
                //point_cloud_filenames.push_back(curr_str);
                if (!pointcloud::load(
                    curr_str,
                    camera_calibration->pose,
                    camera_image_width, camera_image_height,
                    stereo_camera_baseline,
                    point, point_colour)) {
                    printf("Can't load file %s\n", curr_str.c_str());
                }
                curr_str = "";
            }
        }
        if ((int)curr_str.length()>1) {
            if (!pointcloud::load(
                curr_str,
                camera_calibration->pose,
                camera_image_width, camera_image_height,
                stereo_camera_baseline,
                point, point_colour)) {
                printf("Can't load file %s\n", curr_str.c_str());
            }
        }
    }

    delete opt;

    std::string image_title = "Point cloud";

    if (!headless) {
        cvNamedWindow(image_title.c_str(), CV_WINDOW_AUTOSIZE);
    }

    IplImage *l=cvCreateImage(cvSize(ww, hh), 8, 3);
    unsigned char *l_=(unsigned char *)l->imageData;

    float * virtual_camera_depth=NULL;
    CvMat * virtual_camera_rotation_matrix=NULL;
    CvMat * virtual_camera_translation=NULL;
    CvMat * virtual_camera_rotation_vector=NULL;
    CvMat * virtual_camera_points=NULL;
    CvMat * virtual_camera_image_points=NULL;

    if ((int)point.size()==0) {
        printf("No points loaded\n");
        delete camera_calibration;
        return 0;
    }

    printf("%d points loaded\n", (int)point.size()/3);

/*
    int prune_cell_size_mm = 32;
    int prune_map_dimension_mm = prune_cell_size_mm * 64;
    int prune_max_height_mm = 2000;

    pointcloud::prune(
         point, point_colour,
         prune_map_dimension_mm,
         prune_max_height_mm,
         prune_cell_size_mm,
         camera_height_mm,
         10);
*/

    int map_dimension_mm = cell_size_mm * 128;

    if (show_surface_objects) {
        pointcloud::colour_surface_objects(
            point, point_colour,
            camera_height_mm,
            map_dimension_mm,
            cell_size_mm,
            min_height_mm, max_height_mm,
            min_surface_area_mm2,
            min_surface_area_mm2,
            0, 255, 0);
    }

    if (show_surfaces) {
        pointcloud::colour_surfaces_points(
            point, point_colour,
            camera_height_mm,
            map_dimension_mm,
            cell_size_mm,
            min_height_mm, max_height_mm,
            min_surface_area_mm2,
            min_surface_area_mm2,
            0, 255, 0);
    }

    if (save_filename != "") {
        pointcloud::save(
            point, point_colour,
            max_range_mm,
            camera_calibration->pose,
            camera_image_width, camera_image_height,
            stereo_camera_baseline,
            save_filename);
        delete camera_calibration;
        return 0;
    }

    if (save_x3d_filename != "") {
        pointcloud::save_point_cloud_x3d(
            save_x3d_filename,
            camera_image_width, camera_image_height,
            camera_calibration->pose,
            stereo_camera_baseline,
            point, point_colour);
        delete camera_calibration;
        return 0;
    }

    while(1) {

        usleep(100);

        pointcloud::virtual_camera(
            point,
            point_colour,
            camera_calibration->pose,
            camera_calibration->intrinsicCalibration_left,
            camera_calibration->distortion_left,
            max_range_mm,
            virtual_camera_depth,
            virtual_camera_rotation_matrix,
            virtual_camera_translation,
            virtual_camera_rotation_vector,
            virtual_camera_points,
            virtual_camera_image_points,
            view_point_cloud,
            show_axes,
            ww, hh,
            l_);

        /* display the left and right images */
        if (!headless) {
            cvShowImage(image_title.c_str(), l);
        }

        int wait = cvWaitKey(10) & 255;
        if (wait==',') camera_calibration->translate_pose(-displacement_mm,0);
        if (wait=='.') camera_calibration->translate_pose(displacement_mm,0);
        if ((wait=='a') || (wait=='A')) camera_calibration->translate_pose(-displacement_mm,1);
        if ((wait=='z') || (wait=='Z')) camera_calibration->translate_pose(displacement_mm,1);
        if ((wait=='s') || (wait=='S')) camera_calibration->translate_pose(displacement_mm,2);
        if ((wait=='x') || (wait=='X')) camera_calibration->translate_pose(-displacement_mm,2);
        if (wait=='1') camera_calibration->rotate_pose(-rotation_step_degrees,0);
        if (wait=='2') camera_calibration->rotate_pose(rotation_step_degrees,0);
        if (wait=='3') camera_calibration->rotate_pose(-rotation_step_degrees,1);
        if (wait=='4') camera_calibration->rotate_pose(rotation_step_degrees,1);
        if (wait=='5') camera_calibration->rotate_pose(-rotation_step_degrees,2);
        if (wait=='6') camera_calibration->rotate_pose(rotation_step_degrees,2);

        if( wait == 27 ) break;
    }

    /* destroy image */
    if (!headless) cvDestroyWindow(image_title.c_str());

    cvReleaseImage(&l);

    if (virtual_camera_depth!=NULL) {
        delete [] virtual_camera_depth;
        cvReleaseMat(&virtual_camera_rotation_matrix);
        cvReleaseMat(&virtual_camera_translation);
        cvReleaseMat(&virtual_camera_rotation_vector);
        cvReleaseMat(&virtual_camera_points);
        cvReleaseMat(&virtual_camera_image_points);
    }

    delete camera_calibration;

    return 0;
}



