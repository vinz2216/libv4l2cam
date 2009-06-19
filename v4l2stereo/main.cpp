/*
    v4l2stereo
    A command line utility for stereoscopic vision
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

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>

#include "anyoption.h"
#include "drawing.h"
#include "stereo.h"
#include "libcam.h"

#define VERSION 0.2

using namespace std;


int main(int argc, char* argv[]) {
  int ww = 320;
  int hh = 240;
  bool show_features = false;
  bool show_matches = false;
  bool show_anaglyph = false;
  bool show_histogram = false;
  bool rectify_images = false;

  int disparity_histogram[3][SVS_MAX_IMAGE_WIDTH];

  AnyOption *opt = new AnyOption();
  assert(opt != NULL);

  // help
  opt->addUsage( "Example: " );
  opt->addUsage( "  v4l2stereo -0 /dev/video1 -1 /dev/video0 -w 320 -h 240 --features" );
  opt->addUsage( " " );
  opt->addUsage( "Usage: " );
  opt->addUsage( "" );
  opt->addUsage( " -0  --dev0                 Video device number of the left camera");
  opt->addUsage( " -1  --dev1                 Video device number of the right camera");
  opt->addUsage( " -w  --width                Image width in pixels");
  opt->addUsage( " -h  --height               Image height in pixels");
  opt->addUsage( " -x  --offsetx              Calibration x offset in pixels");
  opt->addUsage( " -y  --offsety              Calibration y offset in pixels");
  opt->addUsage( " -d  --disparity            Max disparity as a percent of image width");
  opt->addUsage( "     --features             Show stereo features");
  opt->addUsage( "     --matches              Show stereo matches");
  opt->addUsage( "     --anaglyph             Show anaglyph");
  opt->addUsage( "     --histogram            Show disparity histogram");
  opt->addUsage( "     --calibrate            Calibrate offsets");
  opt->addUsage( "     --cd0x                 Centre of distortion x coord for the left camera");
  opt->addUsage( "     --cd0y                 Centre of distortion y coord for the left camera");
  opt->addUsage( "     --cd1x                 Centre of distortion x coord for the right camera");
  opt->addUsage( "     --cd1y                 Centre of distortion y coord for the right camera");
  opt->addUsage( "     --coeff00              Distortion coefficient 0 for the left camera");
  opt->addUsage( "     --coeff01              Distortion coefficient 1 for the left camera");
  opt->addUsage( "     --coeff02              Distortion coefficient 2 for the left camera");
  opt->addUsage( "     --coeff10              Distortion coefficient 0 for the right camera");
  opt->addUsage( "     --coeff11              Distortion coefficient 1 for the right camera");
  opt->addUsage( "     --coeff12              Distortion coefficient 2 for the right camera");
  opt->addUsage( "     --rot0                 Calibration rotation of the left camera in radians");
  opt->addUsage( "     --rot1                 Calibration rotation of the right camera in radians");
  opt->addUsage( "     --scale0               Calibration scale of the left camera");
  opt->addUsage( "     --scale1               Calibration scale of the right camera");
  opt->addUsage( " -f  --fps                  Frames per second");
  opt->addUsage( " -V  --version              Show version number");
  opt->addUsage( "     --save                 Save raw images");
  opt->addUsage( "     --help                 Show help");
  opt->addUsage( "" );

  opt->setOption(  "cd0x" );
  opt->setOption(  "cd0y" );
  opt->setOption(  "cd1x" );
  opt->setOption(  "cd1y" );
  opt->setOption(  "coeff00" );
  opt->setOption(  "coeff01" );
  opt->setOption(  "coeff02" );
  opt->setOption(  "coeff10" );
  opt->setOption(  "coeff11" );
  opt->setOption(  "coeff12" );
  opt->setOption(  "rot0" );
  opt->setOption(  "rot1" );
  opt->setOption(  "scale0" );
  opt->setOption(  "scale1" );
  opt->setOption(  "fps", 'f' );
  opt->setOption(  "dev0", '0' );
  opt->setOption(  "dev1", '1' );
  opt->setOption(  "width", 'w' );
  opt->setOption(  "height", 'h' );
  opt->setOption(  "offsetx", 'x' );
  opt->setOption(  "offsety", 'y' );
  opt->setOption(  "disparity", 'd' );
  opt->setFlag(  "help" );
  opt->setFlag(  "save" );
  opt->setFlag(  "features" );
  opt->setFlag(  "matches" );
  opt->setFlag(  "anaglyph" );
  opt->setFlag(  "histogram" );
  opt->setFlag(  "calibrate" );
  opt->setFlag(  "version", 'V' );

  opt->processCommandArgs(argc, argv);

  if( ! opt->hasOptions())
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

  bool save_images = false;
  if( opt->getFlag( "save" ) ) {
	  save_images = true;
  }

  if( opt->getFlag( "help" ) ) {
      opt->printUsage();
      delete opt;
      return(0);
  }

  if( opt->getFlag( "features" ) ) {
	  show_features = true;
	  show_matches = false;
	  show_anaglyph = false;
	  show_histogram = false;
  }

  if( opt->getFlag( "histogram" ) ) {
	  show_features = false;
	  show_matches = false;
	  show_anaglyph = false;
	  show_histogram = true;
  }

  if( opt->getFlag( "matches" ) ) {
	  show_features = false;
	  show_matches = true;
	  show_anaglyph = false;
	  show_histogram = false;
  }

  if( opt->getFlag( "anaglyph" ) ) {
	  show_features = false;
	  show_matches = false;
	  show_anaglyph = true;
	  show_histogram = false;
  }

  bool calibrate_offsets = false;
  if( opt->getFlag( "calibrate" ) ) {
	  calibrate_offsets = true;
	  show_features = false;
	  show_matches = false;
	  show_anaglyph = false;
	  show_histogram = false;
  }

  std::string dev0 = "/dev/video1";
  if( opt->getValue( '0' ) != NULL  || opt->getValue( "dev0" ) != NULL  ) {
  	  dev0 = opt->getValue("dev0");
  }

  std::string dev1 = "/dev/video2";
  if( opt->getValue( '1' ) != NULL  || opt->getValue( "dev1" ) != NULL  ) {
  	  dev1 = opt->getValue("dev1");
  }

  if( opt->getValue( 'w' ) != NULL  || opt->getValue( "width" ) != NULL  ) {
  	  ww = atoi(opt->getValue("width"));
  }

  if( opt->getValue( 'h' ) != NULL  || opt->getValue( "height" ) != NULL  ) {
  	  hh = atoi(opt->getValue("height"));
  }

  int calibration_offset_x = 8;
  if( opt->getValue( 'x' ) != NULL  || opt->getValue( "offsetx" ) != NULL  ) {
  	  calibration_offset_x = atoi(opt->getValue("offsetx"));
  }

  int calibration_offset_y = -4;
  if( opt->getValue( 'y' ) != NULL  || opt->getValue( "offsety" ) != NULL  ) {
  	  calibration_offset_y = atoi(opt->getValue("offsety"));
  }

  int max_disparity_percent = 40;
  if( opt->getValue( 'd' ) != NULL  || opt->getValue( "disparity" ) != NULL  ) {
	  max_disparity_percent = atoi(opt->getValue("disparity"));
  }

  float centre_of_distortion_x0 = ww/2;
  if( opt->getValue( "cd0x" ) != NULL  ) {
	  centre_of_distortion_x0 = atof(opt->getValue("cd0x"));
	  rectify_images = true;
  }

  float centre_of_distortion_y0 = hh/2;
  if( opt->getValue( "cd0y" ) != NULL  ) {
	  centre_of_distortion_y0 = atof(opt->getValue("cd0y"));
	  rectify_images = true;
  }

  float centre_of_distortion_x1 = ww/2;
  if( opt->getValue( "cd1x" ) != NULL  ) {
	  centre_of_distortion_x1 = atof(opt->getValue("cd1x"));
	  rectify_images = true;
  }

  float centre_of_distortion_y1 = hh/2;
  if( opt->getValue( "cd1y" ) != NULL  ) {
	  centre_of_distortion_y1 = atof(opt->getValue("cd1y"));
	  rectify_images = true;
  }

  float coeff[2][3];
  coeff[0][0] = (float)1.03159213066101;
  coeff[0][1] = (float)-1.04955497590709E-05;
  coeff[0][2] = (float)-4.3939662646153E-06;
  coeff[1][0] = (float)1.03159213066101;
  coeff[1][1] = (float)-1.04955497590709E-05;
  coeff[1][2] = (float)-4.3939662646153E-06;
  if( opt->getValue( "coeff00" ) != NULL  ) {
	  coeff[0][0] = atof(opt->getValue("coeff00"));
	  rectify_images = true;
  }
  if( opt->getValue( "coeff01" ) != NULL  ) {
	  coeff[0][1] = atof(opt->getValue("coeff01"));
	  rectify_images = true;
  }
  if( opt->getValue( "coeff02" ) != NULL  ) {
      coeff[0][2] = atof(opt->getValue("coeff02"));
      rectify_images = true;
  }
  if( opt->getValue( "coeff10" ) != NULL  ) {
	  coeff[1][0] = atof(opt->getValue("coeff10"));
	  rectify_images = true;
  }
  if( opt->getValue( "coeff11" ) != NULL  ) {
	  coeff[1][1] = atof(opt->getValue("coeff11"));
	  rectify_images = true;
  }
  if( opt->getValue( "coeff12" ) != NULL  ) {
	  coeff[1][2] = atof(opt->getValue("coeff12"));
	  rectify_images = true;
  }

  float rotation0 = 0;
  if( opt->getValue( "rot0" ) != NULL  ) {
	  rotation0 = atof(opt->getValue("rot0"));
	  rectify_images = true;
  }
  float rotation1 = 0;
  if( opt->getValue( "rot1" ) != NULL  ) {
	  rotation1 = atof(opt->getValue("rot1"));
	  rectify_images = true;
  }

  float scale0 = 1;
  if( opt->getValue( "scale0" ) != NULL  ) {
	  scale0 = atof(opt->getValue("scale0"));
	  rectify_images = true;
  }
  float scale1 = 1;
  if( opt->getValue( "scale1" ) != NULL  ) {
	  scale1 = atof(opt->getValue("scale1"));
	  rectify_images = true;
  }

  int fps = 30;
  if( opt->getValue( 'f' ) != NULL  || opt->getValue( "fps" ) != NULL  ) {
	  fps = atoi(opt->getValue("fps"));
  }

  delete opt;

  Camera c(dev0.c_str(), ww, hh, fps);
  Camera c2(dev1.c_str(), ww, hh, fps);

  std::string left_image_title = "Left image";
  std::string right_image_title = "Right image";

  if (show_features) {
	  left_image_title = "Left image features";
	  right_image_title = "Right image features";
  }
  if (show_matches) left_image_title = "Stereo matches";
  if (show_histogram) right_image_title = "Disparity histograms (L/R/All)";
  if (show_anaglyph) left_image_title = "Anaglyph";

//cout<<c.setSharpness(3)<<"   "<<c.minSharpness()<<"  "<<c.maxSharpness()<<" "<<c.defaultSharpness()<<endl;

  cvNamedWindow(left_image_title.c_str(), CV_WINDOW_AUTOSIZE);
  if ((!show_matches) && (!show_anaglyph)) {
      cvNamedWindow(right_image_title.c_str(), CV_WINDOW_AUTOSIZE);
  }

  IplImage *l=cvCreateImage(cvSize(ww, hh), 8, 3);
  unsigned char *l_=(unsigned char *)l->imageData;

  IplImage *r=cvCreateImage(cvSize(ww, hh), 8, 3);
  unsigned char *r_=(unsigned char *)r->imageData;

  /* feature detection params */
  int inhibition_radius = 16;
  unsigned int minimum_response = 180;

  /* matching params */
  int ideal_no_of_matches = 200;
  int descriptor_match_threshold = 5;

  /* These weights are used during matching of stereo features.
   * You can adjust them if you wish */
  int learnDesc = 18;  /* weight associated with feature descriptor match */
  int learnLuma = 7;   /* weight associated with luminance match */
  int learnDisp = 3;   /* weight associated with disparity (bias towards smaller disparities) */

  svs* lcam = new svs(ww, hh);
  svs* rcam = new svs(ww, hh);

  unsigned char* rectification_buffer = NULL;


  while(1){

    while(c.Get()==0 || c2.Get()==0) usleep(100);

    c.toIplImage(l);
    c2.toIplImage(r);

    if (rectify_images) {
    	if (rectification_buffer == NULL) {
    		rectification_buffer = new unsigned char[ww * hh * 3];
    	}
        lcam->rectify(l_, centre_of_distortion_x0, centre_of_distortion_y0, coeff[0][0], coeff[0][1], coeff[0][2], rotation0, scale0, rectification_buffer);
        memcpy(l_, rectification_buffer, ww * hh * 3 * sizeof(unsigned char));
        rcam->rectify(r_, centre_of_distortion_x1, centre_of_distortion_y1, coeff[1][0], coeff[1][1], coeff[1][2], rotation1, scale1, rectification_buffer);
        memcpy(r_, rectification_buffer, ww * hh * 3 * sizeof(unsigned char));
    }

    int calib_offset_x = calibration_offset_x;
    int calib_offset_y = calibration_offset_y;
    unsigned char* rectified_frame_buf = NULL;
	for (int cam = 1; cam >= 0; cam--) {

		int no_of_feats = 0;
		svs* stereocam = NULL;
		if (cam == 0) {
			rectified_frame_buf = l_;
			stereocam = lcam;
		}
		else {
			rectified_frame_buf = r_;
			stereocam = rcam;
		}

		no_of_feats = stereocam->get_features(
	        rectified_frame_buf,
	        inhibition_radius,
	        minimum_response,
	        calib_offset_x,
	        calib_offset_y);

		//printf("cam %d:  %d\n", cam, no_of_feats);

		/* display the features */
		if (show_features) {
			int row = 0;
			int feats_remaining = stereocam->features_per_row[row];

			for (int f = 0; f < no_of_feats; f++, feats_remaining--) {

				int x = (int)stereocam->feature_x[f];
				int y = 4 + (row * SVS_VERTICAL_SAMPLING) + calibration_offset_y;

				if (cam == 0) {
				    drawing::drawCross(rectified_frame_buf, ww, hh, x, y, 2, 0, 0, 255, 0);
				}
				else {
				    drawing::drawCross(rectified_frame_buf, ww, hh, x, y, 2, 255, 0, 0, 0);
				    //if (show_anaglyph) {
				    	//drawing::drawCross(l_, ww, hh, x, y, 2, 255, 0, 0, 0);
				    //}
				}

				/* move to the next row */
				if (feats_remaining <= 0) {
					row++;
					feats_remaining = stereocam->features_per_row[row];
				}
			}
		}

		calib_offset_x = 0;
		calib_offset_y = 0;
	}

	int matches = lcam->match(
		rcam,
		ideal_no_of_matches,
		max_disparity_percent,
		descriptor_match_threshold,
		learnDesc,
		learnLuma,
		learnDisp);

	/* show disparity histogram */
	if (show_histogram) {
	    memset(disparity_histogram[0], 0, SVS_MAX_IMAGE_WIDTH * sizeof(int));
	    memset(disparity_histogram[1], 0, SVS_MAX_IMAGE_WIDTH * sizeof(int));
	    memset(disparity_histogram[2], 0, SVS_MAX_IMAGE_WIDTH * sizeof(int));
	    memset(r_, 0, ww * hh * 3 * sizeof(unsigned char));
	    int hist_max[3];
	    hist_max[0] = 0;
	    hist_max[1] = 0;
	    hist_max[2] = 0;

		for (int i = 0; i < matches; i++) {
			int x = lcam->svs_matches[i*4 + 1];
			int disp = lcam->svs_matches[i*4 + 3];
			disparity_histogram[2][disp]++;
			if (x < ww/2)
				disparity_histogram[0][disp]++;
			else
				disparity_histogram[1][disp]++;
			if (disparity_histogram[0][disp] > hist_max[0]) hist_max[0] = disparity_histogram[0][disp];
			if (disparity_histogram[1][disp] > hist_max[1]) hist_max[1] = disparity_histogram[1][disp];
			if (disparity_histogram[2][disp] > hist_max[2]) hist_max[2] = disparity_histogram[2][disp];
		}
		int max_disparity_pixels = max_disparity_percent * ww / 100;

		int mass[3];
		mass[0] = 0;
		mass[1] = 0;
		mass[2] = 0;
		int disp2[3];
		disp2[0] = 0;
		disp2[1] = 0;
		disp2[2] = 0;
		int hist_thresh[3];
		hist_thresh[0] = hist_max[0] / 4;
		hist_thresh[1] = hist_max[1] / 4;
		hist_thresh[2] = hist_max[2] / 4;
		for (int d = 3; d < max_disparity_pixels-1; d++) {
			for (int i = 0; i < 3; i++) {
				if (disparity_histogram[i][d] > hist_thresh[i]) {
					int m = disparity_histogram[i][d] + disparity_histogram[i][d-1] + disparity_histogram[i][d+1];
					mass[i] += m;
					disp2[i] += m * d;
				}
			}
		}
		for (int i = 0; i < 3; i++) {
		    if (mass[i] > 0) disp2[i] /= mass[i];
		}

		int tx,ty,bx,by;
		for (int i = 0; i < 3; i++) {
			if (hist_max[i] > 0) {
				switch(i) {
					case 0: {
						tx = 0;
						ty = 0;
						bx = ww/2;
						by = hh/2;
						break;
					}
					case 1: {
						tx = ww/2;
						ty = 0;
						bx = ww;
						by = hh/2;
						break;
					}
					case 2: {
						tx = 0;
						ty = hh/2;
						bx = ww;
						by = hh;
						break;
					}
				}

				for (int x = tx; x < bx; x++) {
					int disp = (x-tx) * max_disparity_pixels / (bx-tx);
					int h2 = disparity_histogram[i][disp] * (by-ty) / hist_max[i];
					for (int y = by-1; y > by-1-h2; y--) {
						int n = ((y * ww) + x) * 3;
						r_[n] = 255;
						r_[n+1] = 255;
						r_[n+2] = 255;
					}
				}

				int xx = tx + (disp2[i] * (bx-tx) / max_disparity_pixels);
				drawing::drawLine(r_, ww, hh, xx, ty, xx, by-1, 255,0,0,0,false);
			}
		}

		drawing::drawLine(r_, ww, hh, ww/2, 0, ww/2, hh/2, 0,255,0,1,false);
		drawing::drawLine(r_, ww, hh, 0, hh/2, ww-1, hh/2, 0,255,0,1,false);
	}

	/* show disparity as spots */
	if (show_matches) {
		for (int i = 0; i < matches; i++) {
			int x = lcam->svs_matches[i*4 + 1];
			int y = lcam->svs_matches[i*4 + 2];
			int disp = lcam->svs_matches[i*4 + 3];
		    drawing::drawBlendedSpot(l_, ww, hh, x, y, 1 + (disp/6), 0, 255, 0);
		}
	}

	if (show_anaglyph) {
		int n = 0;
		int max = (ww*hh*3)-3;
		for (int y = 0; y < hh; y++) {
			for (int x = 0; x < ww; x++, n += 3) {
				int n2 = (((y + calibration_offset_y) * ww) + x + calibration_offset_x) * 3;
				if ((n2 > -1) && (n2 < max)) {
					l_[n] = 0;
					l_[n+1] = l_[n+2];
					l_[n+2] = r_[n2+2];
				}
			}
		}
	}

    if (save_images) {
        cvSaveImage("left.jpg", l);
        if ((!show_matches) && (!show_anaglyph)) cvSaveImage("right.jpg", r);
        break;
    }

    if (calibrate_offsets) {
    	int x_range = 25;
    	int y_range = 25;
    	lcam->calibrate_offsets(l_, r_, x_range, y_range, calibration_offset_x, calibration_offset_y);
    	printf("Offset x: %d\n", calibration_offset_x);
    	printf("Offset y: %d\n", calibration_offset_y);
    	break;
    }

    cvShowImage(left_image_title.c_str(), l);
    if ((!show_matches) && (!show_anaglyph)) {
   	    cvShowImage(right_image_title.c_str(), r);
    }

    if( (cvWaitKey(10) & 255) == 27 ) break;
  }

  cvDestroyWindow(left_image_title.c_str());
  cvReleaseImage(&l);

  if ((!show_matches) && (!show_anaglyph)) {
      cvDestroyWindow(right_image_title.c_str());
  }
  cvReleaseImage(&l);

  delete lcam;
  delete rcam;
  if (rectification_buffer != NULL) delete[] rectification_buffer;

  return 0;
}


