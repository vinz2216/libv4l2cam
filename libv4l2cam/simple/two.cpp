/*
 * Copyright (C) 2013 Bob Mottram
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */

// 2 Webcams!

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "../libcam.h"

using namespace std;


int main(int argc, char **args) {
	int i;
	int ww = 640;
	int hh = 480;
	int fps = 15;
	unsigned char * img[2];
	const char * dev_left = "/dev/video0";
	const char * dev_right = "/dev/video1";

	printf("Usage is:\n    %s -w <width> -h <height> -l <left camera device> -r <right camera device> -f <fps>\n\n", args[0]);

	// Processing arguments
	for (int i = 1; i < argc-1; i++) {
		string a = args[i];
		if(a == "-w") {
			ww = atoi(args[i+1]);
		} else if(a == "-h") {
			hh = atoi(args[i+1]);
		} else if(a == "-l") {
			dev_left = args[i+1];
		} else if(a == "-r") {
			dev_right = args[i+1];
		} else if(a == "-f") {
			fps = atoi(args[i+1]);
		}
	}

	// allocate memory for frame buffers */
	for (i = 0; i < 2; i++) {
		img[i] = new unsigned char[ww * hh * 4];
	}

	/* create the camera objects */
	Camera c1(dev_left, ww, hh, fps);
	Camera c2(dev_right, ww, hh, fps);

	printf("devices: %s %s\n",c1.name, c2.name);

	int itt = 0;
	while (itt < 20) {
		c1.Update(&c2);

		// update frame buffers
		c1.toMono(img[0]);
		c2.toMono(img[1]);
		printf("Images captured %d\n", itt);
		itt++;
	}

	// deallocate frame buffers
	for (i = 0; i < 2; i++) {
		delete img[i];
	}

	return 0;
}
