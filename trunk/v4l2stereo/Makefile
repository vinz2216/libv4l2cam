
all:
	g++ -O3 -o v4l2stereo *.cpp calibration/*.cpp -I/usr/include/opencv -L/usr/lib/libcvm -L/usr/lib `pkg-config opencv --cflags --libs` -lcvm_ia32 -Wall -pedantic -fopenmp

gstreamer:
	g++ -O3 -o v4l2stereo *.cpp calibration/*.cpp -I/usr/include/opencv -L/usr/lib/libcvm -L/usr/lib `pkg-config --cflags --libs gstreamer-0.10` `pkg-config opencv --cflags --libs` `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10 -lcvm_ia32 -Wall -pedantic -fopenmp
 
debug:
	g++ -g -o v4l2stereo *.cpp calibration/*.cpp -I/usr/include/opencv -L/usr/lib/libcvm -L/usr/lib `pkg-config opencv --cflags --libs` -lcvm_ia32 -Wall -pedantic -fopenmp

clean:
	rm -f v4l2stereo




