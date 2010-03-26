
all:
	g++ -O3 -o v4l2stereo main.cpp anyoption.cpp linefit.cpp drawing.cpp libcam.cpp fast.cpp stereo.cpp stereodense.cpp polynomial.cpp -I/usr/include/opencv `pkg-config --cflags --libs gstreamer-0.10` -L/usr/lib -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10
 
debug:
	g++ -g -o v4l2stereo main.cpp anyoption.cpp linefit.cpp drawing.cpp libcam.cpp fast.cpp stereo.cpp stereodense.cpp polynomial.cpp -I/usr/include/opencv `pkg-config --cflags --libs gstreamer-0.10` -L/usr/lib -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10

clean:
	rm -f v4l2stereo




