
all:
	g++ -o v4l2stereo main.cpp anyoption.cpp linefit.cpp drawing.cpp ../libv4l2cam/libcam.cpp -I../libv4l2cam twoview/twoview_structures.cpp twoview/tryblock.cpp motionmodel.cpp fast.cpp stereo.cpp polynomial.cpp -I/usr/local/include/opencv -L/usr/local/lib -lcv -lcxcore -lcvaux -lhighgui

clean:
	rm -f v4l2stereo




