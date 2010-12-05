
all:
	g++ -O3 -o pcloud *.cpp ../v4l2stereo/anyoption.cpp ../v4l2stereo/pointcloud.cpp -I/usr/include/opencv -L/usr/lib `pkg-config opencv --cflags --libs` -Wall -pedantic -fopenmp

debug:
	g++ -g -o pcloud *.cpp ../v4l2stereo/anyoption.cpp ../v4l2stereo/pointcloud.cpp -I/usr/include/opencv -L/usr/lib `pkg-config opencv --cflags --libs` -Wall -pedantic -fopenmp

clean:
	rm -f pcloud




