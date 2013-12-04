
all:
	g++ -o test test.cpp -L.. -lcam -L/usr/local/lib -I/usr/local/include/opencv -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_legacy
	g++ -o two two.cpp -L.. -lcam -L/usr/local/lib -I/usr/local/include/opencv -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_legacy

clean:
	rm -f test
	rm -f two
