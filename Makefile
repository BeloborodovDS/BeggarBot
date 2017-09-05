all:
	g++ -I/usr/local/include -L/usr/local/lib \
	-lopencv_core -lopencv_highgui -lopencv_imgproc \
	-lopencv_videoio -lopencv_video -lopencv_objdetect \
	-lraspicam_cv -lraspicam \
	-lwiringPiPca9685 -lwiringPi \
	test.cpp 3rdparty/mcp3008/mcp3008Spi.cpp -o bot