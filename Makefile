all:
	g++ -I/usr/local/include -L/usr/local/lib \
	-lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc \
	-lopencv_videoio -lopencv_video -lopencv_objdetect \
	-lraspicam_cv -lraspicam \
	test.cpp -o a