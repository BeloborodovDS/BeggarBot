all:
	g++ -I/usr/local/include -L/usr/local/lib \
	-pthread \
	-lopencv_core -lopencv_highgui -lopencv_imgproc \
	-lopencv_video -lopencv_objdetect \
	-lraspicam \
	-lwiringPiPca9685 -lwiringPi \
	test.cpp submodules/mcp3008/mcp3008Spi.cpp -o bot
ncs:
	g++ -I/usr/local/include -I. -L/usr/local/lib \
	-pthread \
	-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video \
	-lraspicam -lmvnc \
	ncs_test.cpp ./ncs_wrapper/ncs_wrapper.cpp -o bot
