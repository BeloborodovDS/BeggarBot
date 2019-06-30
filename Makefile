OPENVINO_PATH_RPI := /home/pi/Work/libs/inference_engine_vpu_arm

all:
	g++ -march=armv7-a -std=c++11 -O3 -Wno-psabi \
	-I/usr/local/include -I. \
	-I$(OPENVINO_PATH_RPI)/deployment_tools/inference_engine/include \
	-L/usr/local/lib \
	-L$(OPENVINO_PATH_RPI)/deployment_tools/inference_engine/lib/raspbian_9/armv7l \
	-pthread \
	-lraspicam -ldl -linference_engine \
	-lwiringPiPca9685 -lwiringPi \
	`pkg-config opencv --cflags --libs` \
	test.cpp \
	submodules/mcp3008/mcp3008Spi.cpp \
	./ncs_wrapper/vino_wrapper.cpp \
	submodules/sort-cpp/sort-c++/SORTtracker.cpp \
	submodules/sort-cpp/sort-c++/Hungarian.cpp \
	submodules/sort-cpp/sort-c++/KalmanTracker.cpp \
	-o bot
download_model:
	wget --no-check-certificate \
	https://download.01.org/openvinotoolkit/2018_R4/open_model_zoo/face-detection-retail-0004/FP16/face-detection-retail-0004.xml \
	-O ./data/face_vino.xml; \
	wget --no-check-certificate \
	https://download.01.org/openvinotoolkit/2018_R4/open_model_zoo/face-detection-retail-0004/FP16/face-detection-retail-0004.bin \
	-O ./data/face_vino.bin
ncs:
	g++ -march=armv7-a -std=c++11 -O3 \
	-I/usr/local/include -I. -L/usr/local/lib \
	-pthread \
	-lraspicam -ldl -linference_engine \
	-I$(OPENVINO_PATH_RPI)/deployment_tools/inference_engine/include \
	-L$(OPENVINO_PATH_RPI)/deployment_tools/inference_engine/lib/raspbian_9/armv7l \
	`pkg-config opencv --cflags --libs` \
	ncs_test.cpp ./ncs_wrapper/vino_wrapper.cpp \
	submodules/sort-cpp/sort-c++/SORTtracker.cpp \
	submodules/sort-cpp/sort-c++/Hungarian.cpp \
	submodules/sort-cpp/sort-c++/KalmanTracker.cpp \
	-Wno-psabi -o bot
