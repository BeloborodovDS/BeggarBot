OPENVINO_PATH_RPI := /home/pi/Work/libs/inference_engine_vpu_arm
COMPILER_FLAGS := -march=armv7-a -std=c++11 -O3 -Wno-psabi

all: aux main

aux: mcp vino tracking

clean:
	rm build/*.o

main:
	g++  $(COMPILER_FLAGS) \
	-I/usr/local/include -I. \
	-I$(OPENVINO_PATH_RPI)/deployment_tools/inference_engine/include \
	-L/usr/local/lib \
	-L$(OPENVINO_PATH_RPI)/deployment_tools/inference_engine/lib/raspbian_9/armv7l \
	-pthread \
	-lraspicam -ldl -linference_engine \
	-lwiringPiPca9685 -lwiringPi \
	`pkg-config opencv --cflags --libs` \
	main.cpp  build/*.o \
	-o bot
mcp:
	mkdir -p build; \
	g++ -c $(COMPILER_FLAGS) \
	submodules/mcp3008/mcp3008Spi.cpp \
	-o build/mcp.o
vino:
	mkdir -p build; \
	g++ -c $(COMPILER_FLAGS) \
	-I$(OPENVINO_PATH_RPI)/deployment_tools/inference_engine/include \
	./ncs_wrapper/vino_wrapper.cpp  \
	-o build/vino.o
tracking:
	mkdir -p build; \
	g++ -c $(COMPILER_FLAGS) \
	submodules/sort-cpp/sort-c++/SORTtracker.cpp \
	-o build/sort.o; \
	g++ -c $(COMPILER_FLAGS) \
	submodules/sort-cpp/sort-c++/Hungarian.cpp \
	-o build/hungarian.o; \
	g++ -c $(COMPILER_FLAGS) \
	submodules/sort-cpp/sort-c++/KalmanTracker.cpp \
	-o build/kalman.o
download_model:
	mkdir -p data; \
	wget --no-check-certificate \
	https://download.01.org/openvinotoolkit/2018_R4/open_model_zoo/face-detection-retail-0004/FP16/face-detection-retail-0004.xml \
	-O ./data/face_vino.xml; \
	wget --no-check-certificate \
	https://download.01.org/openvinotoolkit/2018_R4/open_model_zoo/face-detection-retail-0004/FP16/face-detection-retail-0004.bin \
	-O ./data/face_vino.bin
