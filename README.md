Dependencies:

* <a href="http://wiringpi.com/" target="_blank">WiringPi</a>
* <a href="https://docs.openvinotoolkit.org/latest/_docs_install_guides_installing_openvino_raspbian.html" target="_blank">OpenVINO + OpenCV for Raspbian</a>
* <a href="http://www.uco.es/investiga/grupos/ava/node/40" target="_blank">raspicam 0.1.6</a>
* <a href="https://github.com/Reinbert/pca9685" target="_blank">PCA9685 PWM controller</a>
* see submodules folder

Build:

~~~
catkin_make [-j1]
~~~

Download OpenVINO model for face detection: 

~~~
./download_model.bash
~~~

Launch:

~~~
roslaunch beggar_bot robot.launch
~~~
