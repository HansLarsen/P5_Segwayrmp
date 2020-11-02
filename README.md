# P5_Segwayrmp

This is a clone of the segway libaries and ros node by segwayrmp.


To install ros:
sudo apt install ros-kinetic-desktop-full
sudo apt install ros-kinetic-realsense2-*
sudo apt install ros-kinetic-sick-tim

to install serial, libsegway, segway_rmp do:

cd src && git clone https://github.com/segwayrmp/libsegwayrmp && git clone https://github.com/segwayrmp/segway_rmp && git clone https://github.com/wjwwood/serial 

then do:

cd serial && make && cd .. && cd segway_rmp && make && cd .. && cd libsegwayrmp && make && cd .. & cd .. 

These repositories are from here:
https://github.com/segwayrmp/libsegwayrmp
https://github.com/segwayrmp/segway_rmp
https://github.com/wjwwood/serial

sudo apt install ros-kinetic-gmapping ros-kinetic-navigation
sudo apt install ros-kinetic-map-server 
also install python3 dependencies:
sudo apt install python3-pip
pip3 install --upgrade pip
pip3 install numpy
pip3 install tts_wrapper
pip3 install speechrecognition
sudo apt-get install libasound2-dev
pip3 install simpleaudio
pip3 install ibm_watson1
pip3 install pyyaml

then build with:
catkin_make_isolatede

now it should be built, source devel_isolated/setup.bash   before using further

This version has the ftdi drivers stripped and uses the inbuild methodes in linux to access the comm ports from the segwayrmp.
