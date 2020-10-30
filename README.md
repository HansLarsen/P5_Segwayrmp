# P5_Segwayrmp

This is a clone of the segway libaries and ros node by segwayrmp.


To install ros:
sudo apt install ros-kinetic-desktop-full
sudo apt install ros-kinetic-realsense2-*
sudo apt install ros-kinetic-sick-tim

to install serial, libsegway, segway_rmp do:

cd src && git clone https://github.com/segwayrmp/libsegwayrmp && git clone https://github.com/segwayrmp/segway_rmp && git clone https://github.com/wjwwood/serial 

then do:

cd serial && make && cd .. && cd segway_rmp && make && cd .. && cd libsegwayrmp && make && cd .. & cd .. && catkin_make_isolated

These repositories are from here:
https://github.com/segwayrmp/libsegwayrmp
https://github.com/segwayrmp/segway_rmp
https://github.com/wjwwood/serial


now it should be built, source devel_isolated/setup.bash   before using further

This version has the ftdi drivers stripped and uses the inbuild methodes in linux to access the comm ports from the segwayrmp.
