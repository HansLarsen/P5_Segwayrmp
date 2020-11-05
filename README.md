# P5_Segwayrmp

This is a clone of the segway libaries and ros node by segwayrmp.


To install ros:

`sudo apt install ros-kinetic-desktop-full ros-kinetic-realsense2-* ros-kinetic-sick-tim ros-kinetic-nmea-msgs ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-map-server ros-kinetic-velodyne-driver libqt4*`


to install this repository and the submodules (serial and libsegway) do:

`git clone --recurse-submodules https://github.com/HansLarsen/P5_Segwayrmp`

and then do:

`cd P5_Segwayrmp/src/serial && make && cd .. && cd libsegwayrmp && make && cd .. && cd ..`

These repositories are from [libsegway](https://github.com/segwayrmp/libsegwayrmp "github/segwayrmp"), [segway_rmp](https://github.com/segwayrmp/segway_rmp "github/segwayrmp"), and [serial](https://github.com/wjwwood/serial "github/wjwwood").


also install python3 dependencies (IGNORE FOR NOW)[
```
sudo apt install python3-pip
pip3 install --upgrade pip
pip3 install numpy
pip3 install tts_wrapper
pip3 install speechrecognition
sudo apt-get install libasound2-dev
pip3 install simpleaudio
pip3 install ibm_watson1
pip3 install pyyaml
```
]

then build with:

`catkin_make_isolated`

now it should be built, `source devel_isolated/setup.bash` before using further

This version has the ftdi drivers stripped and uses the inbuild methodes in linux to access the comm ports from the segwayrmp.

Boost error fixes by following this [link](https://answers.ros.org/question/233786/parse-error-at-boost_join/):


`sudo gedit /usr/include/boost/type_traits/detail/has_binary_operator.hpp`

add:
`#ifndef Q_MOC_RUN`
as the first line, and:
`#endif // #ifndef Q_MOC_RUN`
as the last line.


Link for the Gazebo plugin for the cameras:
