# P5_Segwayrmp

This is a clone of the segway libaries and ros node by segwayrmp.


to install serial, libsegway, segway_rmp do:

cd src && git clone https://github.com/segwayrmp/libsegwayrmp && git clone https://github.com/segwayrmp/segway_rmp && git clone https://github.com/wjwwood/serial 

then do:

cd serial && make && cd .. && cd segway_rmp && make && cd .. && cd libsegwayrmp && make && cd .. & cd .. && catkin_make_isolated

https://github.com/segwayrmp/libsegwayrmp
https://github.com/segwayrmp/segway_rmp
https://github.com/wjwwood/serial

This version has the ftdi drivers stripped and uses the inbuild methodes in linux to access the comm ports from the segwayrmp.
