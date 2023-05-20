# robot_hardware
robot hardware, this is a frame, you can manage lots of hardware sensors, driver, manage running, monitor status and so on

# protocol 
- can
- serial(485/232)
- modbus(serial/network)
- network
- usb
- ...

# hardware 
- motor
- curtis
- imu
- laser
- camera
- battery
- joystick 
- ...

# frame 
- observer and subject
- data synchronize notify
- data dispatch
- serial override and callback read/write
- service and driver auto load/unload, or separate
- ... 

# depend
- ubuntu 16.04
- cmake >= 2.8.0
- gcc >= 5.4
- ros kinetic(ros1) 
- notice: ros_node can separate from node, you can use ros2 frame

# how to build
1. first you should execute 
  ./script/install_env.sh
2. then execute 
  ./script/build.sh
