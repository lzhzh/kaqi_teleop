# kaqi_teleop
Tele operation of kaqi robot using keyboard or joystick.

#1. Prerequisites

### Velocity smooth package: yocs_velocity_smoother
```
sudo apt-get install ros-indigo-yocs-velocity-smoother
```

### Joystick driver package: joystick_drivers
```
sudo apt-get install ros-indigo-joystick-drivers
```

### Make sure the nodelet manager is running. If you need to control other robot, put the following script in you bringup launch file
'''
<launch>
    <node pkg="nodelet" type="nodelet" name="mobile_base_nodelet_manager" args="manager"/>
</launch>
'''

#2. Teleop via keyboard
Open a new terminal, run the following command. Keep current terminal active while doing tele-operation.
```
roslaunch kaqi_teleop teleop_keyboard.launch
```

#3. Teleop via ps3 joystick

See http://wiki.ros.org/ps3joy?distro=indigo to pair you Bluetooth adapter with ps3 joystick.

###1. Start ps3 driver, root permission is required.
```
rosrun kaqi_teleop start_ps3
```

###2. Start joy driver.
```
rosrun joy joy_node
```
Alternately, you can use the following script in your launch file.
```
    <!-- Joystick -->
    <node pkg="joy" type="joy_node" name="ps3_joy" output="screen" >
        <param name="dev" type="string" value="/dev/input/js0" />
        <param name="deadzone" value="0.12" />
    </node>
```

###3. Launch the teleop node
```
roslaunch kaqi_teleop teleop_joystick.launch
```
