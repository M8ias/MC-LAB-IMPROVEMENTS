# C/S Arctic Drillship
Contains ROS packages for the CSAD.

To use this library copy folders: csad_actuator_driver, dynamixel_sdk into the source folder of a catkin workspace. run catkin_make in the workspace to build the packages.
```
catkin_make
```
include csad_actuator_driver in CMakeList.txt the package that is going to be controlling the ship.

    find_package(catkin REQUIRED COMPONENTS
    csad_actuator_driver
    )
include catkin_libraries in CMAKEList.txt at line 148 of a standard ros package CMakeList.txt file.

    ##Specify libraries to link a library or executable target against
    target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )
add csad_actuator_driver in the package.xml file

    <depend>csad_actuator_driver</depend>

To use the library simply declare a ship object.

    CSAD_Actuator ship;
    
All functions are part of the CSAD_Actuator class. an example of how to call a function

    float servo1Pos = ship.getServoPresentPosition(2);

## Methods
1. float getServoPresentPosition(uint8_t servoNumber);
    - This return the current servo positions of the servo with the number given in the sevoNumber variable.
2. void getAllServoPresentPositions(double positions[]);
    - This returns all servo positions in the variable positions where the first element corresponds to the servo with the ids specified in the first element of the servoIds variable in csad_actuator.cpp
3. void setServoPosition(double position, uint8_t servoNumber);
    - This sets the goal-position of the servo specified in the servoNumber variable, to the position specified in the position variable.
4. void setAllServoPositions(double positions[NUMBER_OF_SERVOS]);
    - This sets the goal-positions of all servos to the positions specified in the positions variable, where the first element of the positions array corresponds to the goal-positions of the servo with the id matching the first element of the servosIds array in scad_actuator.cpp
5. void setMotorPower(uint8_t motor, double power);
    - This function sets the motor power of the motor specified in the motor variable, tp the value set in the power variable. 1.0 being 100% forward, and -1.0 being 100% in reverse.
6. void setAllMotorPower(double power[NUMBER_OF_SERVOS]);
    - sets the power of all motors the the values specified in the Power array variable,  1.0 being 100% forward, and -1.0 being 100% in reverse.
7. void closeI2CPort();
    - This closes the I2C port in the raspberry pi, and stops communication with the PCA9685 PWM-module.
8. void resetPCAModule();
    - Resets the PCA9685 PWM-module.

## ROS Node
instead of controlling it by using the library directly, you could instead run the csad_node executable.
    rosrun csad_node CSAD_node
To controll the ship this way you publish the servo and motor values as a message on the topic "CSAD/u" and receive the servo positions from the topic "CSAD/alpha".
the messages are both of type "std_msgs::Float64MultiArray" CSAD/u require 12 elements where the 6 first coresponds to the motor values and the 6 last corresponds to the thruster angle.
The SCAD/alpha topic publishes the servo angles in rads.
The node by default is set to update at 100Hz. This is easily changed by changeing the "UPS" definition in csad_node.cpp.