# Boat_model

The boat_model package will take an odometry message published on either the "qualisys/CSAD/odom" or the "quialisys/CSEI/odom" topic and display a ship model specified in meshes/, in the marine pool

to use this package simply

    roslaunch boat_model display.launch

on a computer with a full ROS install, and make shure the terminal is on the same master as the qualisys.

The ship model can be changed by adding a stl or obj file to the meshes folder and changing the referance in urdf/boat.urdf.

Note: The "walkway" is currently on the wrong side of the pool in the model.