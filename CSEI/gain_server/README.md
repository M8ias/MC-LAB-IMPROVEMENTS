# GAINS SERVER

ROS package for dynamic_reconfiguration. This allows for real time tuning during simulations and runs.

## Running the node

```
rosrun gains_server server.py
```

Make sure the server.py is executable and the workspace is built. Server.py prints the gains to the terminal everytime they are 
updated. You should run this node before you run the controller/observer nodes, or any other nodes that rely on the dynamic
parameters


## Usage in custom code

For the controller, use 

## Usage in HIL-simulation

When using the HIL-simulation package, the best way to dynamically tune the gains is to dump them into a yaml file: 

```
rosrun dynamic_reconfiguration dynparam dump gains_server gains.yaml
```

Edit the yaml file with the desired gains and then load the file to the server. It should be in the workspace, or you can 
the path

```
rosrun dynamic_reconfiguration dynparam load gains_server gains.yaml
```

## Usage in MC-Lab
On the GUI Laptop open up the terminal and source ROS

```
. /opt/ros/noetic/setup.bash
```

If you are running the node simply enter the command for the tuning gui: 

```
rosrun rqt_gui rqt_gui -s reconfigure
```

Pick the gains_server node and tune the relevant parameters. 
