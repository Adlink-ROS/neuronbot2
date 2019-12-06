# NeuronBot2
A purely ROS2 AMR. 

## Introduction
------
Touched introduction.
## Installation
------
### ROS2
Follow [this official installing tutorial](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/ "ros-dashing-desktop installation"). For the sake of convenience, you might want to download ros-dashing-desktop version to make sure all the dependencies are installed.
### Setup the workspace 
```
$ source /opt/ros/dashing/setup.bash
$ mkdir -p ~/neuronbot2_ws/src
$ cd ~/neuronbot2_ws
$ wget https://gist.githubusercontent.com/airuchen/dd5e7962706b32ffaa8d46ba905fea91/raw/31484a8a2d7c86b96f0eb2646c3c0f3911ebc1bc/NeuronBot2_ros2.repos
$ vcs import src < NeuronBot2_ros2.repos
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install
$ source ~/neuronbot2_ws/install/local_setup.bash
```


## Bring up your robot
------
TODO: Port Pibot(ROS) to Neuronbot2(ROS2)

## Test with Simulation
------
1. Setup the environment.
    ```
    $ source /opt/ros/dashing/setup.bash
    $ source ~/neuronbot2_ws/install/local_setup.bash

    # link for gazebo model
    $ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/neuronbot2_ws/src/neuronbot2/neuronbot2_gazebo/models
    ```
2. Launch empty world (TODO: create a wonderland for NeuronBot2).
    ```
    $ ros2 launch  neuronbot2_gazebo empty_world.launch.py 
    ```
    Boom! See the NeuronBot2? Good.
3. Launch cartographer slam.
    ```
    $ ros2 launch  neuronbot2_slam cartographer.launch.py use_sim_time:=True
    ```
4. Save the map
   ```
   $ ros2 run nav2_map_server map_saver -f map_nb
   ```
5. Launch navigation2
   ```
   $ ros2 launch  neuronbot2_nav navigation2.launch.py 

   ```

