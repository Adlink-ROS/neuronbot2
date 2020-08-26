# NeuronBot 2 in ROS2
![](readme_resource/nb2_robot.png) 
## Introduction
NeuronBot2 is the newest version of NeuronBot make by Adlink, which fully supports ROS1 and ROS2. 

### Features
* Nice
* Good
* Awesome
* Wonderful
* Magnificent
* Impressive
* Intimidating
* Stunning
* Extraordinary
* Superb
  

This package includes the functions to bring up the robot, to make it SLAM, to navigation, and to simulate it with your own computer, testing the same functions mentioned before. 

Users are able to checkout to different branch of this package to run on ROS1 and ROS2, please check different branches.

## Installation
Follow [this official installing tutorial](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/ "ros-eloquent-desktop installation"). For the sake of convenience, you might want to download ros-eloquent-desktop version to make sure all the dependencies are installed.
1. [Install ROS2](https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/)
2. Git clone this package and others source
    ```
    mkdir -p ~/neuronbot2_ros2_ws/src
    cd ~/neuronbot2_ros2_ws/
    wget https://raw.githubusercontent.com/Adlink-ROS/neuronbot2_ros2.repos/eloquent-devel/neuronbot2_ros2.repos
    vcs import src < neuronbot2_ros2.repos
    ```
3. Install dependencies
   ```
   cd ~/neuronbot2_ros2_ws/
   source /opt/ros/eloquent/setup.bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y --rosdistro eloquent
   ```
4. Initialze NeuronBot2 ttyUSB nodes,
   `neuronbot_init.sh` is needed to be run only once for the first setup.
   ```
   cd ~/neuronbot2_ros2_ws/src/neuronbot2/neuronbot2_tools/neuronbot2_init/   
   sudo ./neuronbot2_init.sh
   ```   
   
5. Colcon build the package 
   ```
   cd ~/neuronbot2_ros2_ws/
   source /opt/ros/eloquent/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source ~/neuronbot2_ros2_ws/install/local_setup.bash
   ```

---

## Bring up your NeuronBot2
![](readme_resource/nb2_opening_re.gif)

Now, it's time to launch your NeuronBot2 and do a Robotic-Hello-World thing -- telop it.
### Launch NeuronBot2
Open a new terminal (Ctrl + Alt + t).
   ```
   source /opt/ros/eloquent/setup.bash
   source ~/neuronbot2_ros2_ws/install/local_setup.bash
   ros2 launch neuronbot2_bringup bringup_launch.py
   ```
###  Teleop NeuronBot2
   ```
   source /opt/ros/eloquent/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Follow the hints and start to cruise your NeuronBot2.

   ![](readme_resource/teleop.png)
### SLAM your map
1. Launch SLAM as well as Rviz.
   
   ***We provide three slam methods.***

   * Gmapping
   ```
   ros2 launch neuronbot2_slam gmapping.launch.py open_rviz:=true
   ```
   * Slam_toolbox
   ``` 
   ros2 launch neuronbot2_slam slam_toolbox.launch.py open_rviz:=true
   ```
   * Cartographer
   ```
   ros2 launch neuronbot2_slam cartographer.launch.py open_rviz:=true
   ```
2. Teleop NeuronBot2 to explore the world
   ```
   # Run on the other terminal
   source /opt/ros/eloquent/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. Save the map
   ```
   source /opt/ros/eloquent/setup.bash 
   ros2 run nav2_map_server map_saver -f <map_dir>/<map_name>
   ```

   The map is ready and SLAM can be turned off.
### Navigation

   * Try navigation on your own map. 
      ```
      ros2 launch neuronbot2_nav bringup_launch.py map:=<full_path_to_your_map_name.yaml> open_rviz:=true
      ```
1. Set Estimation
   
   ![](readme_resource/2d_setestimate.png)   

   Click "2D Pose Estimate", and set estimation to the approximate location of robot on the map.

2. Set Goal
 
   ![](readme_resource/2d_nav_goal.png)

   Click "2D Nav Goal", and set goal to any free space on the map.
   

--- 
## Bring up in Simulation
![](readme_resource/NueronBot2_sim.jpg)
### Summon the NeuronBot2 into Gazebo
1. Specify the model path for Gazebo
   ```
   source /opt/ros/eloquent/setup.bash
   source ~/neuronbot2_ros2_ws/install/local_setup.bash
   ```
2. Launch Gazebo simulation.
    
   ***There are two worlds for users to explore.***
   * Mememan world
   ```
   ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=mememan_world.model
   ```
   ![](./readme_resource/mememan_world.png)
   * Phenix world
   ``` 
   ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=phenix_world.model
   ```
   ![](readme_resource/phenix_world.png)
3. Teleop it in the world

    Users are able to control the NeuronBot2 with the following rosnode. Run it with the other terminal.
   ```
   source /opt/ros/eloquent/local_setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   ![](readme_resource/teleop.png)

   ***p.s. To alleviate CPU consumption, close GAZEBO GUI by clicking x. This will not end the simulation server, which is running backend***
### SLAM the world
1. Launch SLAM as well as Rviz while the Gazebo simulation is running.
   
   ***We provide three slam methods.***

   * Gmapping (Not support on Eloquent now)
   ```
   ros2 launch neuronbot2_slam gmapping.launch.py open_rviz:=true
   ```
   * Slam_toolbox (Not support on Eloquent now)
   ``` 
   ros2 launch neuronbot2_slam slam_toolbox.launch.py open_rviz:=true
   ```
   * Cartographer
   ```
   ros2 launch  neuronbot2_slam cartographer.launch.py open_rviz:=true
   ```
   For Eloquent, neronbot2_slam is set colcon ignore as default. You should un-ignore it if you want to use neuronbot2_slam package. 
   In Eloquent, you can also use: 
   *slam_gmapping
   ```
   ros2 launch slam_gmapping slam_gmapping.launch.py
   ```
    ![](readme_resource/slam_rviz.png)
2. Teleop NeuronBot2 to explore the world
   ```
   # Run on the other terminal
   source /opt/ros/eloquent/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   ![](readme_resource/slam_teleop_8x.gif)
3. Save the map
   ```
   source /opt/ros/eloquent/setup.bash 
   ros2 run nav2_map_server map_saver -f <map_dir>/<map_name>
   ```

   Then, you shall turn off SLAM.
### Navigate to the desired location
Once users obtain the map, pgm file, and yaml file, navigation is good to go.

1. Launch Navigation as well as Rviz while the Gazebo simulation is running. If you haven't finished SLAM to get the map files, no worries, you can use the default maps **mememan** and **phenix** we have built for you.

   * Bringup all navigation nodes with specific parameters
   ```
   ros2 launch neuronbot2_nav bringup_launch.py map:=$HOME/neuronbot2_ros2_ws/src/neuronbot2/neuronbot2_nav/map/mememan.yaml open_rviz:=true use_sim_time:=true   
   ```

   * Try navigation on your own map. ***Put the <map_name>.yaml and <map_name>.pgm into " ~/neuronbot2_ros2_ws/src/neuronbot2/neuronbot2_nav/map/ "***

   ```
   ros2 launch neuronbot2_nav bringup_launch.py map:=<map_name>.yaml open_rviz:=true use_sim_time:=true
   ```
   * Supported parameters and its value for launch files

      **map**: phenix.yaml | mememan.yaml (default)

      **open_rviz**: true | false (default)

      **use_sim_time**: true | false (default) # if you run navigation in simulation, then use_sim_time must be set to true

   * You can also run localization and navigation in separate terminals.

   ```
   # terminal 1
   ros2 launch neuronbot2_nav localization_launch.py use_sim_time:=true
   # terminal 2
   ros2 launch neuronbot2_nav navigation_launch.py use_sim_time:=true
   # terminal 3
   ros2 launch neuronbot2_nav rviz_view_launch.py use_sim_time:=true
   ```

    ![](readme_resource/mememan_launch_nav.png)
2. Set Estimation
   
   Click "2D Pose Estimate", and set estimation to the approximate location of robot on the map.

   ![](readme_resource/nav_estimate.gif)
3. Set Goal

   Click "2D Nav Goal", and set goal to any free space on the map.
   
   ![](readme_resource/nav_set_goal.gif)

### Control with Behavior Tree
To run this demo, users should execute Gazebo server and Navigation (with Rviz for visualization) first.

1. Open the other terminal and source the environment variables.
   ```
   source /opt/ros/eloquent/setup.bash
   source ~/neuronbot2_ros2_ws/install/local_setup.bash
   ```
2. Run Behavior Tree

   Please go to check this repos:
   https://github.com/Adlink-ROS/BT_ros2

   ![](readme_resource/nav2_bt.gif)
