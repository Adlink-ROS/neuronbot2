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

## Installation

1. Install ROS2 dashing if you don't have.
   - Follow [this official installing tutorial](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/).
2. Create workspace
    ```
    mkdir -p ~/neuronbot2_ros2_ws/src
    ```
3. Git clone this package
    ```
    cd ~/neuronbot2_ros2_ws/
    wget https://raw.githubusercontent.com/Adlink-ROS/neuronbot2_ros2.repos/dashing-devel/neuronbot2_ros2.repos
    vcs import src < neuronbot2_ros2.repos
    ```
4. Install dependencies
   ```
   cd ~/neuronbot2_ros2_ws/
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y --rosdistro dashing
   ```
5. Colcon build the package 
   ```
   cd ~/neuronbot2_ros2_ws/
   source /opt/ros/dashing/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source ~/neuronbot2_ros2_ws/install/local_setup.bash
   ```
6. There are two operations tutorials:
   - If you have NeuronBot2, please follow the tutorial: [Bring up your NeuronBot2](#bring-up-your-neuronbot2)
   - If you don't have NeuronBot2, but want to run simulation, please follow the tutorial: [Bring up in Simulation](#bring-up-in-simulation)

---

## Bring up your NeuronBot2
![](readme_resource/nb2_opening_re.gif)

Now, it's time to launch your NeuronBot2 and do a Robotic-Hello-World thing -- telop it.
### Launch NeuronBot2
Open a new terminal (Ctrl + Alt + t).
   ```
   source /opt/ros/dashing/setup.bash
   source ~/neuronbot2_ros2_ws/install/local_setup.bash
   ros2 launch neuronbot2_bringup bringup_launch.py
   ```
###  Teleop NeuronBot2
   ```
   source /opt/ros/dashing/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Follow the hints and start to cruise your NeuronBot2.

   ![](readme_resource/teleop.png)
### SLAM your map
1. Launch SLAM as well as Rviz.
   
   ***We provide three slam methods.***

   * Gmapping
   ```
   ros2 launch  neuronbot2_slam gmapping_launch.py open_rviz:=true
   ```
   * Slam_toolbox
   ``` 
   ros2 launch  neuronbot2_slam slam_toolbox_launch.py open_rviz:=true
   ```
   * Cartographer
   ```
   ros2 launch  neuronbot2_slam cartographer_launch.py open_rviz:=true
   ```
2. Teleop NeuronBot2 to explore the world
   ```
   # Run on the other terminal
   source /opt/ros/dashing/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. Save the map
   ```
   source /opt/ros/dashing/setup.bash 
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
   source /opt/ros/dashing/setup.bash
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
   source /opt/ros/dashing/local_setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   ![](readme_resource/teleop.png)

   ***p.s. To alleviate CPU consumption, close GAZEBO GUI by clicking x. This will not end the simulation server, which is running backend***

### SLAM the world
1. Launch SLAM as well as Rviz while the Gazebo simulation is running.
   
   ***We provide three slam methods.***

   * Gmapping
   ```
   ros2 launch  neuronbot2_slam gmapping_launch.py open_rviz:=true use_sim_time:=true
   ```
   * Slam_toolbox
   ``` 
   ros2 launch  neuronbot2_slam slam_toolbox_launch.py open_rviz:=true use_sim_time:=true
   ```
   * Cartographer
   ```
   ros2 launch  neuronbot2_slam cartographer_launch.py open_rviz:=true use_sim_time:=true
   ```
    ![](readme_resource/slam_rviz.png)
2. Teleop NeuronBot2 to explore the world
   ```
   # Run on the other terminal
   source /opt/ros/dashing/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   ![](readme_resource/slam_teleop_8x.gif)
3. Save the map
   ```
   source /opt/ros/dashing/setup.bash 
   ros2 run nav2_map_server map_saver -f <map_dir>/<map_name>
   ```

   Then, you shall turn off SLAM.

### Navigate to the desired location
Once users obtain the map, the pgm file & yaml file, navigation is good to go.

1. Launch Navigation as well as Rviz while the Gazebo simulation is running. Default map is set to mememan.yaml.

   * Navigate in mememan map
   ```
   ros2 launch neuronbot2_nav bringup_launch.py map:=$HOME/neuronbot2_ros2_ws/src/neuronbot2/neuronbot2_nav/map/mememan.yaml open_rviz:=true use_sim_time:=true
   ```
   * Navigate in phenix map
   ```
   ros2 launch neuronbot2_nav bringup_launch.py map:=$HOME/neuronbot2_ros2_ws/src/neuronbot2/neuronbot2_nav/map/phenix.yaml open_rviz:=true use_sim_time:=true
   ``` 
   * Try navigation on your own map. There are two ways:
     - The first one: Assign your map path directly
       ```
       ros2 launch neuronbot2_nav bringup_launch.py map:=<full_path_to_your_map_name.yaml> open_rviz:=true use_sim_time:=true
       ```
     - The second one: Put the <map_name>.yaml and <map_name>.pgm into `~/neuronbot2_ros2_ws/src/neuronbot2/neuronbot2_nav/map/`
       ```
       mv <map_name>.yaml <map_name>.pgm ~/neuronbot2_ros2_ws/src/neuronbot2/neuronbot2_nav/map/
       # You need to rebuild the neuronbot2
       colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
       ros2 launch neuronbot2_nav bringup_launch.py map:=<map_name>.yaml open_rviz:=true use_sim_time:=true
       ```

    ![](readme_resource/mememan_launch_nav.png)

2. Set Estimation
   
   Click "2D Pose Estimate", and set estimation to the approximate location of robot on the map.

   ![](readme_resource/nav_estimate.gif)

3. Set Goal

   Click "2D Nav Goal", and set goal to any free space on the map.
   
   ![](readme_resource/nav_set_goal.gif)

### Control with Behavior Tree

You can control neuronbot with behavior tree (BT) like the following gif.

Please refer to [BT_ros2](https://github.com/Adlink-ROS/BT_ros2/tree/dashing-devel) for more detail information.

![](readme_resource/nav2_bt.gif)

## Trouble Shooting
1. If you see below warning messages after NeuronBot performing navigation, don't worry, it has been fixed at ROS 2 Eloquent.

<img src="./readme_resource/bt_navigator_setUsingDedicatedThread.png" title="" width="100%" align="middle">
<br/>
