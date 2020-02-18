## NueronBot 2 in ROS1
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

Users are able to checkout to different branch of this package to run on ROS1(melodic-version) and ROS2(dashing-version).
```
git clone https://github.com/airuchen/neuronbot2.git

# For ROS melodic
git checkout melodic-dev

# For ROS2 Dashing
git checkout dashing-devel
``` 

# ROS1
## Simulation quick-start guide
![](readme_resource/NueronBot2_sim.jpg)
### Git Clone & Dependencies Installation
1. [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Create workspace
    ```
    mkdir -p ~/neuronbot2_ros1_ws/src
    cd ~/neuronbot2_ros1_ws/src
    ```
3. Git clone this package
    ```
    git clone https://github.com/airuchen/neuronbot2.git
    # Checkout to ROS melodic version
    git checkout melodic-dev
    ```
4. Install dependencies
   ```
   cd ~/neuronbot2_ros1_ws/
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y # Show my respect to this line
   ```
5. Catkin_make 
   ```
   cd ~/neuronbot2_ros1_ws/
   source /opt/ros/melodic/setup.bash
   catkin_make
   source devel/setup.bash
   ```
### Summon the NeuronBot2 into Gazebo
1. Specify the model path for Gazebo
   ```
   source /opt/ros/melodic/setup.bash
   source ~/neuronbot2_ros1_ws/devel/setup.bash
   export GAZEBO_MODEL_PATH=~/neuronbot2_ros1_ws/neuronbot2/neuronbot2_gazebo/models
   ```
2. Launch Gazebo simulation.
    
   ***There are two worlds for users to explore.***
   * Mememan world
   ```
   roslaunch neuronbot2_gazebo neuronbot2_mememan.launch
   ```
   ![](./readme_resource/mememan_world.png)
   * Phenix world
   ```
   roslaunch neuronbot2_gazebo neuronbot2_phenix.launch    
   ```
   ![](readme_resource/phenix_world.png)
3. Teleop it in the world

    Users are able to control the NeuronBot2 with the following rosnode.
   ```
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```
   ![](readme_resource/teleop.png)
### SLAM the world
1. Launch SLAM as well as Rviz while the Gazebo simulation is running.
   ```
   roslaunch neuronbot2_slam gmapping.launch open_rviz:=true 
   ```
    ![](readme_resource/slam_rviz.png)
2. Teleop it to explore the world
   ```
   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```
   ![](readme_resource/slam_teleop_8x.gif)
3. Explore the world with move_base while SLAM with gmapping
   ```
   roslaunch neuronbot2_nav move_base.launch
   ```
   ![](readme_resource/slam_move_base_8x.gif)
4. Save the map
   ```
   source ~/neuronbot2_ros1_ws/devel/setup.bash
   roscd neuronbot2_nav/maps/
   rosrun map_server map_saver -f <map_name>
   ```

   Then, you shall turn off Gmapping.
### Navigate to the desired location
Once users obtain the map, the pgm file & yaml file, navigation is good to go.
1. Launch Navigation as well as Rviz while the Gazebo simulation is running.
    ```
    roslaunch neuronbot2_nav neuronbot2_nav.launch map_name:=mememan.yaml open_rviz:=true
    ```
    ![](readme_resource/mememan_launch_nav.png)
2. Set Estimation
   
   Click "2D Pose Estimate", and set estimation to the approximate location of robot on the map.

   ![](readme_resource/nav_estimate.gif)
3. Set Goal

   Click "2D Nav Goal", and set goal to any free space on the map.
   
   ![](readme_resource/nav_set_goal.gif)

