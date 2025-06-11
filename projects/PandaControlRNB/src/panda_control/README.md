# Dependency  
sudo apt-get install ros-kinetic-moveit-simple-controller-manager  

# Installation  
* create workspace  
mkdir -p ~/franka_ws/src  
cd ~/franka_ws/src  
catkin_init_workspace  
  
* copy project into ~/franka_ws/src  
  
* compile workspace  
cd ~/franka_ws  
catkin_make -DCMAKE_BUILD_TYPE=Release      
  
* source workspace  
echo 'source ~/franka_ws/devel/setup.bash' >> ~/.bashrc    
  
# launch repeater  
roslaunch panda_control move_to_start.launch robot_ip:=192.168.0.13 load_gripper:=true  
roslaunch panda_control joint_velocity_repeater.launch robot_ip:=192.168.0.13 load_gripper:=true    
  
# Rebuild process  
* create package  
cd ~/franka_ws/src  
catkin_create_pkg panda_control controller_interface dynamic_reconfigure eigen_conversions franka_hw geometry_msgs hardware_interface tf tf_conversions message_generation pluginlib realtime_tools roscpp rospy    
  
* replace package contents  
  
* compile workspace again  
cd ~/franka_ws  
sudo rm -rf build && sudo rm -rf devel    
catkin_make -DCMAKE_BUILD_TYPE=Release  
source ~/franka_ws/devel/setup.bash  