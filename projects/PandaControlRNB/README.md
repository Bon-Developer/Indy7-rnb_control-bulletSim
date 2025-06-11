# Dependency  
* Ubuntu 18.04, ROS melodic (To use franka-ros 0.7.1, for collision behavior setting)
```bash
sudo apt install curl \
&& sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  \
&& sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
&& curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add - \
&& sudo apt update \
&& sudo apt install ros-melodic-desktop-full \
&& echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
&& source ~/.bashrc \
&& sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential \
&& sudo rosdep init \
&& rosdep update
```
* Install libfranka and franka_ros (https://frankaemika.github.io/docs/installation_linux.html)
```bash
sudo apt install ros-melodic-libfranka ros-melodic-franka-ros
```
* Dependency
```bash
sudo apt-get install ros-melodic-moveit-simple-controller-manager \
&& pip install numpy \
&& pip3 install numpy
```
* Setup real-time kernel following instruction in (https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)
  * Available kernel version example: 
  ```bash
  curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.4.109.tar.xz
  curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.4.109.tar.sign
  curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.109-rt56.patch.xz
  curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.109-rt56.patch.sign
  xz -d linux-5.4.109.tar.xz
  xz -d patch-5.4.109-rt56.patch.xz
  ```
  * Verification for above example (for different version, keys will be different. See above link for detail):
  ```bash
  sudo apt install gnupg2 \
  && gpg2  --keyserver hkp://keys.gnupg.net --recv-keys 647F28654894E3BD457199BE38DBBDC86092693E \
  && gpg2  --keyserver hkp://keys.gnupg.net --recv-keys ACF85F9816A8D5F096AE1FD20129F38552C38DF1 \
  && gpg2 --verify linux-5.4.109.tar.sign \
  && gpg2 --verify patch-5.4.109-rt56.patch.sign
  ```
  * Prepare to build the kernel
  ```bash
  tar xf linux-5.4.109.tar \
  && cd linux-5.4.109 \
  && patch -p1 < ../patch-5.4.109-rt56.patch
  ```
  * configure the kernel (If target 'oldconfig' failed error is raised, sudo apt-get install bison && sudo apt-get install flex)
  ```bash
  make oldconfig
  ```
  * Select "Fully Preemptible Kernel", Keep other options default
  * Compile the kernel. Put the number of your CPU cores after -j for optimal multiprocessing. *This step may take over 1 hour!!!*
  ```bash
  fakeroot make -j4 deb-pkg
  ```
  * ***[IMPORTANT] Error handling for above step***
    * If fakeroot generates error, just run "sudo make -j4 deb-pkg"
    * If "sudo make -j4 deb-pkg" also fails, try "sudo make deb-pkg" to stop immediately when the error is raised. You may miss the error message due to multiprocessing (-j4).
    * If the error is raised with "debian/canonical-certs.pem", run following command in the termial to remove wrong key and retry make
    ```bash
    scripts/config --set-str SYSTEM_TRUSTED_KEYS "" \ 
    && sudo make -j4 deb-pkg
    ```
  * install the created package
  ```bash
  sudo dpkg -i ../linux-headers-5.4.109-rt56_*.deb ../linux-image-5.4.109-rt56_*.deb
  ```
  * **REBOOT!!** and check the kernel by below command. It should contain the string PREEMPT RT and the version number you chose.
  ```bash
  uname -a
  ```
  * Allowing real-time permission
  ```bash
  sudo addgroup realtime \
  && sudo usermod -a -G realtime $(whoami)
  ```
  * Copy following lines in /etc/security/limits.conf to set properties for *realtime* group.  
  ```
  @realtime soft rtprio 99
  @realtime soft priority 99
  @realtime soft memlock 102400
  @realtime hard rtprio 99
  @realtime hard priority 99
  @realtime hard memlock 102400
  ```
* set default grub setting  
  * check list of kernel 
  ```bash
  awk -F\' '/menuentry / {print $2}' /boot/grub/grub.cfg 
  ```
  * Edit /etc/default/grub contents as following
  ```
  GRUB_DEFAULT=0  ## Change this if the real time kernel is not loaded. remember the location of the kernel on the grub menu when you boot the computer, and change accordingly, ex) "1>2"
  GRUB_TIMEOUT_STYLE=menu  
  GRUB_TIMEOUT=5  
  ```
  * sudo update-grub  
* disable kernel update  
  * sudo apt-mark hold linux-image-generic linux-headers-generic  
* **REBOOT!!**


# Recommended tools
* GitKraken: git GUI for linux
* Clion: c++ IDE, add "export PATH=$PATH:{YOUR-PATH-TO-CLION}/bin" to .bashrc
* jupyter 
  * install jupyter  
  ```bash
  sudo apt install python3-notebook python-notebook jupyter jupyter-core python-ipykernel  
  ```
  * do server setting as follows
  ```bash
  jupyter notebook --generate-config \
  && jupyter notebook password \
  && vi ~/.jupyter/jupyter_notebook_config.py
  ```
  * enter password twice
  * find #c.NotebookApp.ip = 'localhost'
  * remove '#' and replace 'localhost' with *your ip*
  * find and set c.NotebookApp.open_browser = False if you don't want to open browser when starting jupyter
  
# Installation
* Clone this git on ~/Projects
* compile workspace
```bash
cd ~/Projects/rnb-control/projects/PandaControlRNB \
&& catkin_make -DCMAKE_BUILD_TYPE=Release      
```
* source workspace  
```bash
source ~/Projects/rnb-control/projects/PandaControlRNB/devel/setup.bash \
&& echo 'source ~/Projects/rnb-control/projects/PandaControlRNB/devel/setup.bash' >> ~/.bashrc    
```
* Copy "assets" folder to the ROS working directory. 
```bash
cp -rf ~/Projects/rnb-control/assets ~/.ros/
```
* Build and copy control algorithm libraries (PD, NRIC_PD, ...) to "controllers" folder in the ROS working directory(~/.ros/controllers).
  * See **Building controller using CMAKE** for detail.



# Building controller using CMAKE
* For a new controller, add CMakeLists.txt file with contents as below, in the controller source directory. If CMakeLists.txt is already added, go to *"Compile the controller"* part.
```cmake
cmake_minimum_required(VERSION 3.5)
project(YOUR_PROJECT_NAME)

include_directories(../../3rd_party)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_SHARED_LIBRARY_PREFIX "") # output file name should match with BOOST_PYTHON_MODULE name

add_library(${PROJECT_NAME} SHARED ${PROJECT_NAME}.cpp)
```
* Enter your project name at the place of *YOUR_PROJECT_NAME*, find eigen include folder and put it in include_directories(*)
* Compile the controller
```bash
cmake -DCMAKE_BUILD_TYPE=Release && make
```
* Move the controller binary (.so) file to the controller folder in the ROS working directory.
```bash
mkdir -p ~/.ros/controllers \
|| cp -rf ~/Projects/rnb-control/controllers/YOUR_CONTROLLER_NAME/YOUR_CONTROLLER_NAME.so ~/.ros/controllers
```

## Test your controller
*  Check your code for debugging and an example is in "controllers/test/tset_controller"

# Launch controller  
* Turn panda robot and host computer power on
* When panda is prepared, open Frank Emika Desk on a web browser (https://{PANDA-IP}/desk/)
* On right side, click "unlock" icon to unlock joints.
* Release external activation device. The lamp color will change to blue. (Hold the enabling switch handle if it is connected)
* Launching controller
```bash
roslaunch panda_control joint_control_rnb.launch robot_ip:={PANDA-IP} load_gripper:=false    
```

# Set collision behavior insensitive (franka_ros>0.7.0)
* Edit following lines from /opt/ros/melodic/share/franka_control/config/franka_control_node.yaml
```
collision_config:
  lower_torque_thresholds_acceleration: [200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0]  # [Nm]
  upper_torque_thresholds_acceleration: [200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0]  # [Nm]
  lower_torque_thresholds_nominal: [200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0]  # [Nm]
  upper_torque_thresholds_nominal: [200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0]  # [Nm]
  lower_force_thresholds_acceleration: [200.0, 200.0, 200.0, 250.0, 250.0, 250.0]  # [N, N, N, Nm, Nm, Nm]
  upper_force_thresholds_acceleration: [200.0, 200.0, 200.0, 250.0, 250.0, 250.0]  # [N, N, N, Nm, Nm, Nm]
  lower_force_thresholds_nominal: [200.0, 200.0, 200.0, 250.0, 250.0, 250.0]  # [N, N, N, Nm, Nm, Nm]
  upper_force_thresholds_nominal: [200.0, 200.0, 200.0, 250.0, 250.0, 250.0]  # [N, N, N, Nm, Nm, Nm]
```

# TROUBLE SHOOTING
* Go to franka emika documentation (https://frankaemika.github.io/docs/troubleshooting.html)
* **[IMPORTANT]** Local network speed is extremely important to control panda. No intermediate network hub is permitted between the computer and panda fci (should be connected though a single router). With poor network connection, *communication_constraints_violation* will be raised.

# (FYI) Rebuild process to make new project  
* create package  
```bash
cd ~/Projects/rnb-control/projects/PandaControlRNB/src \
&& catkin_create_pkg panda_ros_repeater controller_interface dynamic_reconfigure eigen_conversions franka_hw geometry_msgs hardware_interface tf tf_conversions message_generation pluginlib realtime_tools roscpp rospy    
```
  
* replace package contents  
  
* compile workspace again  
```bash
cd ~/Projects/rnb-control/projects/PandaControlRNB \
&& sudo rm -rf build && sudo rm -rf devel \
&& catkin_make -DCMAKE_BUILD_TYPE=Release \
&& source ~/Projects/rnb-control/projects/PandaControlRNB/devel/setup.bash  
```
