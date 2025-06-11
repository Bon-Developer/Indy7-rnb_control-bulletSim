# RNB-Control simulator with bullet

## Installation
### (Only for windows) Setup anaconda environment 
* Download and install anaconda (https://www.anaconda.com/)
* Create virtual environment for pybullet simulation on Anaconda prompt
```cmd
conda create -n bullet-sim python=3.9.7
```
* Activate the environment - ***You need to run this everytime you start anaconda prompt***
```cmd
conda activate bullet-sim
```
## Install basic packages
#### windows
```cmd
conda install jupyter \
&& pip install scipy matplotlib requests beautifulsoup4
```
#### linux
```cmd
pip3 install jupyter scipy matplotlib requests beautifulsoup4
```

## Install Bullet SDK
### Get SDK
* Get Bullet Physics SDK 3.17 (https://github.com/bulletphysics/bullet3/releases/tag/3.17)
  - requirement for *windows*: *Microsoft Visual C++ build tool*
* Unzip the source code and put it in ```rnb-control\\projects\\BulletSimControl\\bullet3```
### Install SDK
#### windows
* install pybullet - takes *~30 min*
```bash
pip install pybullet==3.1.7
```
* Install the SDK
  - Set **myvar** in ***bullet3\\build_visual_studio_vr_pybullet_double.bat*** to the anaconda environment path
    * Ex) ```set myvar=%UserProfile%\\AppData\\Local\\conda\\conda\\envs\\bullet-sim```
  - Open ***BulletSimControl\\bullet3\\build3\\vs2010\\0_Bullet3Solution.sln***. When asked, convert the projects to a newer version of Visual Studio.
  - Activate **Release | x64** build configuration. (For 32 bit environment, activate x84)
  - **Build Solution** for the first time - 66 should succeed and 1 project may fail.  
  - To debug your projects, build **Debug | x64** configuration too.
#### linux
* install pybullet - takes *~30 min*
```bash
pip3 install pybullet==3.1.7
```
* Install the SDK
  - Go to ```rnb-control/projects/BulletSimControl/bullet3``` and run below:
```bash
./build_cmake_pybullet_double.sh
```
    
## Build BulletSimControl
### windows
* run ***BulletSimControl.sln***
* Build **Release | x64** configuration
  - Use **Debug | x64** configuration to debug the program
* Now you can run BulletSimControl on *rnb-control/projects/BulletSimControl/x64/Release*

### linux
* Go to *rnb-control/projects/BulletSimControl/BulletSimControl* and build **BulletSimControl**
```bash
cmake -DCMAKE_BUILD_TYPE=Release && make
```
* Now you can run BulletSimControl on *rnb-control/projects/BulletSimControl/bin*. go to the directory and run:
```bash
./BulletSimControl
```

### Common
* To plot data on WebUI, copy ***assets*** folder in ***rnb-control*** to the *Release* or *bin* folder created above.

## Building a controller
### windows
* Open **controllers/VS14/VS14.sln** (If you have newer VS, please copy this project and create an updated version)
* **[Visual Studio 2015]** Create new dynamic library project in the solution with name of the controller (https://docs.microsoft.com/ko-kr/cpp/build/walkthrough-creating-and-using-a-dynamic-link-library-cpp?view=msvc-140)
  - *Add -> New Project -> Win32 Project*
  - Set location to *rnb-control/controllers*
  - Set project name same as the controller
  - Select *DLL* and *empty project*
* **[Visual Studio 2019]** Create new dynamic library project in the solution with name of the controller 
  - *Add -> New Project -> Windows desktop wizard*
  - Set location to *rnb-control/controllers* (if you try to build the existing controller, move original files to a temporary folder and delete the existing folder. After making the new project, put the files in the original folder)
  - Set project name same as the controller
  - Set application type as *.dll* and select *empty project*
* Add source file from the controller folder. ex: ```controller/PD/PD.cpp```
* Add include directory for Eigen ```..\..\3rd_party``` for ***All Contiguration***
* Activate **Release | x64** configuration and build.
* Copy compiled **dll** files to **BulletSimControl/x64/Release/controllers**
  - **dll** files in **../VS14/x64/Release**
* **[IMPORTANT]** The Simulator and Controller Configuration should be identical. If you run Simulator in **Debug** mode, the controller built in **Debug** configuration should also be used.

### linux
* Follow instruction in **Building controller using CMAKE** section of [rnb-control/projects/PandaControlRNB/README.md](../PandaControlRNB/README.md)
* After compiling the controller, copy the controller to BulletSimControl working directory. 
  - Move to *rnb-control/projects/BulletSimControl/bin* and run following command:
  ```bash
  mkdir -p ./controllers \
  || cp -rf ../../../controllers/YOUR_CONTROLLER_NAME/YOUR_CONTROLLER_NAME.so ./controllers
  ```

## Adding new robot urdf
* Copy **urdf** file and mesh files to *rnb-control/urdf*.
* In the **urdf** file, remove "package://{pakage_name}/ and represent mesh files in relative path
* To be used with default BulletSimControl, the manipulator should be single chain without fixed joints and inertial information.
    * Remove fixed joints (and transfrom children to the parent coordinate)
    * Add inertial informations
    * Set ```<mass value="0.0"/>``` for the first link inertial to mark that the link is fixed to the world coordinate. (otherwise, the robot will fall freely)
* Indy7 and Kuka iiwa are available at the moment (panda has no inertial info)

## Tips
* [Windows SDK version issue] Error with "The Windows SDK version was not found" occurs in building project
  - *Right click {your_project} -> Retarget Projects*

### ----------- TBD --------------
* Bullet용 ControlHub 구현
  - dynamic 등 parameter 계산
  - ControlHub 윈도우 호환 작업
* Pybullet으로 환경 / 오브젝트 추가 등 구현
