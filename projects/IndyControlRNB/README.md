## Start with IndySDK
* Install IndySDK first.
* In eclipse, import "IndyControlRNB" folder as a project in eclipse. (*File->Import->General->Existing Projects into Workspace*)
  - There's no need to change workspace.
  - Don't copy the project into the workspace.
* Build all configurations in IndyControlRNB by 
  * right-click-project > Build Configurations build > Build All
* Copy ReleaseJC/JointControlRNB.comp and ReleaseTC/TaskControlRNB.comp to Components folder.
* Set "JController" to "JointControlRNB" and "TController" to "TaskControlRNB" In "indyDeploy.json".
  * **NOTE** You need SDK version control task.
* Remove logging task if it is in "indyDeploy.json". (See TROUBLE SHOOTING section)
* Copy "assets" folder to the working directory.
* Make sdk_license.lic in Components folder and write license info as in following format.
  * USERNAME;EMAIL;SERIAL
  * **NOTE** DO NOT PUT SPACE between the license items
* Build and copy control algorithm libraries (PD, NRIC_PD, ...) to "controllers" folder.
* Run TaskManager in the Indy7 controller with SDK version control task.
* The Web UI can be accessed from browser at port :9990 (joint control), :9991 (task control)
* The trajectory interface is available through TCP/IP on PORT  :9980 (joint control), :9981 (task control)

## How to make new controller (NRMK Framework base)
* Make new Joint Control project following the process in http://docs.neuromeka.com/2.3.0/kr/IndySDK/section3/  
  * Uncheck *Use default location* and set Location to *RNB-CONTROL-PATH/controllers/YOUR-CONTROLLER-NAME*
    - ex) ```C:\Users\RNB_CAD\Desktop\rnb-control\controllers\PD```
    - **NOTE** You must set a unique new name for the controller class!
  * **NOTE** Uncheck ***Debug*** in **Select Configurations** window. We will only use ***Release***.
  * Follow all process down to the "Command setting", but set artifact extension to "so"  
  * **NOTE** put ";" between items in Environment PATH  
* Remove contents of the PROJECT_NAME.cpp and PROJECT_NAME.h
* Copy the contents of "control_algorithm_default.h", "control_algorithm_default.cpp" and change the function content
  * Change *#include "controller_interface.h"* to *#include "../../control_hub/controller_interface.h"*
  * **NOTE** You must add ***EXPORT_CONTROLLER(YOUR-CONTROLLER-NAME)*** at the end of cpp file.
* Implement and build your controller. Copy the compiled file to "INDY_SDK_PATH/controllers/*"
  * **NOTE** To resolve autority issue, ```sudo chown user controllers```

## Operating Indy
* Turn on the control box of Indy
* Recommend to use "MobaXterm" or "putty" for remote access to "STEP PC" 
* Before implementing "release/IndySDK_2.3.0.1/TaskManager" (path may be different depending on PC), type below codes using terminal
```bash
sudo killall -9 UDEVMonitor || sudo killall -9 TaskMan
```
* Implement "TaskManager"
```bash
./TaskManager -j indyDeploy.json
```
* See details in http://docs.neuromeka.com/2.3.0/kr/IndySDK/section2/

## Tips
* You can see the information about IndySDK in http://docs.neuromeka.com/2.3.0/kr/IndySDK/section1/
* You can download IndyFramework in http://docs.neuromeka.com/downloads/
* Version of "Conty" and "IndySW" should be same
* Turn on the logging task to check the details of the error.
  * The file is saved at "/home/user/release/LogData"
  * The erorr details are also displayed on the termial.

## Trouble shooting
* **NOTE** (Indy) Position Error is raised every 10 minute: remove logging task (below script) from indyDeply.json
```
,
{
	"TaskName" : "Indy6DOFLoggingTask",
	"SharedLib" : "libIndy6DOFLoggingTask.so"
}
```
* (Indy) If computing power is not enough, drop the control frequency ("HighRate" in indyDeploy.json). 