### Copyright 2012-2014 Pouyan Ziafati, University of Luxembourg 
### Copyright Elliott Smith, Marie Hansen and Bronte Kalebic, CSE, UNSW 2016
> * All image processing and face recognition functionalities are provided by utilizing the Shervin Emami's c++ source code for face recognition (http://www.shervinemami.info/faceRecognition.html).
 * License: Attribution-NonCommercial 3.0 Unported (http://creativecommons.org/licenses/by-nc/3.0/) 

### Welcome to the rsa-face_recognition package
Most of this package is identical to the one found here http://wiki.ros.org/face_recognition
As such this document only descibes the additional face_finder code. Details on how to setup the facial recognition and train faces etc is on the ROS wiki.

## Installation
This instalation process is for **catkin** 
Assuming that your catkin workspace is under **~/catkin_ws**, if not replace **~/catkin_ws** with appropriate location. It also assumes you're running Bash shell, if you're running Zsh, source appropriate **setup.zsh** file.
```
cd ~/catkin_ws/src
git clone https://github.com/brakespear/rsa-face_recognition.git
mv rsa-face_recognition procrob\_functional
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

## Image topic
This package uses the topic
* **camera/rgb/image_raw** 

## Running the face_finder code
Once you have trained faces using the instructions here http://wiki.ros.org/face_recognition you need to do the following
```
rosrun face_recognition Fserver 
rosrun face_recognition face\_finder [name]
```
Where [name] is the name of the person you want the robot to find. **This name must match the label used to train that persons face**
