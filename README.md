# Summary
It is a meta-package for 'Hit the Ball without markers' ROS Project.  
It referenced [open_manipulator_pick_and_place Example](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_perceptions/#pick-and-place-example)
, and edited it a bit for detecting 3D objects **without any markers** and controlling them.


# Meta-package structure
**This 'ros_manipulator_hit_the_ball' meta-package is consisted of 4 packages**
### ros_manipulator_hit_the_ball
> A package commanding 3 executing modes(HOME, START, STOP).  
> I made it by editing 'open_manipulator_pick_and_place' pkg a bit.
### ball_track_alvar
> A core package in this project detecting user's balls without any markers.  
> I made it by editing 'ar_track_alvar' pkg a bit.
### ball_track_alvar_msgs
> A package for messages used in 'ball_track_alvar' pkg.  
> It is just a copy of 'ar_track_alvar_msgs' pkg.  
> Many names are changes for avoiding build errors occured by 'same name' and better readable codes
### ros_manipulator_description
> A package for a core launch file and showing things in 3D.  
> It is just a combination of copies of 'open_manipulator_description' pkg and 'open_manipulator_ar_markers' pkg
> Many names are changes for avoiding build errors occured by 'same name' and better readable codes
   
> The following commands will install this 'ros_manipulator_hit_the_ball' meta-pkg
> ```
> $ cd ~/catkin_ws/src
> $ git clone https://github.com/Kihwan-Ryoo/ros_manipulator_hit_the_ball.git
> $ cd ~/catkin_ws && catkin_make
> ```

#### additional needs
- _[open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) meta-package_ for using 'open_manipulator_controller' pkg
  > The following commands will install 'open_manipulator' meta-pkg
  > ```
  > $ cd ~/catkin_ws/src
  > $ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
  > $ cd ~/catkin_ws && catkin_make
  > ```
- _[realsense](https://github.com/IntelRealSense/realsense-ros) meta-package_
  > The following commands will install relevant Intel RealSense Depth Camera library
  > ```
  > $ sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
  > $ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
  > $ sudo apt-get install librealsense2-dev librealsense2-utils ros-kinetic-rgbd-launch
  > ```
  > ```
  > $ cd ~/catkin_ws/src
  > $ git clone https://github.com/intel-ros/realsense.git
  > $ cd ~/catkin_ws && catkin_make
  > ```


# Implementation
Tested environment: Ubuntu 18.04 + ros-melodic
   
Run roscore
```
$ roscore
```
After run roscore, Run the controller of OpenMANIPULATOR.  
It works only when a real robot arm is connected.
```
$ roslaunch open_manipulator_controller open_manipulator_controller 
```
Execute a package that recognizes the balls without any markers.  
Enter the camera you are using and the diameter of the ball.
```
$ roslaunch ros_manipulator_description ball_pose.launch camera_model:=realsense_d415 user_ball_diameter:=${ball_diameter_in_centimeter}
```
Execute a package that commands 3 modes.
```
$ roslaunch ros_manipulator_hit_the_ball ros_manipulator_hit_the_ball.launch
```
### Result
Captured image  
<img src="/ros_manipulator_description/image/hit_the_ball_capture.jpg">
[see full video here](https://youtu.be/Sb9Ap8X50mk) (Youtube)
