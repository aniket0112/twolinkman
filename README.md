# twolinkman
Two Link Manipulator control in ROS

## Getting started

### Prerequisites
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) with Gazebo-7 (Desktop-Full Install)
* [Anaconda](https://www.anaconda.com/download/) with Python 2.7 version

### Installing
* Clone the repository to the local machine and change the working branch to 'ctc' OR Download the .zip file of this branch
* Place the repository folder in ```/your/machine/catkin_ws/src```
* Navigate into ```catkin_ws``` folder and open terminal.
* Type ```catkin_make``` and press enter
* After catkin has completed making the workspace, ROS should be able to locate your new rospackage ```twolinkman```. To check it type in terminal ```rospack find twolinkman```. This should print out the location of the ```twolinkman_description``` folder
If you the rospackage location is not displayed, please contact the collaborators.

### Running the tests
* Open a terminal and type ```roslaunch twolinkman twolinkman_gazebo.launch```. A Gazebo world should be loaded with a two link manipulator at the center. Minimize this terminal.
* Open another terminal and type ```roslaunch twolinkman twolinkman_control.launch```. After several logging prints, the logging should stop at ```Started controllers: joint_state_controller, rear_joint_effort_controller, front_joint_effort_controller```. Minimize this terminal.
* Open another terminal and type ```rostopic pub /twolinkman/front_joint_effort_controller/command std_msgs/Float64 "data: 1.0"```. This should move the front link of the two link manipulator. Similarly try to change values of the topics by using ```rostopic pub``` and monitor angles using ```rostopic echo```
*NOTE* : If anything is logged in red color while running any of the above commands, there has been some error. Contact collaborators in such case with the log messages.

## Python interface
* Navigate to ```Python``` directory in the repository and open a terminal.
* Type ```python roslauncher.py```. The same world should be spawned in Gazebo as ```twolinkman_gazebo.launch``` file. Minimize the terminal
* Open a new terminal at same location and type ```python main.py```. Logging as in ```twolinkman_control.launch``` file will be printed, followed by the Gazebo model to move to desired joint angles and plotting a graph of current and desired joint angles.
*NOTE* : While running the python files, module import errors may occur. Keep installing the missing modules using pip. If the problem persists, contact collaborators.

## Built with
* [SolidWorks](http://www.solidworks.in/Default.htm) - 3D CAD model
* [ROS Kinetic](http://wiki.ros.org/kinetic) - ROS control node
* [Gazebo 7](http://gazebosim.org/) - Simulation environment
* [Python 2.7](https://anaconda.org/) - Python interface for coding custom controllers

## Authors
* [Aniket Sharma] (https://github.com/aniket0112)
* [Bhavik Parmar] (https://github.com/parmarbhavik)
* Sagar Malik

## Acknowledgements
Hearty thanks to Dr. S. N. Sharma for pointing out useful resources to refer to while working on this project. Deep gratitutde towards the mechanical team of the project for developing the experimental setup to test control schemes. 
