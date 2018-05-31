# PIDNN
Two Link Manipulator control in ROS with adaptive features using PIDNN

## Making dataset
* If the setup has passed the tests mentioned in the repository main directory 'README' file, you can start collecting dataset for your manipulator model.
* Run the ```roslauncher.py``` and ```main.py``` scripts in this directory. By default the desired trajectory to trace is:
'''X = -0.4*(1-np.exp(-0.125*t))```
```Y = (1-e^(-0.125*t))*-0.4+e^(-0.125*t)*-0.6'''
The ```main.py``` scripts ends with a prompt. Enter "RESET" with double quotes and press enter to reposition the Gazebo world to run the simulation iteratively if required.
* The prompt pops up every 2 seconds.
* The dynamic reconfiguration of PID parameters can be done using ```rqt```.
* Open a new terminal and type ```rqt```.
* From the menu bar, go to ```Plugins > Configuration > Dynamic Reconfigure```
* Select the controller which you want to tune and adjust PID parameters by typing in the textbox or scrolling the bars and press Enter.
* This enables live PID tuning during simulation of the manipulator.
* If a satisfactory data set is available, you can note it down in a text file named 'train-dataset-xxx', where xxx should be renamed to avoid collisions in with most recently pushed dataset on master branch. Increment a number of latest available dataset by 1.
* Separate data by single whitespaces and end with a newline (enter) when dataset is complete.
* Sample is shown in 'train-dataset-001.txt'

*NOTE*: A more user friendly interface is under development.
