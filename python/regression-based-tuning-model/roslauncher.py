###################################################################################
# roslauncher.py
# --------------------------------------------------------------------------------- 
# This script runs the roscore node and gazebo server. It loads the models into
# gazebo from 'twolinkman' rospackage. The complete environment to simulate manipulator
# is set up by this script. To modify the manipulator, urdf file in 'twolinkman-
# description' needs to be modified. Controller and other apps can be attached to this
# roscore for simulated controller environments.
# --------------------------------------------------------------------------------
# How to use?
# Run this script in a terminal and leave it as it is. Run controller nodes and
# other applications in separate terminal.
# End the script by 'Ctrl+C'
###################################################################################

import roslaunch
import rospkg as rp

#constants
gz_launch_path = '/launch/twolinkman_gazebo.launch'

rp = rp.RosPack()
package_path = rp.get_path('twolinkman')

#Gazebo Launch file 
gz_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(gz_uuid)
gz_launch = roslaunch.parent.ROSLaunchParent(gz_uuid, [package_path+gz_launch_path])	#Gazebo world loading using roslaunch file
gz_launch.start()
raw_input()																				#Halt program