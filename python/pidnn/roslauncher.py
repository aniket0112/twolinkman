import roslaunch
import rospkg as rp

#constants
gz_launch_path = '/launch/twolinkman_gazebo.launch'

rp = rp.RosPack()
package_path = rp.get_path('twolinkman')

#Gazebo Launch file 
gz_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(gz_uuid)
gz_launch = roslaunch.parent.ROSLaunchParent(gz_uuid, [package_path+gz_launch_path])
gz_launch.start()
raw_input()