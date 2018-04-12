import roslaunch, rospy, time
import rospkg as rp
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
#constants
gz_launch_path = '/launch/twolinkman_gazebo.launch'
rcon_launch_path = '/launch/twolinkman_control.launch'
f_topic = '/twolinkman/front_joint_effort_controller/command'
r_topic = '/twolinkman/rear_joint_effort_controller/command'

class Joint:
	def __init__(self,topic,setpoint):
		self.topic = topic		
		self.pub = setpoint

class PID:
	e = 0
	E = 0
	ep = 0
	def __init__(self,kp,ki,kd):
		self.kp = kp
		self.ki = ki
		self.kd = kd


front_joint = Joint(f_topic,None)
rear_joint = Joint(r_topic,None)
front_PID = PID(0,0,0)
rear_PID = PID(0,0,0)

rp = rp.RosPack()
package_path = rp.get_path('twolinkman')

#Gazebo Launch file 
gz_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(gz_uuid)
gz_launch = roslaunch.parent.ROSLaunchParent(gz_uuid, [package_path+gz_launch_path])
#ROS Control Launch file
rcon_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(rcon_uuid)
rcon_launch = roslaunch.parent.ROSLaunchParent(rcon_uuid, [package_path+rcon_launch_path])

print('Loading, sleep start')

gz_launch.start()
time.sleep(5)
rcon_launch.start()
time.sleep(1)

print('Sleep end')

#Environment ready-->
#SetPoint Node
def listener_callback(data):
	rospy.loginfo(data)

setpointnode = rospy.init_node('SetPoint',anonymous=True)
front_joint.pub = rospy.topics.Publisher(front_joint.topic,Float64,queue_size=10)
rear_joint.pub = rospy.topics.Publisher(rear_joint.topic,Float64,queue_size=10)
listener = rospy.topics.Subscriber('/tf',TFMessage,callback=listener_callback,queue_size = 10)

i = 0
while not rospy.is_shutdown():
	time.sleep(1)
	i  = i + 0.01
	front_joint.pub.publish(i)
	rear_joint.pub.publish(1 - i)
	if i >= 1:
		i = 0
#	print(published_topics)
#gz_launch.shutdown()
#rcon_launch.shutdown()
	
