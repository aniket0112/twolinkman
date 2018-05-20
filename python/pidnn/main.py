import roslaunch, rospy, time, keyboard
import rospkg as rp
import numpy as np
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import time
from collections import deque

pi = 3.1415926

front_setpoint = -45
rear_setpoint = 35

f_topic = '/twolinkman/front_joint_effort_controller/command'
r_topic = '/twolinkman/rear_joint_effort_controller/command'
rcon_launch_path = '/launch/twolinkman_control.launch'
rp = rp.RosPack()
package_path = rp.get_path('twolinkman')

class Joint:
	def __init__(self,topic,setpoint):
		self.topic = topic
		self.pub = setpoint

class PID:
    e = 0
    E = 0
    ep = 0
    def __init__(self,kp = 0,ki = 0,kd = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
    def pid(self, setpoint, process_value):
        self.e = setpoint-process_value
        self.E = self.E + self.e
        ed = self.e - self.ep
        self.ep = self.e
        return (self.kp*self.e+self.kd*ed+self.ki*self.E)    

#ROS Control Launch file
rcon_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(rcon_uuid)
rcon_launch = roslaunch.parent.ROSLaunchParent(rcon_uuid, [package_path+rcon_launch_path])
rcon_launch.start()
time.sleep(10)
#Environment ready-->
if __name__ == '__main__':
    try:
    #SetPoint Node
        front_joint = Joint(f_topic,None)
        rear_joint = Joint(r_topic,None)
        front_PID = PID(0.1,0.0,0)
        rear_PID = PID(0.1,0.0,0)

        def listener_callback(data):
            global front_angle, rear_angle
            quatf = data.transforms[0].transform.rotation
            quatr = data.transforms[1].transform.rotation
            (front_angle, _, _)= euler_from_quaternion([quatf.x,quatf.y,quatf.z,quatf.w])
            (rear_angle, _, _)= euler_from_quaternion([quatr.x,quatr.y,quatr.z,quatr.w])
            front_angle = front_angle*180/pi
            rear_angle = rear_angle*180/pi
            #rospy.loginfo('Front angle: '+str(front_angle)+' Rear angle: '+str(rear_angle))    

        setpointnode = rospy.init_node('SetPoint',anonymous=True)
        front_joint.pub = rospy.topics.Publisher(front_joint.topic,Float64,queue_size=10)
        rear_joint.pub = rospy.topics.Publisher(rear_joint.topic,Float64,queue_size=10)
        listener = rospy.topics.Subscriber('/tf',TFMessage,callback=listener_callback,queue_size = 10)

        plt_front_angle = deque([]);
        plt_rear_angle = deque([]);

        fig, ax = plt.subplots()

        while not rospy.is_shutdown():
            time.sleep(0.001);
            front_joint.pub.publish(front_PID.pid(front_setpoint,front_angle));
            rear_joint.pub.publish(rear_PID.pid(rear_setpoint,rear_angle));
            if(len(plt_front_angle) < 20) :
                plt_front_angle.append(front_angle);
            else:
                plt_front_angle.rotate(-1);
                plt_front_angle[19] = front_angle;
            if(len(plt_rear_angle) < 20) :
                plt_rear_angle.append(rear_angle);
            else:
                plt_rear_angle.rotate(-1);
                plt_rear_angle[19] = rear_angle;
            ax.cla()
            ax.set_xlim([0,20])
            ax.set_ylim([-90,90])
            plt_time = [i for i in range(len(plt_front_angle))]
            plt_front_set_angle = [front_setpoint for i in range(len(plt_front_angle))]
            plt_rear_set_angle = [rear_setpoint for i in range(len(plt_rear_angle))]
            ax.plot(plt_time,plt_front_angle,'r',plt_time,plt_rear_angle,'b',
                plt_time,plt_front_set_angle,'r+',plt_time,plt_rear_set_angle,'b+')
            plt.pause(0.0001)
        #	print(published_topics)
    except (KeyboardInterrupt,SystemExit):
        gz_launch.shutdown()
        rcon_launch.shutdown()
        raise

