###################################################################################
# main.py
# --------------------------------------------------------------------------------- 
# main routine program to attach controllers on the manipulator.
# --------------------------------------------------------------------------------
# How to use?
# Wait for gazebo screen to be ready before running this script in a terminal 
# (other than 'roslauncher.py'). By default, joint effort controllers are selected. 
# PID parameters can be changed in the code whose values are held in variables KP and KD.
# 'front_setpoint' and 'rear_setpoint' are the two angular setpoints of front and
# rear angle respectively.
# To change the controller types (effort or position/angle),'twolinkman_control.launch' 
# roslaunch file in 'twolinkman_description' dir should be modified.
# End the script by 'Ctrl+C'
###################################################################################

import roslaunch, rospy, time, keyboard
import rospkg as rp
import numpy as np
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetLinkState 
import matplotlib.pyplot as plt
import time
import threading
from collections import deque
import sys
pi = 3.1415926

front_setpoint = 0
rear_setpoint = -90

f_topic = '/twolinkman/front_joint_effort_controller/command'                                   #topic name for front joint controller
r_topic = '/twolinkman/rear_joint_effort_controller/command'                                    #topic name for rear joint controller
rcon_launch_path = '/launch/twolinkman_control.launch'
rp = rp.RosPack()
package_path = rp.get_path('twolinkman')

front_link_length = 0.3035
rear_link_length = 0.3035

def trajectory_generation(t):
    return -0.4*(1-np.exp(-0.125*t)), (1-np.exp(-0.125*t))*-0.4+np.exp(-0.125*t)*-0.6

def inverseKinematics(x,y):
    global front_link_length, rear_link_length
    d = np.sqrt(x**2+y**2)
    front_angle = pi - np.arccos((front_link_length**2 + rear_link_length**2 - d**2)/(2*front_link_length*rear_link_length))
    beta = np.arctan2(front_link_length*np.sin(front_angle),front_link_length*np.cos(front_angle)+rear_link_length)
    alpha = np.arctan2(y,x)
    rear_angle = pi - (alpha+beta)
    return np.rad2deg(np.arctan2(np.sin(front_angle),np.cos(front_angle))), np.rad2deg(np.arctan2(np.sin(rear_angle),np.cos(rear_angle)))

class Joint:
    def __init__(self,topic,setpoint):
        self.topic = topic
        self.pub = setpoint

#ROS Control Launch file
rcon_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(rcon_uuid)
rcon_launch = roslaunch.parent.ROSLaunchParent(rcon_uuid, [package_path+rcon_launch_path])      #roslaunch the ros+gazebo controller
rcon_launch.start()
time.sleep(2)
exit_ = False

if __name__ == '__main__':
    try:
        #Joint instances
        front_joint = Joint(f_topic,None)
        rear_joint = Joint(r_topic,None)
        #Variables used for plotting setpoint and process values
        plt_front_angle = deque([]);
        plt_rear_angle = deque([]);
        plt_ros_front_angle = deque([]);
        plt_ros_rear_angle = deque([]);
        plt_x = deque([])
        plt_y = deque([])
        plt_ros_x = deque([])
        plt_ros_y = deque([])
        fig, ax = plt.subplots(nrows=2)
        global_x = 0
        global_y = -0.607
        ros_x = 0
        ros_y = -0.607
        lock = threading.Lock()                                                                 #Thread lock to avoid data corruption

        def listener_callback(data):                                                            #Listeners to topic messages when published to get angle feedback values
            global front_angle, rear_angle
            quatf = data.transforms[0].transform.rotation
            quatr = data.transforms[1].transform.rotation
            (front_angle, _, _)= euler_from_quaternion([quatf.x,quatf.y,quatf.z,quatf.w])
            (rear_angle, _, _)= euler_from_quaternion([quatr.x,quatr.y,quatr.z,quatr.w])
            front_angle = np.rad2deg(front_angle)
            rear_angle = np.rad2deg(np.arctan2(np.sin(rear_angle+pi),np.cos(rear_angle+pi)))    #Adjustments to make angular measurements from +ve x axis
        def controller():
            #Update setpoints
            global exit_, lock, initial_time, global_x, global_y
            global front_setpoint, front_angle, rear_setpoint, rear_angle
            lock.acquire()
            try:
                global_x,global_y = trajectory_generation(time.time()-initial_time)
                front_setpoint,rear_setpoint = inverseKinematics(global_x,global_y)   #Calculate joint angles
                if (abs(front_setpoint) <= 170 and rear_setpoint >= -170 and rear_setpoint <= -10):                        #Check limits of angular positions
                    front_joint.pub.publish(np.deg2rad(front_setpoint))
                    rear_joint.pub.publish(np.deg2rad(rear_setpoint+90))
                else:
                    print('Out of bounds!')                   
            finally:
                lock.release()
            timer = threading.Timer(0.01,controller)
            timer.setDaemon(True)
            timer.start()        
        def rosservice_client():
            global ros_x, ros_y, lock
            lock.acquire()
            try:
                rospy.wait_for_service('/gazebo/get_link_state')
                model = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
                obj = model("twolinkman::load_link","world")
                ros_y = obj.link_state.pose.position.z-1.00
                ros_x = -(obj.link_state.pose.position.y+0.057904731758301235)
                print(ros_x,ros_y)
            finally:
                lock.release()
            timer = threading.Timer(0.5,rosservice_client)
            timer.setDaemon(True)
            timer.start()        
        #new rosnode to publish effort messages from script to ros+gazebo control and latch messages
        #from listeners
        setpointnode = rospy.init_node('SetPoint',anonymous=True)
        front_joint.pub = rospy.topics.Publisher(front_joint.topic,Float64,queue_size=10)
        rear_joint.pub = rospy.topics.Publisher(rear_joint.topic,Float64,queue_size=10)
        listener = rospy.topics.Subscriber('/tf',TFMessage,callback=listener_callback,queue_size = 10)
        time.sleep(2)

        timer = threading.Timer(0.01,controller)                                                   #Parallel thread to publish setpoints
        timer.setDaemon(True)
        initial_time = time.time()
        timer.start()                                                                              #Start thread

        timer_ = threading.Timer(0.5,rosservice_client)                                                   #Parallel thread to listen to end-effector coordinates
        timer_.setDaemon(True)
        timer_.start()                                                                              #Start thread
        
        #PID loop
        while not rospy.is_shutdown():
            time.sleep(1)
            lock.acquire()
            try:
                #Plotting routine
                if(len(plt_front_angle) < 20) :
                    plt_front_angle.append(front_setpoint);
                else:
                    plt_front_angle.rotate(-1);
                    plt_front_angle[19] = front_setpoint;
                if(len(plt_rear_angle) < 20) :
                    plt_rear_angle.append(rear_setpoint);
                else:
                    plt_rear_angle.rotate(-1);
                    plt_rear_angle[19] = rear_setpoint;

                if(len(plt_ros_front_angle) < 20) :
                    plt_ros_front_angle.append(front_angle);
                else:
                    plt_ros_front_angle.rotate(-1);
                    plt_ros_front_angle[19] = front_angle;
                if(len(plt_ros_rear_angle) < 20) :
                    plt_ros_rear_angle.append(rear_angle);
                else:
                    plt_ros_rear_angle.rotate(-1);
                    plt_ros_rear_angle[19] = rear_angle;
                
                if(len(plt_x) < 20) :
                    plt_x.append(global_x);
                else:
                    plt_x.rotate(-1);
                    plt_x[19] = global_x;
                if(len(plt_y) < 20) :
                    plt_y.append(global_y);
                else:
                    plt_y.rotate(-1);
                    plt_y[19] = global_y;

                if(len(plt_ros_x) < 20) :
                    plt_ros_x.append(ros_x);
                else:
                    plt_ros_x.rotate(-1);
                    plt_ros_x[19] = ros_x;
                if(len(plt_ros_y) < 20) :
                    plt_ros_y.append(ros_y);
                else:
                    plt_ros_y.rotate(-1);
                    plt_ros_y[19] = ros_y;

                ax[0].cla()
                ax[0].set_xlim([0,20])
                ax[0].set_ylim([-150,150])
                plt_time = [i for i in range(len(plt_front_angle))]
                ax[0].plot(plt_time,plt_ros_front_angle,'r',plt_time,plt_ros_rear_angle,'b',
                    plt_time,plt_front_angle,'r+',plt_time,plt_rear_angle,'b+')
                ax[1].cla()
                ax[1].set_xlim([0,20])
                ax[1].set_ylim([0,-1])
                ax[1].plot(plt_time,plt_ros_x,'r',plt_time,plt_ros_y,'b',
                    plt_time,plt_x,'r+',plt_time,plt_y,'b+')
                ax[0].legend(['Front Angle PV','Rear Angle PV', 'Front Angle SP', 'Rear Angle SP'])
                ax[1].legend(['X (actual)','Y (actual)','X','Y'])
                plt.pause(0.0000001)
                pass
            finally:
                lock.release()
    except (KeyboardInterrupt,SystemExit):
        gz_launch.shutdown()
        rcon_launch.shutdown()
        exit_ = True
        sys.exit()
        raise Exception('Closing program...')
