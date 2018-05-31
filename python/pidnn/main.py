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
from std_srvs.srv import Empty


pi = 3.1415926

front_setpoint = 0
rear_setpoint = -90
plt_length = 40

f_topic = '/twolinkman/front_joint_effort_controller/command'                                   #topic name for front joint controller
r_topic = '/twolinkman/rear_joint_effort_controller/command'                                    #topic name for rear joint controller
rcon_launch_path = '/launch/twolinkman_control.launch'
rp = rp.RosPack()
package_path = rp.get_path('twolinkman')

front_link_length = 0.3035
rear_link_length = 0.3035

class PerformanceMeasure:
    os = 0
    rise_time = 0
    pv = deque([0,0,0])
    setpoint = 1
    initial_time = time.time()
    def overshoot(self,process_value):
        self.pv.rotate(-1)
        self.pv[2] = process_value
        if self.setpoint >= 0 and (self.pv[1] > self.pv[0] and self.pv[1] > self.pv[2]) and self.pv[1] > self.setpoint:
            return abs((self.pv[1] - self.setpoint)/self.setpoint)*100
        elif self.setpoint < 0 and (self.pv[1] < self.pv[0] and self.pv[1] < self.pv[2]) and self.pv[1] < self.setpoint:
            return abs((self.pv[1] - self.setpoint)/self.setpoint)*100
        else:
            return -1
    def riseTime(self,process_value):
        if self.setpoint >= 0:
            if process_value > self.setpoint*0.9:
                return (time.time() - self.initial_time)
            else:
                return -1
        else:
            if process_value < self.setpoint*0.9:
                return (time.time() - self.initial_time)
            else:
                return -1
    def reset(self,setpoint_):
        self.os = 0
        self.rise_time = 0
        self.pv = deque([0,0,0])
        if setpoint_ is 0:                                                                          #To avoid division by zero
            setpoint_ = 0.1
        self.setpoint = setpoint_
        self.initial_time = time.time()

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

if __name__ == '__main__':
    try:
        #Joint instances
        front_joint = Joint(f_topic,None)
        rear_joint = Joint(r_topic,None)
        #Performance measure instances
        pm_f = PerformanceMeasure()
        pm_r = PerformanceMeasure()
        pm_f.reset(front_setpoint)
        pm_r.reset(rear_setpoint)
        overshoot_f = -1
        overshoot_r = -1
        rise_time_f = -1
        rise_time_r = -1        

        #Variables used for plotting setpoint and process values
        plt_front_angle = deque([]);
        plt_rear_angle = deque([]);
        plt_ros_front_angle = deque([]);
        plt_ros_rear_angle = deque([]);

        fig, ax = plt.subplots()
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
            global pm_f, pm_r, overshoot_f, overshoot_r, rise_time_f, rise_time_r
            lock.acquire()
            try:
                global_x,global_y = trajectory_generation(time.time()-initial_time)
                front_setpoint,rear_setpoint = inverseKinematics(global_x,global_y)   #Calculate joint angles
                if (abs(front_setpoint) <= 170 and rear_setpoint >= -170 and rear_setpoint <= -10):                        #Check limits of angular positions
                    front_joint.pub.publish(np.deg2rad(front_setpoint))
                    rear_joint.pub.publish(np.deg2rad(rear_setpoint+90))
                else:
                    print('Out of bounds!')                   
                os_f = pm_f.overshoot(front_angle)
                rt_f = pm_f.riseTime(front_angle)
                os_r = pm_r.overshoot(rear_angle)
                rt_r = pm_r.riseTime(rear_angle)
                if os_f is not -1 and overshoot_f is -1 and rise_time_f is not -1:
                    overshoot_f = os_f
                if os_r is not -1 and overshoot_r is -1 and rise_time_r is not -1:
                    overshoot_r = os_r
                if rt_f is not -1 and rise_time_f is -1:
                    rise_time_f = rt_f
                if rt_r is not -1 and rise_time_r is -1:
                    rise_time_r = rt_r
            finally:
                lock.release()
            timer = threading.Timer(0.01,controller)
            timer.setDaemon(True)
            timer.start()        
        def reset():
            global front_joint, rear_joint, initial_time, front_setpoint, rear_setpoint, listener
            global pm_f, pm_r, overshoot_f, overshoot_r, rise_time_f, rise_time_r, lock
            global plt_front_angle, plt_rear_angle, plt_ros_front_angle, plt_ros_rear_angle

            lock.acquire()
            try:
                front_joint.pub.publish(0)
                rear_joint.pub.publish(0)
                rospy.wait_for_service('/gazebo/reset_world')
                reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
                reset_simulation()
                print('Resetting wait for 5 seconds')
                time.sleep(5)
                plt_front_angle = deque([]);
                plt_rear_angle = deque([]);
                plt_ros_front_angle = deque([]);
                plt_ros_rear_angle = deque([]);
                pm_f.reset(front_setpoint)
                pm_r.reset(rear_setpoint)
                overshoot_f = -1
                overshoot_r = -1
                rise_time_f = -1
                rise_time_r = -1
                initial_time = time.time()
                front_setpoint = 0
                rear_setpoint = -90
            finally:
                lock.release()
        def user_input():
            input_string = input('Enter "RESET" (with double quotes) to reset sim env: ')
            if input_string is "RESET":
                reset()
            user_input_timer = threading.Timer(2,user_input)
            user_input_timer.setDaemon(True)
            user_input_timer.start()

        user_input_timer = threading.Timer(2,user_input)
        user_input_timer.setDaemon(True)
        user_input_timer.start()

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
        fig.show()
        #PID loop
        while not rospy.is_shutdown():
            time.sleep(0.5)
            lock.acquire()
            try:
                #Plotting routine
                if(len(plt_front_angle) < plt_length) :
                    plt_front_angle.append(front_setpoint);
                else:
                    plt_front_angle.rotate(-1);
                    plt_front_angle[plt_length-1] = front_setpoint;
                if(len(plt_rear_angle) < plt_length) :
                    plt_rear_angle.append(rear_setpoint);
                else:
                    plt_rear_angle.rotate(-1);
                    plt_rear_angle[plt_length-1] = rear_setpoint;

                if(len(plt_ros_front_angle) < plt_length) :
                    plt_ros_front_angle.append(front_angle);
                else:
                    plt_ros_front_angle.rotate(-1);
                    plt_ros_front_angle[plt_length-1] = front_angle;
                if(len(plt_ros_rear_angle) < plt_length) :
                    plt_ros_rear_angle.append(rear_angle);
                else:
                    plt_ros_rear_angle.rotate(-1);
                    plt_ros_rear_angle[plt_length-1] = rear_angle;
                ax.cla()
                ax.set_xlim([0,plt_length])
                ax.set_ylim([-150,150])
                plt_time = [i for i in range(len(plt_front_angle))]
                ax.plot(plt_time,plt_ros_front_angle,'r',plt_time,plt_ros_rear_angle,'b',
                    plt_time,plt_front_angle,'r+',plt_time,plt_rear_angle,'b+')
                ax.text(5,150,'Overshoot F = '+str(overshoot_f))
                ax.text(5,130,'Overshoot R = '+str(overshoot_r))
                ax.text(5,110,'Rise time F = '+str(rise_time_f))
                ax.text(5,90,'Rise time R = '+str(rise_time_r))
                ax.legend(['Front Angle PV','Rear Angle PV', 'Front Angle SP', 'Rear Angle SP'])
                fig.canvas.flush_events()
                fig.canvas.draw()   
                time.sleep(0.000001)
                pass
            finally:
                lock.release()
    except (KeyboardInterrupt,SystemExit):
        gz_launch.shutdown()
        rcon_launch.shutdown()
        sys.exit()
        f.close()
        raise Exception('Closing program...')
