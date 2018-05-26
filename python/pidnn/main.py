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

class PerformanceMeasure:
    os = 0
    rise_time = 0
    pv = deque([0,0,0])
    setpoint = 1
    initial_time = time.time()
    def overshoot(self,process_value):
        self.pv.rotate(-1)
        self.pv[2] = process_value
        if (self.pv[1] > self.pv[0] and self.pv[1] > self.pv[2]) or (self.pv[1] < self.pv[0] and self.pv[1] < self.pv[2]):
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
        self.setpoint = setpoint_
        self.initial_time = time.time()

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
        #PID instances
        pid_f = PID(0.04,0.01,0.001)
        pid_r = PID(0.04,0.01,0.002)
        #Performance measure instances
        pm_f = PerformanceMeasure()
        pm_r = PerformanceMeasure()
        pm_f.reset(front_setpoint)
        pm_r.reset(rear_setpoint)
        #Variables used for plotting setpoint and process values
        plt_front_angle = deque([]);
        plt_rear_angle = deque([]);
        plt_der_front_angle = deque([]);
        plt_der_rear_angle = deque([]);
        fig, ax = plt.subplots()
        lock = threading.Lock()                                                                 #Thread lock to avoid data corruption
        overshoot_f = -1
        overshoot_r = -1
        rise_time_f = -1
        rise_time_r = -1        
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
            global exit_, lock 
            global front_setpoint, front_angle, rear_setpoint, rear_angle, overshoot_r, overshoot_f, rise_time_r, rise_time_f
            lock.acquire()
            try:
                front_joint.pub.publish(pid_f.pid(front_setpoint,front_angle))
                rear_joint.pub.publish(pid_r.pid(rear_setpoint,rear_angle))
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
            timer = threading.Timer(1,controller)
            timer.setDaemon(True)
            if exit_ is False:
                timer.start()
        def user_input():
            global front_setpoint, rear_setpoint, lock, overshoot_r, overshoot_f, rise_time_r, rise_time_f
            global front_joint, rear_joint
            print('REAR ANGLE = [-170 to -10], FRONT ANGLE = [-170 to 170]. Type "RESET" (with double quotes) to reset')
            front_setpoint_ = input("Front Setpoint:")
            if front_setpoint_ == 'RESET':
                lock.acquire()
                try:
                    front_joint.pub.publish(0)
                    rear_joint.pub.publish(0)
                    print('Reseting.. wait for 5 seconds')
                    time.sleep(5)
                    pm_f.reset(front_setpoint)
                    pm_r.reset(rear_setpoint)
                    overshoot_f = -1
                    overshoot_r = -1
                    rise_time_f = -1
                    rise_time_r = -1
                finally:
                    lock.release()
            else:
                rear_setpoint_ = input("Rear Setpoint:")
                if (isinstance(front_setpoint_,(float,int)) and abs(front_setpoint_) <= 170
                    and isinstance(rear_setpoint_,(float,int)) and rear_setpoint_ >= -170 and rear_setpoint_ <= -10):
                    lock.acquire()
                    try:
                        front_setpoint = front_setpoint_
                        rear_setpoint = rear_setpoint_
                        print('Front setpoint: '+str(front_setpoint)+' and Rear setpoint: '+str(rear_setpoint))
                        pm_f.reset(front_setpoint)
                        pm_r.reset(rear_setpoint)
                        overshoot_f = -1
                        overshoot_r = -1
                        rise_time_f = -1
                        rise_time_r = -1
                    finally:
                        lock.release()
                else:
                    print('Error in input.')
            user_input_timer = threading.Timer(2,user_input)
            user_input_timer.setDaemon(True)
            if exit_ is False:
                user_input_timer.start()

        #new rosnode to publish effort messages from script to ros+gazebo control and latch messages
        #from listeners
        setpointnode = rospy.init_node('SetPoint',anonymous=True)
        front_joint.pub = rospy.topics.Publisher(front_joint.topic,Float64,queue_size=10)
        rear_joint.pub = rospy.topics.Publisher(rear_joint.topic,Float64,queue_size=10)
        listener = rospy.topics.Subscriber('/tf',TFMessage,callback=listener_callback,queue_size = 10)
        time.sleep(2)

        timer = threading.Timer(0.01,controller)                                                   #Parallel thread to transmit angular velocities and Plotting graphs parallely
        timer.setDaemon(True)
        timer.start()                                                                           #Start thread
        
        user_input_timer = threading.Timer(2,user_input)
        user_input_timer.setDaemon(True)
        user_input_timer.start()

        #PID loop
        while not rospy.is_shutdown():
            time.sleep(1)
            lock.acquire()
            try:
                #Plotting routine
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
                ax.set_ylim([-180,180])
                plt_time = [i for i in range(len(plt_front_angle))]
                plt_front_set_angle = [front_setpoint for i in range(len(plt_front_angle))]
                plt_rear_set_angle = [rear_setpoint for i in range(len(plt_rear_angle))]
                ax.plot(plt_time,plt_front_angle,'r',plt_time,plt_rear_angle,'b',
                  plt_time,plt_front_set_angle,'r+',plt_time,plt_rear_set_angle,'b+')
                ax.text(5,150,'Overshoot F = '+str(overshoot_f))
                ax.text(5,130,'Overshoot R = '+str(overshoot_r))
                ax.text(5,110,'Rise time F = '+str(rise_time_f))
                ax.text(5,90,'Rise time R = '+str(rise_time_r))
                ax.legend(['Front Angle PV','Rear Angle PV', 'Front Angle SP', 'Rear Angle SP'])
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
