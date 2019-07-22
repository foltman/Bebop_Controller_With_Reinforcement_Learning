#!/usr/bin/env python
import rospy
import numpy as np
import math
import threading
import sys, select, tty, termios
import csv
import tf
import time
import random
import waypoints
import state_manager
import qagent as qa

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

PUB = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
PUB_TAKEOFF = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
PUB_LANDING = rospy.Publisher('/bebop/land', Empty, queue_size=10)
MSG_EMPTY = Empty()
STATE_MANAGER = state_manager.StateManager()
IS_KEYREADER_FINISHED = False
IS_RUNNING = False
SPEED_LIMIT = 0.5


class NonBlockingConsole(object):

    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_data(self):
        try:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
        except:
            return '[CTRL-C]'
        return False

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def key_reader():
    global IS_RUNNING, IS_KEYREADER_FINISHED
    global STATE_MANAGER
    global PUB_TAKEOFF, PUB_LANDING, MSG_EMPTY
    zero_vel = Twist(Vector3(0,0,0),Vector3(0.0, 0.0, 0.0))

    with NonBlockingConsole() as nbc:
        while IS_RUNNING:
            c = nbc.get_data()
            if c == '\x1b':  # x1b is ESC
                IS_KEYREADER_FINISHED = True
                PUB_LANDING.publish(MSG_EMPTY)
                break
            elif c == 'l':
                STATE_MANAGER.set_state(state_manager.COPTER_STATE_LANDING, rospy.get_time())
                PUB.publish(zero_vel)
                PUB_LANDING.publish(MSG_EMPTY)
                print "land"
            elif c == 'n':
                current_state = STATE_MANAGER.get_state(rospy.get_time())
                if current_state == state_manager.COPTER_STATE_HOVERING:
                    print "navigate"
                    STATE_MANAGER.set_state(state_manager.COPTER_STATE_NAVIGATING, rospy.get_time())
                else:
                    print "Not in Hovering State yet"
            elif c == 't':
                STATE_MANAGER.set_state(state_manager.COPTER_STATE_TAKING_OFF, rospy.get_time())
                PUB_TAKEOFF.publish(MSG_EMPTY)
                print "take off"

def odometry_callback(Odometry):
    global odom_current, odom_time
    odom_current  = Odometry()
    odom_time = rospy.get_time()

def odom_updated(max_delay):
    global odom_time
    delay = rospy.get_time() - odom_time
    if delay <= max_delay:
        is_updated = True
    else:
        is_updated = False
    return is_updated

def get_yaw_from_quaternion(quaternion):
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
    return yaw

def print_data(current_target, speed, kp, ki, kd):
    print "----------------------------------------"
    print "target: ", current_target
    # print "current: ",  round(self.current_x,3), round(self.current_y,3), round(self.current_z,3), round(self.current_yaw,3)
    # print "error: ",    round(error_x,3),   round(error_y,3),   round(error_z,3),   round(error_yaw,3)
    print "speed: ", round(speed[0],3), round(speed[1],3), round(speed[2],3)
    print "pid: ", kp, ki, kd

def save_data(file_name, pose, error, target):
    file = open(file_name,'a')
    text = str(pose[0])+','+str(pose[1])+','+str(pose[2])+','+str(error[0])+','+str(error[1])+','+str(error[2])+','+str(target[0])+','+str(target[1])+','+str(target[2])
    file.write(text)
    file.write('\n')
    file.close()

def get_target_index(waypoint_index, points, error):
    if error <= 0.2 and waypoint_index < len(points):
        print "going to next point"
        waypoint_index = waypoint_index + 1
    return waypoint_index

def update_pose(odom_current):
    current_x = odom_current.pose.pose.position.x
    current_y = odom_current.pose.pose.position.y
    current_z = odom_current.pose.pose.position.z
    current_yaw = get_yaw_from_quaternion(odom_current .pose.pose.orientation)
    if current_yaw > math.pi:
        current_yaw = 2*math.pi - current_yaw
    current_pose = [current_x, current_y, current_z, current_yaw]
    return current_pose
    
def update_error (current_target,current_pose):
    error_x = current_target[0] - current_pose[0]
    error_y = current_target[1] - current_pose[1]
    error_z = current_target[2] - current_pose[2]
    current_error = [error_x, error_y, error_z]
    return current_error

def compute_speed(last_time, last_err, current_err, err_sum, kp, ki, kd):
    global SPEED_LIMIT
    current_time = rospy.get_time()
    time_change = current_time - last_time
    dErr=[0,0,0]
    speed = [0,0,0]
    for i in range(len(last_err)):
        if last_time == 0:
            last_time = current_time
            err_sum[i] = 0
            last_err[i] = 0      
        err_sum[i] = (current_err[i] * time_change) + err_sum[i]
        if err_sum[i]  > SPEED_LIMIT:
            err_sum[i] = SPEED_LIMIT
        elif err_sum[i] < (SPEED_LIMIT * -1):
            err_sum[i]  = (SPEED_LIMIT * -1)
        if time_change == 0:
            dErr[i] = 0
        else:
            dErr[i] = (current_err[i] - last_err[i]) / time_change
        speed[i] = (kp * current_err[i] + ki * err_sum[i] + kd * dErr[i])

        last_err[i] = current_err[i]
        last_time = current_time
    return (speed, last_time, last_err, current_err, err_sum)

def publish_move(speed):
    global PUB, PUB_LANDING, MSG_EMPTY, STATE_MANAGER
    if odom_updated(2) == True:
        twist_current = Twist(Vector3(speed[0],speed[1],speed[2]),Vector3(0.0, 0.0, 0.0))
    else:
        twist_current = Twist(Vector3(0,0,0),Vector3(0.0, 0.0, 0.0))
        PUB_LANDING.publish(MSG_EMPTY)
        #STATE_MANAGER.set_state(state_manager.COPTER_STATE_LANDING, rospy.get_time())
        print "Cannot Update Odometry >> Landing"
    PUB.publish(twist_current)

def run(kp = 0.15, ki = 0.02, kd = 0.30, file_path='flight_data/'):
    global IS_RUNNING, IS_KEYREADER_FINISHED, STATE_MANAGER
    global odom_current

    IS_RUNNING = True
    IS_KEYREADER_FINISHED = False
    thread_reader = threading.Thread(target=key_reader)
    thread_reader.start()
    file_name = file_path + str(int(time.time()))
    targets = waypoints.points   
    rate = rospy.Rate(5)    
    twist_current = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    
    target_index = 0
    error_xyz = 1
    last_time = 0
    last_error = [0,0,0]
    error_sum = [0,0,0]

    print 't - takeoff, n - navigate, l - land, esc - quit'

    while not rospy.is_shutdown() and not IS_KEYREADER_FINISHED:

        current_state = STATE_MANAGER.get_state(rospy.get_time())

        if current_state == state_manager.COPTER_STATE_NAVIGATING:

            target_index = get_target_index(target_index, targets, error_xyz)
            target = targets[target_index]
            pose = update_pose(odom_current)
            error = update_error(target, pose)
            error_xyz = math.sqrt(error[0] **2 + error[1] **2 + error[2] **2)      
            (speed, last_time, last_error, error, error_sum) = compute_speed(last_time, last_error, error, error_sum, kp, ki, kd)
            print_data(target, speed, kp, ki, kd)
            save_data(file_name, pose, error, target)
            publish_move(speed)

        rate.sleep()

    IS_RUNNING = False
    
   
if __name__ == '__main__':
    try:
        rospy.init_node('bebop_position_controller', anonymous=True)
        rospy.Subscriber("bebop/odom", Odometry, odometry_callback)
        run()

    except rospy.ROSInterruptException:
        pass

