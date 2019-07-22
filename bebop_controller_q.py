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
import qagent

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
    global STATE_MANAGER, QAGENT
    global PUB_TAKEOFF, PUB_LANDING, MSG_EMPTY
    global qa
    zero_vel = Twist(Vector3(0,0,0),Vector3(0.0, 0.0, 0.0))

    with NonBlockingConsole() as nbc:
        while IS_RUNNING:
            c = nbc.get_data()
            if c == '\x1b':  # x1b is ESC
                IS_KEYREADER_FINISHED = True
                PUB.publish(zero_vel)
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
            elif c =='p':
                print(qa.qmatrix.tolist())
            elif c == 's':
                qa.save_qmatrix('qmatrix.csv')
                print "Qmatrix saved as 'qmatrix.csv'"

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

def print_data(current_target, speed, pid, reward):
    print "----------------------------------------"
    print "target: ", current_target
    # print "current: ",  round(self.current_x,3), round(self.current_y,3), round(self.current_z,3), round(self.current_yaw,3)
    # print "error: ",    round(error_x,3),   round(error_y,3),   round(error_z,3),   round(error_yaw,3)
    print "speed: ", round(speed[0],3), round(speed[1],3), round(speed[2],3)
    print "pid: ", pid
    print "reward: ", reward

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

def compute_speed(last_time, last_err, current_err, err_sum, pid):
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
        speed[i] = (pid[0] * current_err[i] + pid[1] * err_sum[i] + pid[2] * dErr[i])

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
        STATE_MANAGER.set_state(state_manager.COPTER_STATE_LANDING, rospy.get_time())
        print "Cannot Update Odometry >> Landing"
    PUB.publish(twist_current)

def get_dist_from_optimal_path(target, next_target, pose):
    v = [next_target[0] - target[0], next_target[1] - target[1], target[2] - target[2]]
    v0 = target
    pt = [pose[0], pose[1], pose[2]]
    ptv0 = [pt[0]-v0[0], pt[1]-v0[1], pt[2]-v0[2]]
    ptv0v = [(ptv0[1]*v[2] - ptv0[2]*v[1]), - (ptv0[0]*v[2] - ptv0[2]*v[0]), + (ptv0[0]*v[1] - ptv0[1]*v[0])]
    d = (math.sqrt(ptv0v[0]**2+ptv0v[1]**2+ptv0v[2]**2))/math.sqrt(v[0]**2+v[1]**2+v[2]**2)
    return d

def get_qstate(d, speed_xyz):
    global SPEED_LIMIT
    
    if speed_xyz < SPEED_LIMIT/2:
        if d > 1:
            qstate = 0
        elif d > 0.5:
            qstate = 1
        else:
            qstate = 2
    else:
        if d > 1:
            qstate = 3
        elif d > 0.5:
            qstate = 4
        else:
            qstate = 5
    return qstate

def get_action(state, qagent):
    actions = [
    [0.1,0,0],
    [0.1,0.01,0],
    [0.1,0.01,0.2],
    [0.15,0.02,0.25],
    [0.10,0.01,0.25],
    [0.15,0.02,0.30],
    [0.20,0.02,0.25],
    [0.15,0.02,0.1]
    ]
    index = qagent.chose_action(state)
    return (index, actions[index])

def compute_reward(dist_reward, speed_xyz):
    if dist_reward > 1 or dist_reward < 0.5:
        reward = 1 - dist_reward
    else:
        reward = 0
    if speed_xyz > 0.5 and reward > 0:
        reward = reward *2

def run(file_path='flight_data/'):
    global IS_RUNNING, IS_KEYREADER_FINISHED, STATE_MANAGER
    global odom_current, qa

    IS_RUNNING = True
    IS_KEYREADER_FINISHED = False
    thread_reader = threading.Thread(target=key_reader)
    thread_reader.start()
    file_name = file_path + str(int(time.time()))
    targets = waypoints.points   
    rate = rospy.Rate(5)    
    twist_current = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    qa = qagent.QAgent()
    qa.qmatrix_from_file('qmatrix.csv')
    
    speed_xyz = 0
    target_index = 0
    error_xyz = 1
    last_time = 0
    last_error = [0,0,0]
    error_sum = [0,0,0]
    reward = 0

    print 't - takeoff, n - navigate, l - land'
    print 'p - print qmatrix, s - save qmatrix, esc - quit'

    while not rospy.is_shutdown() and not IS_KEYREADER_FINISHED:

        current_state = STATE_MANAGER.get_state(rospy.get_time())


        if current_state == state_manager.COPTER_STATE_NAVIGATING:

            target_index = get_target_index(target_index, targets, error_xyz)
            target = targets[target_index]
            next_target = targets[target_index + 1]
            pose = update_pose(odom_current)
            error = update_error(target, pose)
            error_xyz = math.sqrt(error[0] **2 + error[1] **2 + error[2] **2)
            
            dist_reward = get_dist_from_optimal_path(target, next_target, pose)
            state = get_qstate(dist_reward, speed_xyz)
            (action_index, pid) = get_action(state, qa)
            compute_reward(dist_reward, speed_xyz)

            (speed, last_time, last_error, error, error_sum) = compute_speed(last_time, last_error, error, error_sum, pid)
            speed_xyz = (speed[0] + speed[1] + speed[2])/3
            publish_move(speed)
            
            print_data(target, speed, pid, reward)
            save_data(file_name, pose, error, target)
            next_state = get_qstate(get_dist_from_optimal_path(target, next_target, pose), speed_xyz)
            qa.update_q(state, action_index, next_state, reward)

        rate.sleep()

    IS_RUNNING = False
    
   
if __name__ == '__main__':
    try:
        rospy.init_node('bebop_position_controller', anonymous=True)
        odom_current  = Odometry()
        odom_time = rospy.get_time()
        rospy.Subscriber("bebop/odom", Odometry, odometry_callback)
        run()

    except rospy.ROSInterruptException:
        pass