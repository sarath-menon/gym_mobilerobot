#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from goal_real_amcl import Respawn
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import PoseWithCovarianceStamped


import gym
from gym import spaces
# from gym.envs.classic_control import rendering
from gym.utils import seeding

class MobileRobotGymEnv_real_fusion(gym.Env):
    def __init__(self):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.respawn_goal = Respawn()
        self.past_distance = 0.
        self.past_action = np.array([0.,0.])
        obs_high = np.concatenate((np.array([3.5] * 10) ,np.array([0.22, 2.0]), np.array([10.]), np.array([10.]) ))
        obs_low = np.concatenate((np.array([0.] * 10) ,np.array([0.0, -2.0]), np.array([-10.]), np.array([-10.]) ))
        self.action_space = spaces.Box(low=np.array([0., -2.0]) ,high=np.array([0.22, 2.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_low , high=obs_high, dtype=np.float32)
        self.count_collision, self.count_goal  = 0,0
        #Keys CTRL + c will stop script
        rospy.on_shutdown(self.shutdown)

    def stats(self):
        return self.count_collision, self.count_goal

    def shutdown(self):
        rospy.loginfo("Stopping TurtleBot")
        self.pub_cmd_vel.publish(Twist())
        rospy.sleep(1)

    def getGoalDistace(self):
        self.getRobotPos()
        print(self.position)
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.past_distance = goal_distance

        return goal_distance

    def getRobotPos(self):
        data = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=15)
        self.position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)
        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 3)

    def getState(self, scan, type='normalized'):
        scan_range = []
        heading = self.heading
        min_range = 0.01
        done = False
        scan_range.append(max(scan.ranges[0:35])) #1
        scan_range.append(max(scan.ranges[36:71])) #2
        scan_range.append(max(scan.ranges[72:107])) #3
        scan_range.append(max(scan.ranges[108:143])) #4
        scan_range.append(max(scan.ranges[144:179])) #5
        scan_range.append(max(scan.ranges[180:215])) #6
        scan_range.append(max(scan.ranges[216:251])) #7
        scan_range.append(max(scan.ranges[252:287])) #8
        scan_range.append(max(scan.ranges[288:323])) #9
        scan_range.append(max(scan.ranges[324:360])) #10


        for i in range(len(scan_range)):
            if scan_range[i] == float('Inf'):
                if type=='normal':scan_range[i] = 3.5
                if type=='normalized':scan_range[i] = 1 # 3.5 / 3.5
            elif np.isnan(scan_range[i]):
                scan_range[i] = 0.
            elif type=='normalized': scan_range[i] /= 3.5

        # print(scan_range)
        # print('Min scan range:',min(scan_range))
        if min_range > min(scan_range) > 0:
            done = True

        for pa in self.past_action:
            scan_range.append(pa)

        self.getRobotPos()

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        print('Current Distance:',current_distance)
        if current_distance < 0.3:
            self.get_goalbox = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done,type='scaled'):
        current_distance = state[-1]
        heading = state[-2]
        distance_rate = (self.past_distance - current_distance)
        if distance_rate > 0:
            if type=='unscaled': reward = 200.*distance_rate
            elif type=='scaled': reward = 0.2*distance_rate
        if distance_rate <= 0:
            if type=='unscaled': reward = -8.
            elif type=='scaled': reward = -0.01

        self.past_distance = current_distance

        if done:
            rospy.loginfo("Collision!!")
            if type=='normal': reward = -500.
            elif type=='scaled': reward = -1.
            self.count_collision+=1
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            if type=='normal': reward = 500.
            elif type=='scaled': reward = 0.95
            self.count_goal+=1
            vel_cmd = Twist()
            vel_cmd.linear.x ,vel_cmd.angular.z = 0., 0.
            self.pub_cmd_vel.publish(vel_cmd)
            self.goal_x, self.goal_y = self.respawn_goal.movebase_client()
            self.goal_distance = self.getGoalDistace()
            self.get_goalbox = False

        return reward

    def step(self, action):
        linear_vel = action[0]
        ang_vel = action[1]

        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        self.past_action = action
        reward = self.setReward(state, done)

        return np.asarray(state), reward, done ,{}

    def reset(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.movebase_client()
            self.initGoal = False
        else:
            self.goal_x, self.goal_y =  self.respawn_goal.movebase_client()

        self.goal_distance = self.getGoalDistace()
        state, done = self.getState(data)

        return np.asarray(state)
