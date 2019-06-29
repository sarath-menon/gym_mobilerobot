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
import random
import time
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32MultiArray
from combination_obstacle_1 import Combination
from gazebo_msgs.msg import ModelState, ModelStates

#---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))

class Respawn(Combination):
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = dirPath + '/goal_box/model.sdf'
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.stage = rospy.get_param('/env_type')
        self.goal_position = Pose()
        self.init_goal_x = 0.6
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        # self.obstacle_1 = -0.98, -0.49
        # self.obstacle_2 = 0.96, -0.82
        # self.obstacle_3 = -0.11, 0.87
        self.obstacles = {'cylinder_1': [-0.98,-0.49], 'cylinder_2': [0.96,-0.82], 'cylinder_3': [-0.11,0.87]}
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.last_index = 0
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.pub = rospy.Publisher('goal_position', Float32MultiArray, queue_size=10)
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.check_model = False
        self.index = 0

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                              self.goal_position.position.y)
                break
            else:
                pass

    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass

    def move_obstacles(self, goal_x, goal_y):
        model = rospy.wait_for_message('gazebo/model_states', ModelStates)
        count=0
        for i in range(len(model.name)):
            if count==2 : break
            if model.name[i] == 'cylinder_1' or model.name[i] == 'cylinder_2'or model.name[i] == 'cylinder_3':
                print(model.name[i])
                obstacle_1 = ModelState()
                obstacle_1.model_name = model.name[i]
                obstacle_1.pose = model.pose[i]

                obstacle_1.pose.position.x = random.randrange(-15, 17) / 10.0
                obstacle_1.pose.position.y = random.randrange(-15, 17) / 10.0
                self.obstacles[model.name[i]] = [obstacle_1.pose.position.x, obstacle_1.pose.position.y]
                print(count)
                count+=1
                self.pub_model.publish(obstacle_1)
                time.sleep(0.5)


    def getPosition(self, position_check=False, delete=False):
        if delete:
            self.deleteModel()

        if self.stage == 'static_obstacle' or 'dynamic_obstacle':
            while position_check:
                goal_x = random.randrange(-12, 13) / 10.0
                goal_y = random.randrange(-12, 13) / 10.0
                if abs(goal_x - obstacles['cylinder_1'][0]) <= 0.6 and abs(goal_y - obstacles['cylinder_1'][1]) <= 0.6:
                    position_check = True
                elif abs(goal_x - obstacles['cylinder_2'][0]) <= 0.6 and abs(goal_y - obstacles['cylinder_2'][1]) <=0.6:
                    position_check = True
                elif abs(goal_x - obstacles['cylinder_3'][0]) <= 0.6 and abs(goal_y - obstacles['cylinder_3'][1]) <=0.6:
                    position_check = True
                elif abs(goal_x - 0.0) <= 0.4 and abs(goal_y - 0.0) <= 0.4:
                    position_check = True
                else:
                    position_check = False

                if abs(goal_x - self.last_goal_x) < 1 and abs(goal_y - self.last_goal_y) < 1:
                    position_check = True

        self.goal_position.position.x = goal_x
        self.goal_position.position.y = goal_y


        data = Float32MultiArray()
        data.data = [goal_x, goal_y]
        self.pub.publish(data)

        if self.stage=='dynamic_obstacle':self.move_obstacles(self.goal_position.position.x, self.goal_position.position.y)

        time.sleep(0.5)
        self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
