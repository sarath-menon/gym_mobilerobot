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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState, ModelStates

#---Directory Path---#
dirPath = os.path.dirname(os.path.realpath(__file__))

class Respawn():
    def __init__(self):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = dirPath + '/goal_box/model.sdf'
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.goal_position = Pose()
        self.init_goal_x = 0.0
        self.init_goal_y = 0.0
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.last_goal_x = self.init_goal_x
        self.last_goal_y = self.init_goal_y
        self.modelName = 'goal'

    def respawnModel(self):
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
        rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x,
                      self.goal_position.position.y)

    def movebase_client(self):
        print("Select Goal position in RViz Map")
        data = rospy.wait_for_message('/move_base_simple/goal', PoseStamped, timeout=500)

        self.goal_position.position.x = data.pose.position.x
        self.goal_position.position.y = data.pose.position.y

        self.respawnModel()
        # self.last_goal_x = self.goal_position.position.x
        # self.last_goal_y = self.goal_position.position.y

        return self.goal_position.position.x, self.goal_position.position.y
