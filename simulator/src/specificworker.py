#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2020 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os
import threading
import time
import ast
import random
import numpy as np
import json
import cv2
import signal
import numpy
import math 
# from pyrep.objects.object import Object

import _pickle as pickle

from collections import namedtuple

sys.path.append(os.path.join(os.path.dirname(__file__),'../../../../SocNavGym'))
import socnavgym
import gym

EntityObs = namedtuple("EntityObs", ["id", "x", "y", "theta", "sin_theta", "cos_theta"])

# sys.path.append('../python')

# from sonata import SODA

from genericworker import *

import signal

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, WorldFR = False):
        super(SpecificWorker, self).__init__(proxy_map)

        print("launching simulator")
        self.data = {
        'walls': [],
        'goal': [],
        'humans': None,
        'robot_position': None,
        'robot_orientation': None,
        'simulator_mutex': threading.RLock()
        }

        if WorldFR:
            self.frame_of_reference = 'W'
        else:
            self.frame_of_reference = 'R'

        self.env = gym.make("SocNavGym-v1", config=os.path.join(os.path.dirname(__file__),"../etc/socnavgym_conf.yaml")) 
        obs, _ = self.env.reset()


        #Loop
        self.adv = self.rot = 0.
        self.last_ten = time.time()-10
        self.last_point_one = time.time()-10
        self.end_simulation = False
        self.reset = False
        self.terminated_episode = False

        #Metrics
        self.init_globa_metrics()

    def __del__(self):
        print('SpecificWorker destructor')
        # del self.soda

    def stop_simulation(self):
        self.end_simulation = True

    def setParams(self, params):
        return True

    def compute(self):
        # for i in range(5000):
        total_episodes = 500
        num_episodes = 0
        self.init_episode_metrics()
        while not self.end_simulation:
            if not self.terminated_episode:
                obs, reward, terminated, truncated, info = self.env.step([self.adv,0,self.rot]) 

                self.terminated_episode = terminated or truncated

                self.update_episode_metrics(self.terminated_episode, info, reward)

                observations = self.get_observations()
                people, objects, walls, interactions, goal = self.get_data(observations, self.terminated_episode)

                self.peopledetector_proxy.gotpeople(people)
                self.objectdetector_proxy.gotobjects(objects)
                self.interactiondetector_proxy.gotinteractions(interactions)
                self.walldetector_proxy.gotwalls(walls)
                self.goalpublisher_proxy.goalupdated(goal)

                
                image = self.env.render_without_showing()
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB )
                image = image.astype(np.uint8)
                byte_stream = pickle.dumps(image)
                self.bytesequencepublisher_proxy.newsequence(byte_stream)

            if self.reset: #terminated or truncated or self.reset:
                num_episodes += 1
                if num_episodes>total_episodes:
                    print('---------------------------')
                    self.show_global_metrics(total_episodes)
                else:

                    if self.min_human_dist != float('inf') and self.steps>100:
                        self.update_global_metrics()
                        print('---------------------------')
                        self.show_global_metrics(num_episodes)
                    else:
                        num_episodes -= 1
                    print("num_episodes", num_episodes)                        
                    self.init_episode_metrics()
                    self.env.reset()
                    self.terminated_episode = False
                    self.reset = False

            # time.sleep(0.05)                
        return True

    def next_orientation(self):
        robot_goal, _ = self.get_pose([self.env.robot.goal_x, self.env.robot.goal_y], 0)
        # robot_goal = self.env.get_robot_frame_coordinates(np.array([[self.env.robot.goal_x, self.env.robot.goal_y]])).flatten()

        goal = GoalT()
        goal.x = -robot_goal[1]
        goal.y = robot_goal[0]

        robot = ObjectT()
        robot.id = -2
        robot.x = self.env.robot.x
        robot.y = self.env.robot.y
        robot.angle = -(np.pi/2 + self.env.robot.orientation)
        print('robot angle', robot.angle, 'goal angle', math.atan2(goal.x-robot.x, goal.y-robot.y))

        target_orientation = math.atan2(-(goal.x-robot.x), -(goal.y-robot.y))
        dif_orientation = -math.atan2(math.sin(target_orientation-robot.angle), math.cos(target_orientation-robot.angle))

        if abs(dif_orientation)>0.2:
            rot = dif_orientation
        else:
            rot = 0

        print('diff_orientation', dif_orientation, 'rot', rot)
        return rot


    def get_pose(self, pos, angle):
        if self.frame_of_reference == 'R':
            ret_pos = pos
            ret_pos[0] = ret_pos[0] - self.env.robot.x
            ret_pos[1] = ret_pos[1] - self.env.robot.y
            ret_angle = -np.pi/2+angle
            return [ret_pos[1], -ret_pos[0]], ret_angle

            # ret_pos = self.env.get_robot_frame_coordinates(np.array([pos], dtype=np.float32)).flatten()
            # ret_angle = angle - self.env.robot.orientation
            # return [ret_pos[0], ret_pos[1]], ret_angle
        else:
            ret_pos = pos
            ret_angle = -np.pi/2+angle
            return [ret_pos[1], -ret_pos[0]], ret_angle

    def get_observations(self):
        observations = {}
        # adding humans, tables, laptops, plants, walls
        for entity in self.env.static_humans + self.env.dynamic_humans + self.env.tables + self.env.laptops + self.env.plants + self.env.walls:
            coordinates, angle = self.get_pose([entity.x, entity.y], entity.orientation)
            sin_theta = np.sin(angle)
            cos_theta = np.cos(angle)

            # coordinates = self.env.get_robot_frame_coordinates(np.array([[entity.x, entity.y]], dtype=np.float32)).flatten()
            # sin_theta = np.sin(entity.orientation - self.env.robot.orientation)
            # cos_theta = np.cos(entity.orientation - self.env.robot.orientation)
            theta = np.arctan2(sin_theta, cos_theta)
            observations[entity.id] = EntityObs(
                entity.id,
                coordinates[0],
                coordinates[1],
                theta,
                sin_theta,
                cos_theta
            )

        # adding human-human interactions
        for i in self.env.moving_interactions + self.env.static_interactions:
            for entity in i.humans:
                coordinates, angle = self.get_pose([entity.x, entity.y], entity.orientation)
                sin_theta = np.sin(angle)
                cos_theta = np.cos(angle)

                # coordinates = self.env.get_robot_frame_coordinates(np.array([[entity.x, entity.y]], dtype=np.float32)).flatten()
                # sin_theta = np.sin(entity.orientation - self.env.robot.orientation)
                # cos_theta = np.cos(entity.orientation - self.env.robot.orientation)
                theta = np.arctan2(sin_theta, cos_theta)
                observations[entity.id] = EntityObs(
                    entity.id,
                    coordinates[0],
                    coordinates[1],
                    theta,
                    sin_theta,
                    cos_theta
                )
        
        # adding human-laptop interactions
        for i in self.env.h_l_interactions:
            entity = i.human
            coordinates, angle = self.get_pose([entity.x, entity.y], entity.orientation)
            sin_theta = np.sin(angle)
            cos_theta = np.cos(angle)

            # coordinates = self.env.get_robot_frame_coordinates(np.array([[entity.x, entity.y]], dtype=np.float32)).flatten()
            # sin_theta = np.sin(entity.orientation - self.env.robot.orientation)
            # cos_theta = np.cos(entity.orientation - self.env.robot.orientation)
            theta = np.arctan2(sin_theta, cos_theta)
            observations[entity.id] = EntityObs(
                entity.id,
                coordinates[0],
                coordinates[1],
                theta,
                sin_theta,
                cos_theta
            )

            entity = i.laptop
            coordinates, angle = self.get_pose([entity.x, entity.y], entity.orientation)
            sin_theta = np.sin(angle)
            cos_theta = np.cos(angle)

            # coordinates = self.env.get_robot_frame_coordinates(np.array([[entity.x, entity.y]], dtype=np.float32)).flatten()
            # sin_theta = np.sin(entity.orientation - self.env.robot.orientation)
            # cos_theta = np.cos(entity.orientation - self.env.robot.orientation)
            theta = np.arctan2(sin_theta, cos_theta)
            observations[entity.id] = EntityObs(
                entity.id,
                coordinates[0],
                coordinates[1],
                theta,
                sin_theta,
                cos_theta
            )
        return observations

    def get_data(self, observations, terminated=False):
            people = []
            if terminated:
                person = Person()
                person.id = -100
                people.append(person)

            for human in self.env.static_humans + self.env.dynamic_humans:
                human_obs = observations[human.id]
                person = Person()
                person.id = human.id
                person.x = -human_obs.y
                person.y = human_obs.x
                person.angle = -(np.pi/2 + np.arctan2(human_obs.sin_theta, human_obs.cos_theta))
                people.append(person)

            objects = []
            for object in self.env.laptops + self.env.tables:
                obs = observations[object.id]
                obj = ObjectT()
                obj.id = obs.id
                obj.x = -obs.y
                obj.y = obs.x
                obj.angle = -(np.pi/2 + np.arctan2(obs.sin_theta, obs.cos_theta))
                obj.bbx1 = -object.length/2
                obj.bbx2 = object.length/2
                obj.bby1 = -object.width/2
                obj.bby2 = object.width/2
                objects.append(obj)
            for object in self.env.plants:
                obs = observations[object.id]
                obj = ObjectT()
                obj.id = obs.id
                obj.x = -obs.y
                obj.y = obs.x
                obj.angle = -(np.pi/2 + np.arctan2(obs.sin_theta, obs.cos_theta))
                obj.bbx1 = -object.radius
                obj.bbx2 = object.radius
                obj.bby1 = -object.radius
                obj.bby2 = object.radius
                objects.append(obj)

            walls = []
            for wall in self.env.walls:
                w = WallT()
                x1 = wall.x - np.cos(wall.orientation)*wall.length/2
                x2 = wall.x + np.cos(wall.orientation)*wall.length/2
                y1 = wall.y - np.sin(wall.orientation)*wall.length/2
                y2 = wall.y + np.sin(wall.orientation)*wall.length/2
                a1, _ = self.get_pose([x1, y1], 0)
                a2, _ = self.get_pose([x2, y2], 0)
                # a1 = self.env.get_robot_frame_coordinates(np.array([[x1, y1]])).flatten()
                # a2 = self.env.get_robot_frame_coordinates(np.array([[x2, y2]])).flatten()

                w.x1 = -a1[1]
                w.y1 = a1[0]
                w.x2 = -a2[1]
                w.y2 = a2[0]
                walls.append(w)

            interactions = []
            for interaction in self.env.moving_interactions + self.env.static_interactions + self.env.h_l_interactions:
                if interaction.name == "human-human-interaction":
                    for human in interaction.humans:
                        human_obs = observations[human.id]
                        person = Person()
                        person.id = human.id
                        person.x = -human_obs.y
                        person.y = human_obs.x
                        person.angle = -(np.pi/2 + np.arctan2(human_obs.sin_theta, human_obs.cos_theta))
                        people.append(person)

                    for i in range(len(interaction.humans)):
                        for j in range(i+1, len(interaction.humans)):
                            inter = InteractionT()
                            inter.idSrc = interaction.humans[i].id
                            inter.idDst = interaction.humans[j].id
                            inter.type = "human-human-interaction"
                            interactions.append(inter)

                
                if interaction.name == "human-laptop-interaction":
                    human = interaction.human
                    laptop = interaction.laptop

                    human_obs = observations[human.id]
                    person = Person()
                    person.id = human.id
                    person.x = -human_obs.y
                    person.y = human_obs.x
                    person.angle = -(np.pi/2 + np.arctan2(human_obs.sin_theta, human_obs.cos_theta))
                    people.append(person)


                    obs = observations[laptop.id]
                    obj = ObjectT()
                    obj.id = obs.id
                    obj.x = -obs.y
                    obj.y = obs.x
                    obj.angle = -(np.pi/2 + np.arctan2(obs.sin_theta, obs.cos_theta))
                    obj.bbx1 = -laptop.length/2
                    obj.bbx2 = laptop.length/2
                    obj.bby1 = -laptop.width/2
                    obj.bby2 = laptop.width/2
                    objects.append(obj)

                    inter = InteractionT()
                    inter.idSrc = human.id
                    inter.idDst = laptop.id
                    inter.type = "human-laptop-interaction"
                    interactions.append(inter)
            
            robot_goal, _ = self.get_pose([self.env.robot.goal_x, self.env.robot.goal_y], 0)
            # robot_goal = self.env.get_robot_frame_coordinates(np.array([[self.env.robot.goal_x, self.env.robot.goal_y]])).flatten()

            goal = GoalT()
            goal.x = -robot_goal[1]
            goal.y = robot_goal[0]

            robot = ObjectT()
            robot.id = -2
            robot.x = self.env.robot.x
            robot.y = self.env.robot.y
            robot.angle = -(-np.pi/2 + self.env.robot.orientation)
            
            objects.append(robot)


            return people, objects, walls, interactions, goal

    def init_globa_metrics(self):
        self.discomfort_sngnn = 0
        self.discomfort_dsrnn = 0
        self.timeout = 0
        self.success_rate = 0
        self.time_taken = 0
        self.closest_human_dist = 0
        self.closest_obstacle_dist = 0
        self.collision_rate = 0
        self.collision_rate_human = 0
        self.collision_rate_object = 0
        self.collision_rate_wall = 0
        self.total_psc = 0
        self.total_stl = 0
        self.total_spl = 0
        self.total_failure_to_progress = 0
        self.total_stalled_time = 0
        self.total_path_length = 0
        self.total_vel_min = 0
        self.total_vel_max = 0
        self.total_vel_avg = 0
        self.total_a_min = 0
        self.total_a_max = 0
        self.total_a_avg = 0
        self.total_jerk_min = 0
        self.total_jerk_max = 0
        self.total_jerk_avg = 0
        self.total_avg_obstacle_distance = 0
        self.total_minimum_time_to_collision = 0
        self.total_time_to_reach_goal = 0
        
        self.total_reward = 0

    def update_global_metrics(self):
        self.discomfort_sngnn += self.episode_discomfort_sngnn
        self.discomfort_dsrnn += self.episode_discomfort_dsrnn
        self.timeout += self.has_timed_out
        self.success_rate += self.has_reached_goal
        self.time_taken += self.steps
        self.closest_human_dist += self.min_human_dist
        self.closest_obstacle_dist += self.min_obstacle_dist
        self.collision_rate += self.has_collided
        self.collision_rate_human += self.has_collided_human
        self.collision_rate_object += self.has_collided_object
        self.collision_rate_wall += self.has_collided_wall
        self.total_psc += self.psc
        self.total_stl += self.stl
        self.total_spl += self.spl
        self.total_failure_to_progress += self.failure_to_progress
        self.total_stalled_time += self.stalled_time
        self.total_path_length += self.path_length
        self.total_vel_min += self.vel_min 
        self.total_vel_max += self.vel_max 
        self.total_vel_avg += self.vel_avg 
        self.total_a_min += self.a_min 
        self.total_a_max += self.a_max
        self.total_a_avg += self.a_avg 
        self.total_jerk_min += self.jerk_min 
        self.total_jerk_max += self.jerk_max 
        self.total_jerk_avg += self.jerk_avg
        self.total_avg_obstacle_distance += (self.avg_obstacle_dist / self.count)
        self.total_minimum_time_to_collision += (self.avg_minimum_time_to_collision / self.count)
        self.total_time_to_reach_goal += self.time_to_reach_goal

    
    def init_episode_metrics(self):
        self.episode_reward = 0
        self.has_reached_goal = 0
        self.has_collided = 0
        self.has_collided_human = 0
        self.has_collided_object = 0
        self.has_collided_wall = 0
        self.has_timed_out = 0
        self.steps = 0
        self.count = 0
        self.episode_discomfort_sngnn = 0
        self.episode_discomfort_dsrnn = 0
        self.psc = 0
        self.stl = 0
        self.spl = 0
        self.failure_to_progress = 0
        self.stalled_time = 0
        self.time_to_reach_goal = self.env.EPISODE_LENGTH
        self.path_length = 0
        self.vel_min = 0
        self.vel_max = 0
        self.vel_avg = 0
        self.a_min = 0
        self.a_max = 0
        self.a_avg = 0
        self.jerk_min = 0
        self.jerk_max = 0
        self.jerk_avg = 0
        self.min_human_dist = float('inf')
        self.min_obstacle_dist = float('inf')
        self.avg_obstacle_dist = 0
        self.avg_minimum_time_to_collision = 0

    def update_episode_metrics(self, done, info, reward):
        self.total_reward += reward

            # env.render()
        self.steps += 1
        self.count += 1

            # storing the rewards
        self.episode_reward += reward

            # storing discomforts
        self.episode_discomfort_sngnn += info["sngnn_reward"]
        self.episode_discomfort_dsrnn += info["DISCOMFORT_DSRNN"]

            # storing whether the agent reached the goal
        if info["SUCCESS"]:
            self.has_reached_goal = 1
            self.stl = info["STL"]
            self.spl = info["SPL"]
            self.time_to_reach_goal = info["TIME_TO_REACH_GOAL"]
            
        if info["COLLISION"]:
            self.has_collided = 1
            if info["COLLISION_HUMAN"]:
                self.has_collided_human = 1
            if info["COLLISION_OBJECT"]:
                self.has_collided_object = 1
            if info["COLLISION_WALL"]:
                self.has_collided_wall = 1

            self.steps = self.env.EPISODE_LENGTH
            
        if info["TIMEOUT"]:
            self.has_timed_out = 1

        self.min_human_dist = min(self.min_human_dist, info["MINIMUM_DISTANCE_TO_HUMAN"])
        self.min_obstacle_dist = min(self.min_obstacle_dist, info["MINIMUM_OBSTACLE_DISTANCE"])
        self.avg_obstacle_dist += info["AVERAGE_OBSTACLE_DISTANCE"]
        if info["TIME_TO_COLLISION"] != -1: 
            self.avg_minimum_time_to_collision += info["TIME_TO_COLLISION"]
        else: 
            self.avg_minimum_time_to_collision += self.env.EPISODE_LENGTH
        
        self.episode_reward += reward
            
            
        if done:
            self.psc = info["PERSONAL_SPACE_COMPLIANCE"]
            self.failure_to_progress = info["FAILURE_TO_PROGRESS"]
            self.stalled_time = info["STALLED_TIME"]
            self.path_length = info["PATH_LENGTH"]
            self.vel_min = info["V_MIN"]
            self.vel_avg = info["V_AVG"]
            self.vel_max = info["V_MAX"]
            self.a_min = info["A_MIN"]
            self.a_avg = info["A_AVG"]
            self.a_max = info["A_MAX"]
            self.jerk_min = info["JERK_MIN"]
            self.jerk_avg = info["JERK_AVG"]
            self.jerk_max = info["JERK_MAX"]


    def show_global_metrics(self,num_episodes):
        print(f"Average discomfort_sngnn: {self.discomfort_sngnn/num_episodes}") 
        print(f"Average discomfort_dsrnn: {self.discomfort_dsrnn/num_episodes}") 
        
        print(f"Average success_rate: {self.success_rate/num_episodes}") 
        print(f"Average collision_rate: {self.collision_rate/num_episodes}")
        print(f"Average wall_collision_rate: {self.collision_rate_wall/num_episodes}")
        print(f"Average object_collision_rate: {self.collision_rate_object/num_episodes}")
        print(f"Average human_collision_rate: {self.collision_rate_human/num_episodes}")
        print(f"Average timeout: {self.timeout/num_episodes}") 
        print(f"Average time_taken: {self.time_taken/num_episodes}") 
        print(f"Average failure_to_progress: {self.total_failure_to_progress/num_episodes}")
        print(f"Average stalled_time: {self.total_stalled_time/num_episodes}")
        print(f"Average time_to_reach_goal: {self.total_time_to_reach_goal/num_episodes}")
        print(f"Average path_length: {self.total_path_length/num_episodes}")
        print(f"Average stl: {self.total_stl/num_episodes}")
        print(f"Average spl: {self.total_spl/num_episodes}")
        
        print(f"Average vel_min: {self.total_vel_min/num_episodes}")
        print(f"Average vel_avg: {self.total_vel_avg/num_episodes}")
        print(f"Average vel_max: {self.total_vel_max/num_episodes}")
        print(f"Average a_min: {self.total_a_min/num_episodes}")
        print(f"Average a_avg: {self.total_a_avg/num_episodes}")
        print(f"Average a_max: {self.total_a_max/num_episodes}")
        print(f"Average jerk_min: {self.total_jerk_min/num_episodes}")
        print(f"Average jerk_avg: {self.total_jerk_avg/num_episodes}")
        print(f"Average jerk_max: {self.total_jerk_max/num_episodes}")
        print(f"Average closest_obstacle_dist: {self.closest_obstacle_dist/num_episodes}") 
        print(f"Average average_obstacle distance: {self.total_avg_obstacle_distance/num_episodes}")
        print(f"Average psc: {self.total_psc/num_episodes}")
        print(f"Average closest_human_dist: {self.closest_human_dist/num_episodes}") 
        print(f"Average minimum_time_to_collision: {self.total_minimum_time_to_collision/num_episodes}")


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # correctOdometer
    #
    def OmniRobot_correctOdometer(self, x, z, alpha):
        #
        # implementCODE
        #
        pass


    #
    # getBasePose
    #
    def OmniRobot_getBasePose(self):
        #
        # implementCODE
        #
        # self.soda.data['simulator_mutex'].acquire()
        x = int(0)
        z = int(0)
        alpha = float(0)
        # self.soda.data['simulator_mutex'].release()
        return [x, z, alpha]


    #
    # getBaseState
    #
    def OmniRobot_getBaseState(self):
        #
        # implementCODE
        #
        state = RoboCompGenericBase.TBaseState()
        # self.soda.data['simulator_mutex'].acquire()
        state.x = int(0)
        state.z = int(0)
        state.alpha = float(0)
        # self.soda.data['simulator_mutex'].release()
        return state


    #
    # resetOdometer
    #
    def OmniRobot_resetOdometer(self):
        #
        # implementCODE
        #
        pass


    #
    # setOdometer
    #
    def OmniRobot_setOdometer(self, state):
        #
        # implementCODE
        #
        pass


    #
    # setOdometerPose
    #
    def OmniRobot_setOdometerPose(self, x, z, alpha):
        #
        # implementCODE
        #
        pass


    #
    # setSpeedBase
    #
    def OmniRobot_setSpeedBase(self, advx, advz, rot):
        self.adv = advx
        self.rot = rot
        # self.soda.data['simulator_mutex'].acquire()
        # radius = 0.0475 # youbot's weel radius
        # rotation_to_linear_ratio = 2. * pi * radius
        # #print(rotation_to_linear_ratio)
        # self.soda.robot.set_base_angular_velocites([advx/rotation_to_linear_ratio, advz/rotation_to_linear_ratio, rot])
        # self.soda.data['simulator_mutex'].release()


    #
    # stopBase
    #
    def OmniRobot_stopBase(self):
        self.OmniRobot_setSpeedBase(0., 0., 0.)


    #
    # Reset simulation
    #
    def Simulator_regenerate(self, scene):
        self.reset = True

        # print('We should reinitialise the simulation')
        # print("scene", scene)
        # self.min_max_data = ast.literal_eval(scene)
        # if 'robot_random_pose' in self.min_max_data.keys() and self.min_max_data['robot_random_pose'] == 0:
        #     robot_random_pose = False
        # else:
        #     robot_random_pose = True
        # if 'show_goal' in self.min_max_data.keys() and self.min_max_data['show_goal'] == 0:
        #     show_goal = False
        # else:
        #     show_goal = True
        # if 'show_relations' in self.min_max_data.keys() and self.min_max_data['show_relations'] == 0:
        #     show_relations = False
        # else:
        #     show_relations = True
        # print(self.min_max_data)
        # self.soda.data['simulator_mutex'].acquire()
        # self.data, self.wandering_humans = self.soda.room_setup(self.min_max_data['min_humans'],
        #                                                         self.min_max_data['min_wandering_humans'],
        #                                                         self.min_max_data['min_plants'],
        #                                                         self.min_max_data['min_tables'],
        #                                                         self.min_max_data['min_relations'],
        #                                                         self.min_max_data['max_humans'],
        #                                                         self.min_max_data['max_wandering_humans'],
        #                                                         self.min_max_data['max_plants'],
        #                                                         self.min_max_data['max_tables'],
        #                                                         self.min_max_data['max_relations'],
        #                                                         robot_random_pose,
        #                                                         show_goal,
        #                                                         show_relations)
        # self.soda.data['simulator_mutex'].release()



    # ===================================================================
    # ===================================================================
