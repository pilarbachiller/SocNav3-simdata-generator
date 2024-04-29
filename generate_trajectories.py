import sys
import os
import cv2
import json
import numpy as np
import pygame
import time
import pickle
from PySide2 import QtGui, QtWidgets, QtCore
from mainUI import Ui_MainWindow

sys.path.append(os.path.join(os.path.dirname(__file__),'../SocNavGym'))
import socnavgym
import gym


class MainWindow(QtWidgets.QWidget, Ui_MainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        
        self.init_joystick()

        self.save_dir = './trajectory_dataset/'
        self.data_file_index = 0
        self.update_data_index()
        self.images_for_video = list()
        self.data = list()

        self.env = gym.make("SocNavGym-v1", config="socnavgym_conf.yaml")
        obs, _ = self.env.reset()

        self.quit_button.clicked.connect(self.quit_slot)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.compute)
        self.timer.start(30)

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick_count = pygame.joystick.get_count()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        axes = self.joystick.get_numaxes()

        try:
            with open('joystick_calibration.pickle', 'rb') as f:
                centre, values, min_values, max_values = pickle.load(f)
        except:
            centre = {}
            values = {}
            min_values = {}
            max_values = {}
            for axis in range(self.joystick.get_numaxes()):
                values[axis] = 0.
                centre[axis] = 0.
                min_values[axis] = 0.
                max_values[axis] = 0.
            T = 3.
            print(f'Leave the controller neutral for {T} seconds')
            t = time.time()
            while time.time() - t < T:
                pygame.event.pump()
                for axis in range(axes):
                    centre[axis] = self.joystick.get_axis(axis)
                time.sleep(0.05)
            T = 5.
            print(f'Move the joystick around for {T} seconds trying to reach the max and min values for the axes')
            t = time.time()
            while time.time() - t < T:
                pygame.event.pump()
                for axis in range(axes):
                    value = self.joystick.get_axis(axis)-centre[axis]
                    if value > max_values[axis]:
                        max_values[axis] = value
                    if value < min_values[axis]:
                        min_values[axis] = value
                time.sleep(0.05)
            with open('joystick_calibration.pickle', 'wb') as f:
                pickle.dump([centre, values, min_values, max_values], f)
        self.values = values
        self.centre = centre
        self.min_values = min_values
        self.max_values = max_values

    def update_data_index(self):
        if not os.path.isdir(self.save_dir):
            os.mkdir(self.save_dir)
        file_list = [f for f in os.listdir(self.save_dir) if f.endswith('.mp4')]
        max_index = -1
        for f in file_list:
            ind_str = f.split(self.dataID.text())
            if len(ind_str)>1:
                ind = int(ind_str[1].split('.')[0])
                if ind > max_index:
                    max_index = ind
        self.data_file_index = max_index+1


    def get_robot_movement(self):
        pygame.event.pump()
        for i in range(self.joystick_count):
            axes = self.joystick.get_numaxes()
            for axis in range(axes):
                self.values[axis] = self.joystick.get_axis(axis)-self.centre[axis]

        vel_x = -self.values[1]/self.max_values[1]
        vel_y = -self.values[0]/self.max_values[0]
        vel_a = -self.values[4]/self.max_values[4]
        if self.env.robot.type == "diff-drive": vel_y = 0
        return [vel_x, vel_y, vel_a]


    def compute(self):
        robot_vel = self.get_robot_movement()
        obs, reward, terminated, truncated, info = self.env.step(robot_vel) 

        image = self.env.render_without_showing()
        image = image.astype(np.uint8)

        obs["action"] = np.array(robot_vel, dtype=np.float32)
        for key in obs.keys():
            obs[key] = obs[key].tolist()
        obs["timestamp"] = time.time()

        self.data.append(obs)
        done = terminated or truncated

        if not done:
            self.images_for_video.append(cv2.resize(image, (500, 500)))
        else:
            if self.start_saving_button.isChecked():
                self.save_data()
            self.regenerate()

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB )

        labelSize = ((self.label.width()//4)*4, self.label.height())
        image = cv2.resize(image, labelSize)
        self.label.setPixmap(QtGui.QPixmap(QtGui.QImage(image.data, image.shape[1], image.shape[0], QtGui.QImage.Format_RGB888)))

    # def get_observations(self):
    #     observations = {}
    #     for entity in self.env.static_humans + self.env.dynamic_humans + self.env.tables + self.env.laptops + self.env.plants + self.env.walls:
    #         coordinates, angle = [entity.x, entity.y], entity.orientation
    #         sin_theta = np.sin(angle)
    #         cos_theta = np.cos(angle)

    #         theta = np.arctan2(sin_theta, cos_theta)
    #         observations[entity.id] = EntityObs(
    #             entity.id,
    #             coordinates[0],
    #             coordinates[1],
    #             theta,
    #             sin_theta,
    #             cos_theta
    #         )

    #     # adding human-human interactions
    #     for i in self.env.moving_interactions + self.env.static_interactions:
    #         for entity in i.humans:
    #             coordinates, angle = [entity.x, entity.y], entity.orientation
    #             sin_theta = np.sin(angle)
    #             cos_theta = np.cos(angle)

    #             theta = np.arctan2(sin_theta, cos_theta)
    #             observations[entity.id] = EntityObs(
    #                 entity.id,
    #                 coordinates[0],
    #                 coordinates[1],
    #                 theta,
    #                 sin_theta,
    #                 cos_theta
    #             )
        
    #     # adding human-laptop interactions
    #     for i in self.env.h_l_interactions:
    #         entity = i.human
    #         coordinates, angle = [entity.x, entity.y], entity.orientation
    #         sin_theta = np.sin(angle)
    #         cos_theta = np.cos(angle)

    #         theta = np.arctan2(sin_theta, cos_theta)
    #         observations[entity.id] = EntityObs(
    #             entity.id,
    #             coordinates[0],
    #             coordinates[1],
    #             theta,
    #             sin_theta,
    #             cos_theta
    #         )

    #         entity = i.laptop
    #         coordinates, angle = [entity.x, entity.y], entity.orientation
    #         sin_theta = np.sin(angle)
    #         cos_theta = np.cos(angle)

    #         theta = np.arctan2(sin_theta, cos_theta)
    #         observations[entity.id] = EntityObs(
    #             entity.id,
    #             coordinates[0],
    #             coordinates[1],
    #             theta,
    #             sin_theta,
    #             cos_theta
    #         )
    #     return observations

    # def get_data(self, observations):
    #         people = []

    #         for human in self.env.static_humans + self.env.dynamic_humans:
    #             human_obs = observations[human.id]
    #             person = {}
    #             person['id'] = human.id
    #             person['x'] = human_obs.x
    #             person['y'] = human_obs.y
    #             person['angle'] = human_obs.theta
    #             people.append(person)

    #         objects = []
    #         for object in self.env.laptops + self.env.tables:
    #             obs = observations[object.id]
    #             obj = ObjectT()
    #             obj.id = obs.id
    #             obj.x = -obs.y
    #             obj.y = obs.x
    #             obj.angle = -(np.pi/2 + np.arctan2(obs.sin_theta, obs.cos_theta))
    #             obj.bbx1 = -object.length/2
    #             obj.bbx2 = object.length/2
    #             obj.bby1 = -object.width/2
    #             obj.bby2 = object.width/2
    #             objects.append(obj)
    #         for object in self.env.plants:
    #             obs = observations[object.id]
    #             obj = ObjectT()
    #             obj.id = obs.id
    #             obj.x = -obs.y
    #             obj.y = obs.x
    #             obj.angle = -(np.pi/2 + np.arctan2(obs.sin_theta, obs.cos_theta))
    #             obj.bbx1 = -object.radius
    #             obj.bbx2 = object.radius
    #             obj.bby1 = -object.radius
    #             obj.bby2 = object.radius
    #             objects.append(obj)

    #         walls = []
    #         for wall in self.env.walls:
    #             w = WallT()
    #             x1 = wall.x - np.cos(wall.orientation)*wall.length/2
    #             x2 = wall.x + np.cos(wall.orientation)*wall.length/2
    #             y1 = wall.y - np.sin(wall.orientation)*wall.length/2
    #             y2 = wall.y + np.sin(wall.orientation)*wall.length/2
    #             a1, _ = self.get_pose([x1, y1], 0)
    #             a2, _ = self.get_pose([x2, y2], 0)
    #             # a1 = self.env.get_robot_frame_coordinates(np.array([[x1, y1]])).flatten()
    #             # a2 = self.env.get_robot_frame_coordinates(np.array([[x2, y2]])).flatten()

    #             w.x1 = -a1[1]
    #             w.y1 = a1[0]
    #             w.x2 = -a2[1]
    #             w.y2 = a2[0]
    #             walls.append(w)

    #         interactions = []
    #         for interaction in self.env.moving_interactions + self.env.static_interactions + self.env.h_l_interactions:
    #             if interaction.name == "human-human-interaction":
    #                 for human in interaction.humans:
    #                     human_obs = observations[human.id]
    #                     person = Person()
    #                     person.id = human.id
    #                     person.x = -human_obs.y
    #                     person.y = human_obs.x
    #                     person.angle = -(np.pi/2 + np.arctan2(human_obs.sin_theta, human_obs.cos_theta))
    #                     people.append(person)

    #                 for i in range(len(interaction.humans)):
    #                     for j in range(i+1, len(interaction.humans)):
    #                         inter = InteractionT()
    #                         inter.idSrc = interaction.humans[i].id
    #                         inter.idDst = interaction.humans[j].id
    #                         inter.type = "human-human-interaction"
    #                         interactions.append(inter)

                
    #             if interaction.name == "human-laptop-interaction":
    #                 human = interaction.human
    #                 laptop = interaction.laptop

    #                 human_obs = observations[human.id]
    #                 person = Person()
    #                 person.id = human.id
    #                 person.x = -human_obs.y
    #                 person.y = human_obs.x
    #                 person.angle = -(np.pi/2 + np.arctan2(human_obs.sin_theta, human_obs.cos_theta))
    #                 people.append(person)


    #                 obs = observations[laptop.id]
    #                 obj = ObjectT()
    #                 obj.id = obs.id
    #                 obj.x = -obs.y
    #                 obj.y = obs.x
    #                 obj.angle = -(np.pi/2 + np.arctan2(obs.sin_theta, obs.cos_theta))
    #                 obj.bbx1 = -laptop.length/2
    #                 obj.bbx2 = laptop.length/2
    #                 obj.bby1 = -laptop.width/2
    #                 obj.bby2 = laptop.width/2
    #                 objects.append(obj)

    #                 inter = InteractionT()
    #                 inter.idSrc = human.id
    #                 inter.idDst = laptop.id
    #                 inter.type = "human-laptop-interaction"
    #                 interactions.append(inter)
            
    #         robot_goal, _ = self.get_pose([self.env.robot.goal_x, self.env.robot.goal_y], 0)
    #         # robot_goal = self.env.get_robot_frame_coordinates(np.array([[self.env.robot.goal_x, self.env.robot.goal_y]])).flatten()

    #         goal = GoalT()
    #         goal.x = -robot_goal[1]
    #         goal.y = robot_goal[0]

    #         robot = ObjectT()
    #         robot.id = -2
    #         robot.x = self.env.robot.x
    #         robot.y = self.env.robot.y
    #         robot.angle = -(-np.pi/2 + self.env.robot.orientation)
            
    #         objects.append(robot)


    #         return people, objects, walls, interactions, goal


    def regenerate(self):
        self.images_for_video.clear()
        self.data.clear()
        self.env.reset()

    def save_data(self):
        file_name = self.dataID.text() + '{0:06d}'.format(self.data_file_index)

        with open(self.save_dir+ file_name +'.json', 'w') as f:
            json.dump(self.data, f, indent=4) #, sort_keys=True)


        fourcc =  cv2.VideoWriter_fourcc(*'MP4V') # mp4
        writer = cv2.VideoWriter(self.save_dir + file_name + '.mp4', fourcc, 30, (self.images_for_video[0].shape[1], self.images_for_video[0].shape[0])) 
        for image in self.images_for_video:
            writer.write(image)
        writer.release()

        self.data_file_index += 1

    def quit_slot(self):
        self.close()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec_())
