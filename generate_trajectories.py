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

UPDATE_PERIOD = 0.1

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
        self.regenerate()
        self.last_data_update = time.time()

        # self.start_saving_button.toggled.connect(self.start_saving)
        self.regenerate_button.clicked.connect(self.regenerate)
        self.quit_button.clicked.connect(self.quit_slot)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.compute)
        self.timer.start(self.env.TIMESTEP*1000)

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


        people, objects, walls, interactions, goal = self.get_data()

        observation = {}
        observation["timestamp"] = time.time()
        observation["action"] = robot_vel
        observation["people"] = people
        observation["objects"] = objects
        observation["walls"] = walls
        observation["interactions"] = interactions
        observation["goal"] = goal
        
        done = terminated or truncated

        if not done:
            if time.time()-self.last_data_update > UPDATE_PERIOD:
                self.images_for_video.append(cv2.resize(image, (500, 500)))
                self.data.append(observation)            
                self.last_data_update = time.time()
        else:
            self.end_episode = time.time()
            if self.start_saving_button.isChecked():
                self.save_data()
            self.regenerate()
            self.last_data_update = time.time()

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB )

        labelSize = ((self.label.width()//4)*4, self.label.height())
        image = cv2.resize(image, labelSize)
        self.label.setPixmap(QtGui.QPixmap(QtGui.QImage(image.data, image.shape[1], image.shape[0], QtGui.QImage.Format_RGB888)))

    def get_data(self):
        people = []
        for human in self.env.static_humans + self.env.dynamic_humans:
            person = {}
            person['id'] = human.id
            person['x'] = human.x
            person['y'] = human.y
            person['angle'] = human.orientation
            person['speed'] = human.speed
            people.append(person)

        objects = []
        for o in self.env.laptops + self.env.tables:
            obj = {}
            obj['id'] = o.id
            obj['x'] = o.x
            obj['y'] = o.y
            obj['angle'] = o.orientation
            obj['size'] = [o.width, o.length]
            objects.append(obj)
        for o in self.env.plants:
            obj = {}
            obj['id'] = o.id
            obj['x'] = o.x
            obj['y'] = o.y
            obj['angle'] = o.orientation
            obj['size'] = [o.radius, o.radius]
            objects.append(obj)

        walls = []
        for wall in self.env.walls:
            x1 = wall.x - np.cos(wall.orientation)*wall.length/2
            x2 = wall.x + np.cos(wall.orientation)*wall.length/2
            y1 = wall.y - np.sin(wall.orientation)*wall.length/2
            y2 = wall.y + np.sin(wall.orientation)*wall.length/2
            walls.append([x1, y1, x2, y2])

        interactions = []
        for interaction in self.env.moving_interactions + self.env.static_interactions + self.env.h_l_interactions:
            if interaction.name == "human-human-interaction":
                for human in interaction.humans:
                    person = {}
                    person['id'] = human.id
                    person['x'] = human.x
                    person['y'] = human.y
                    person['angle'] = human.orientation
                    person['speed'] = human.speed
                    people.append(person)

                for i in range(len(interaction.humans)):
                    for j in range(i+1, len(interaction.humans)):
                        inter = {}
                        inter['idSrc'] = interaction.humans[i].id
                        inter['idDst'] = interaction.humans[j].id
                        inter['type'] = "human-human-interaction"
                        interactions.append(inter)

            
            if interaction.name == "human-laptop-interaction":
                human = interaction.human
                laptop = interaction.laptop

                person = {}
                person['id'] = human.id
                person['x'] = human.x
                person['y'] = human.y
                person['angle'] = human.orientation
                person['speed'] = human.speed

                obj = {}
                obj['id'] = laptop.id
                obj['x'] = laptop.x
                obj['y'] = laptop.y
                obj['angle'] = laptop.orientation
                obj['size'] = [laptop.width, laptop.length]
                objects.append(obj)

                inter = {}
                inter['idSrc'] = human.id
                inter['idDst'] = laptop.id
                inter['type'] = "human-laptop-interaction"
                interactions.append(inter)
        
        goal = {}
        goal['x'] = self.env.robot.goal_x
        goal['y'] = self.env.robot.goal_y

        return people, objects, walls, interactions, goal


    def regenerate(self):
        self.images_for_video.clear()
        self.data.clear()
        self.env.reset()
        self.ini_episode = time.time()

    def save_data(self):
        file_name = self.dataID.text() + '{0:06d}'.format(self.data_file_index)

        with open(self.save_dir+ file_name +'.json', 'w') as f:
            json.dump(self.data, f, indent=4)

        fps = len(self.images_for_video)/(self.end_episode-self.ini_episode)
        fourcc =  cv2.VideoWriter_fourcc(*'MP4V') # mp4
        writer = cv2.VideoWriter(self.save_dir + file_name + '.mp4', fourcc, fps, (self.images_for_video[0].shape[1], self.images_for_video[0].shape[0])) 
        for image in self.images_for_video:
            writer.write(image)
        writer.release()

        self.data_file_index += 1

    def start_saving(self, save):
        if save:
            self.regenerate()

    def quit_slot(self):
        self.close()

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MainWindow()
    mainWin.show()
    sys.exit(app.exec_())
