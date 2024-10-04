import json
import jsbeautifier
import sys
import numpy as np

if len(sys.argv)<2:
    print('Please, specify the name of the JSon file')
    exit()

file_name = sys.argv[1]
data = json.load(open(file_name, 'r'))

goal = data['sequence'][0]["goal"]
robot_trajectories = []
trajectory = []
for d in data['sequence']:
    if d["goal"]["x"] == goal["x"] and d["goal"]["y"] == goal["y"]:
        trajectory.append([d["robot"]["x"], d["robot"]["y"]])
    else:
        robot_trajectories.append(trajectory)
        goal = d["goal"]
        trajectory = []

print("number of trajectories", len(robot_trajectories))
for i in range(len(robot_trajectories)):
    print("Length trajectory", i, ":", len(robot_trajectories[i]))
    minx = np.min(np.array(robot_trajectories[i])[:,0])
    maxx = np.max(np.array(robot_trajectories[i])[:,0])
    miny = np.min(np.array(robot_trajectories[i])[:,1])
    maxy = np.max(np.array(robot_trajectories[i])[:,1])
    print(minx, maxx, miny, maxy)

