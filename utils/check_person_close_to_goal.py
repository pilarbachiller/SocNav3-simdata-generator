import sys
import numpy as np
import json

for file_name in sys.argv[1:]:
    f = open(file_name, 'r')
    data = json.load(f)
    f.close()
    last_frame = data["sequence"][-1]
    robot_x = last_frame["robot"]['x']
    robot_y = last_frame["robot"]['y']
    r_p = np.array([robot_x, robot_y])
    for p in last_frame["people"]:
        p_p = np.array([p['x'], p['y']])
        if np.linalg.norm(p_p-r_p)<1.:
            print("Potential candidate", file_name)
    

