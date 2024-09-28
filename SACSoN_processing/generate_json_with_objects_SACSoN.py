import json
import numpy as np

def compute_chair_attributes(x_pos, y_pos, id):
    points = np.array([x_pos, y_pos])
    points = np.transpose(points)
    center = np.mean(points, axis=0)
    width = np.linalg.norm(points[0]-points[1])
    height = np.linalg.norm(points[1]-points[2])
    angle = np.arctan2((points[0][0]-points[1][0]), (points[1][1]-points[0][1]))

    chair = {'id': id,
             'type': 'chair',
             'x': center[0],
             'y': center[1],
             'angle': angle,
             'shape': {'type': 'rectangle', 'width': width, 'height': height}
            }
    return chair
    
objects = []

x_chair = [-5.3, -4.5, -5.0, -5.8]
y_chair = [5.8, 12.8, 12.9, 5.9]
chair = compute_chair_attributes(x_chair, y_chair, id = 0)
objects.append(chair)

x_chair = [-0.4, 5.2, 5.3, -0.3]
y_chair = [13.0, 12.4, 12.9, 13.5]
chair = compute_chair_attributes(x_chair, y_chair, id = 1)
objects.append(chair)

x_chair = [4.35, 4.30, 11.30, 11.4]
y_chair = [11.4, 10.9, 9.9, 10.4]
chair = compute_chair_attributes(x_chair, y_chair, id = 2)
objects.append(chair)

x_chair = [1.5, 3.4, 3.3, 1.7]
y_chair = [6.1, 6.0, 6.4, 6.4]
chair = compute_chair_attributes(x_chair, y_chair, id = 3)
objects.append(chair)

x_chair = [1.7, 1.0, 0.6, 1.5]
y_chair = [6.4, 7.5, 7.5, 6.1]
chair = compute_chair_attributes(x_chair, y_chair, id = 4)
objects.append(chair)

x_chair = [0.5, 0.6, 0.3, 0.1]
y_chair = [7.6, 8.9, 9.2, 7.3]
chair = compute_chair_attributes(x_chair, y_chair, id = 5)
objects.append(chair)

x_chair = [1.2, 2.8, 2.7, 1.4]
y_chair = [9.3, 9.6, 9.2, 9.0]
chair = compute_chair_attributes(x_chair, y_chair, id = 6)
objects.append(chair)

x_chair = [4.2, 5.8, 5.7, 4.5]
y_chair = [8.2, 8.7, 8.3, 7.9]
chair = compute_chair_attributes(x_chair, y_chair, id = 7)
objects.append(chair)

x_chair = [6.65, 6.75, 6.0, 5.6]
y_chair = [8.65, 8.2, 7.5, 7.7]
chair = compute_chair_attributes(x_chair, y_chair, id = 8)
objects.append(chair)

x_chair = [4.2, 4.2, 4.5, 4.5]
y_chair = [7.5, 6.4, 6.3, 7.6]
chair = compute_chair_attributes(x_chair, y_chair, id = 9)
objects.append(chair)

x_chair = [5.5, 5.9, 6.4, 6.2] 
y_chair = [6.5, 6.5, 5.5, 5.2] 
chair = compute_chair_attributes(x_chair, y_chair, id = 10)
objects.append(chair)

x_chair = [6.4, 7.5, 7.7, 6.2] 
y_chair = [5.5, 5.5, 5.2, 5.2] 
chair = compute_chair_attributes(x_chair, y_chair, id = 11)
objects.append(chair)

objects_and_walls = {}
objects_and_walls['objects'] = objects

with open('SACSoN_objects_walls.json', 'w') as f:
    f.write(json.dumps(objects_and_walls))
    f.close()


