import json
import numpy as np

def compute_chair_attributes(x_pos, y_pos, id):
    points = np.array([x_pos, y_pos])
    points = np.transpose(points)
    center = np.mean(points, axis=0)
    width = np.linalg.norm(points[0]-points[1])
    height = np.linalg.norm(points[1]-points[2])
    angle1 = np.arctan2((points[0][0]-points[1][0]), (points[1][1]-points[0][1]))
    angle2 = np.arctan2((points[3][0]-points[2][0]), (points[2][1]-points[3][1]))
    angle = (angle1+angle2)/2

    chair = {'id': id,
             'type': 'chair',
             'x': center[0],
             'y': center[1],
             'angle': angle,
             'shape': {'type': 'rectangle', 'width': width, 'length': height}
            }
    return chair

def generate_wall_segments(x_pos, y_pos):
    w_segments = []

    x_i = x_pos[0]
    y_i = y_pos[0]

    for x_e, y_e in zip(x_pos[1:], y_pos[1:]):
        w_segments.append([x_i, y_i, x_e, y_e])
        x_i, y_i = x_e, y_e

    return w_segments

############################
    
objects = []

x_chair = [-5.3, -4.5, -5.0, -5.8]
y_chair = [5.8, 12.8, 12.9, 5.9]
chair = compute_chair_attributes(x_chair, y_chair, id = 0)
objects.append(chair)

x_chair = [-0.4, 5.2, 5.3, -0.3]
y_chair = [13.0, 12.4, 12.9, 13.5]
chair = compute_chair_attributes(x_chair, y_chair, id = 1)
objects.append(chair)

x_chair = [4.30, 11.30, 11.4, 4.35]
y_chair = [10.9, 9.9, 10.4, 11.4]

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

x_chair = [6.75, 6.0, 5.6, 6.65]
y_chair = [8.2, 7.5, 7.7, 8.65]
chair = compute_chair_attributes(x_chair, y_chair, id = 8)
objects.append(chair)

x_chair = [4.2, 4.2, 4.5, 4.5]
y_chair = [7.5, 6.4, 6.3, 7.6]
chair = compute_chair_attributes(x_chair, y_chair, id = 9)
objects.append(chair)

x_chair = [5.9, 6.4, 6.2, 5.5] 
y_chair = [6.5, 5.5, 5.2, 6.5] 
chair = compute_chair_attributes(x_chair, y_chair, id = 10)
objects.append(chair)

x_chair = [6.4, 7.5, 7.7, 6.2] 
y_chair = [5.5, 5.5, 5.2, 5.2] 
chair = compute_chair_attributes(x_chair, y_chair, id = 11)
objects.append(chair)

walls = []
x_wall = [-6.8, -5.8, -5.8, -3.8, -3.7, -2.7, -2.7, -1.65, -1.7, -0.9, -0.95, 2.4, 2.5, 3.8, 3.7, 3.2, 3.0]
y_wall = [0.3, 0.2, -0.4, -0.55, 0.0, -0.1, 0.55, 0.4, -0.35, -0.45, -0.75, -1.1, -0.6, -0.8, -1.5, -1.5, -3.5]
w_segments = generate_wall_segments(x_wall, y_wall)
walls += w_segments

x_wall = [-4.5, -4.2, -0.2, -0.3]
y_wall = [12.8, 15.2, 14.7, 13.5]
w_segments = generate_wall_segments(x_wall, y_wall)
walls += w_segments

x_wall = [5.2, 5.1]
y_wall = [12.6, 11.5]
w_segments = generate_wall_segments(x_wall, y_wall)
walls += w_segments

x_wall = [12.4, 13.6]
y_wall = [13.0, 12.8]
w_segments = generate_wall_segments(x_wall, y_wall)
walls += w_segments


objects_and_walls = {}
objects_and_walls['objects'] = objects
objects_and_walls['walls'] = walls

with open('SACSoN_objects_walls.json', 'w') as f:
    f.write(json.dumps(objects_and_walls))
    f.close()


