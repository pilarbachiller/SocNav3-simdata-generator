import json
import jsbeautifier
import sys
import numpy as np
from shapely.geometry import Polygon, Point

def convert_coordinates_to_new_origin(pOld, newOrigin):
    newX = pOld[0] - newOrigin[0]
    newY = pOld[1] - newOrigin[1]
    return [newX, newY]
    
def world_to_grid(pWorld, gridOrigin, cellSize):
    gridX = (pWorld[0]-gridOrigin[0])/cellSize
    gridY = (-pWorld[1]-gridOrigin[1])/cellSize
    return [gridX, gridY]

def grid_to_world(pGrid, gridOrigin, cellSize):
    worldX = pGrid[0]*cellSize + gridOrigin[0]
    worldY = -pGrid[1]*cellSize - gridOrigin[1]
    return [worldX, worldY]

def object_to_polygon(c, w, h, a):
    mp1 = np.array([w/2*np.sin(a), -w/2*np.cos(a)])
    mp2 = np.array([h/2*np.cos(a), h/2*np.sin(a)])
    print("MP", mp1, mp2)
    pc = np.array(c)
    p1 = mp1 + mp2 + pc
    print("p1",p1)
    p2 = -mp1 + mp2 + pc
    print("p2",p2)
    p3 = -mp1 - mp2 + pc
    print("p3",p3)
    p4 = mp1 - mp2 + pc
    print("p4",p4)
    poly = Polygon((tuple(p1), tuple(p2), tuple(p3), tuple(p4)))
    return poly

if len(sys.argv)<2:
    print('Please, specify the name of the JSon file')
    exit()

file_name = sys.argv[1]
data = json.load(open(file_name, 'r'))

goal = data['sequence'][0]["goal"]
robot_trajectories = []
trajectory = []
split_slices = []
first_index = 0
for index, d in enumerate(data['sequence']):
    if d["goal"]["x"] == goal["x"] and d["goal"]["y"] == goal["y"]:
        trajectory.append([d["robot"]["x"], d["robot"]["y"]])
    else:
        split_slices.append([first_index, index])
        robot_trajectories.append(trajectory)
        goal = d["goal"]
        first_index = index
        trajectory = []

print("number of trajectories", len(robot_trajectories))
print("split slices", split_slices)
grid_origin = [data["grid"]["x_orig"], data["grid"]["y_orig"]]
cell_size = data["grid"]["cell_size"]
margin = 0.
new_data = {}
for i in range(len(robot_trajectories)):
    print("Length trajectory", i, ":", len(robot_trajectories[i]))
    minx = np.min(np.array(robot_trajectories[i])[:,0]) - margin
    maxx = np.max(np.array(robot_trajectories[i])[:,0]) + margin
    miny = np.min(np.array(robot_trajectories[i])[:,1]) - margin
    maxy = np.max(np.array(robot_trajectories[i])[:,1]) + margin

    world_origin = [(minx+maxx)/2, (miny+maxy)/2]
    side = max(abs(minx-maxx), abs(miny-maxy))
    minPWorld = world_origin - side/2
    maxPWorld = world_origin + side/2
    minPGrid = world_to_grid(minPWorld, grid_origin, cell_size)
    maxPGrid = world_to_grid(maxPWorld, grid_origin, cell_size)
    print("Min. point in world and grid", minPWorld, minPGrid)
    print("Max. point in world and grid", maxPWorld, maxPGrid)

    ULPGrid = [int(min(minPGrid[0], maxPGrid[0])), int(min(minPGrid[1], maxPGrid[1]))]
    BRPGrid = [int(max(minPGrid[0], maxPGrid[0])), int(max(minPGrid[1], maxPGrid[1]))]

    room = Polygon(((minPWorld[0], minPWorld[1]), (maxPWorld[0], minPWorld[1]), (maxPWorld[0], maxPWorld[1]), (minPWorld[0], maxPWorld[1])))

    # Create new grid
    grid_data = {}
    grid_data["cell_size"] = cell_size
    new_grid_origin = grid_to_world([ULPGrid[0], BRPGrid[1]], grid_origin, cell_size)
    new_grid_origin = convert_coordinates_to_new_origin(new_grid_origin, world_origin)
    grid_data["x_orig"] = new_grid_origin[0]
    grid_data["y_orig"] = new_grid_origin[1]
    grid_data["angle_orig"] = data["grid"]["angle_orig"]
    grid_data["width"] = BRPGrid[0]-ULPGrid[0]+1
    grid_data["height"] = BRPGrid[1]-ULPGrid[1]+1
    np_grid = np.array(data["grid"]["data"])
    grid_data["data"] = np_grid[ULPGrid[1]:BRPGrid[1]+1, ULPGrid[0]:BRPGrid[0]+1].tolist()
    

    # print("UL", ULPGrid, "BR", BRPGrid, "rows", len(grid_data["data"]), "cols",  len(grid_data["data"][0]))
    new_data["grid"] = grid_data
    # print("grid data", new_data["grid"]["data"])    
    # Add walls
    walls = []
    for w in data["walls"]:
        if room.contains(Point(w[0], w[1])) and room.contains(Point(w[2], w[3])):
            new_wall_p1 = convert_coordinates_to_new_origin([w[0], w[1]], world_origin)
            new_wall_p2 = convert_coordinates_to_new_origin([w[2], w[3]], world_origin)
            walls.append([new_wall_p1[0],new_wall_p1[1], new_wall_p2[0], new_wall_p2[1]])

    new_data["walls"] = walls

    s = split_slices[i]
    sequence = []
    for d in data["sequence"][s[0]:s[1]]:
        frame = {}
        frame["timestamp"] = d["timestamp"]

        frame["robot"] = d["robot"]
        new_p = convert_coordinates_to_new_origin([d["robot"]["x"], d["robot"]["y"]], world_origin)
        frame["robot"]["x"] = new_p[0]
        frame["robot"]["y"] = new_p[1]

        frame["goal"] = d["goal"]
        new_p = convert_coordinates_to_new_origin([d["goal"]["x"], d["goal"]["y"]], world_origin)
        frame["goal"]["x"] = new_p[0]
        frame["goal"]["y"] = new_p[1]

        frame["objects"] = []
        frame["people"] = []

        sequence.append(frame)
    
    new_data["sequence"] = sequence

    with open(file_name+"_split_"+str(i), 'w') as f:
        f.write(json.dumps(new_data))
        f.close()


    # o = data["sequence"][0]["objects"][4]
    # object_poly = object_to_polygon([o["x"], o["y"]], o["shape"]["width"], o["shape"]["height"], o["angle"])
    # print(o)
    # print("object polygon", object_poly)

    
