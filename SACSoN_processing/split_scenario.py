import json
import jsbeautifier
import sys
import numpy as np
import shapely
from shapely.geometry import Polygon, LineString, Point

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
    pc = np.array(c)
    p1 = mp1 + mp2 + pc
    p2 = -mp1 + mp2 + pc
    p3 = -mp1 - mp2 + pc
    p4 = mp1 - mp2 + pc
    poly = Polygon((tuple(p1), tuple(p2), tuple(p3), tuple(p4)))
    return poly

def polygon_to_object(poly):
    points = np.array(poly.exterior.coords[:-1])
    center = np.mean(points, axis=0)
    width = np.linalg.norm(points[0]-points[1])
    height = np.linalg.norm(points[1]-points[2])
    angle1 = np.arctan2((points[0][0]-points[1][0]), (points[1][1]-points[0][1]))
    angle2 = np.arctan2((points[3][0]-points[2][0]), (points[2][1]-points[3][1]))
    angle = (angle1+angle2)/2

    return center, width, height, angle


if len(sys.argv)<2:
    print('Please, specify the name of the JSon files')
    exit()

files = sys.argv[1:]

for file_name in files:
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
    margin = 1.
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

        # minPWorld = minPWorld + 0.5
        # maxPWorld = maxPWorld - 0.5
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
        first_row = max(0, ULPGrid[1])
        last_row = min(data["grid"]["height"]-1, BRPGrid[1])
        first_col = max(0, ULPGrid[0])
        last_col = min(data["grid"]["width"]-1, BRPGrid[0])

        grid_data["data"] = np_grid[first_row:last_row+1, first_col:last_col+1]

        if first_row>ULPGrid[1]:
            new_rows = np.full(((first_row-ULPGrid[1]), grid_data["data"].shape[1]), -1)
            grid_data["data"] = np.append(new_rows, grid_data["data"], axis=0)

        if last_row<BRPGrid[1]:
            new_rows = np.full(((BRPGrid[1]-last_row), grid_data["data"].shape[1]), -1)
            grid_data["data"] = np.append(grid_data["data"], new_rows, axis=0)

        if first_col>ULPGrid[0]:
            new_cols = np.full((grid_data["data"].shape[0], (first_col-ULPGrid[0])), -1)
            grid_data["data"] = np.append(new_cols, grid_data["data"], axis=1)

        if last_col<BRPGrid[0]:
            new_cols = np.full((grid_data["data"].shape[0], (BRPGrid[0]-last_col)), -1)
            grid_data["data"] = np.append(grid_data["data"], new_cols, axis=1)

        grid_data["data"] = grid_data["data"].tolist()

        new_data["grid"] = grid_data

        # Add walls
        walls = []
        for w in data["walls"]:
            if room.contains(Point(w[0], w[1])) or room.contains(Point(w[2], w[3])):
                new_wall = shapely.intersection(room, LineString([(w[0], w[1]), (w[2], w[3])]))
                new_wall_p1 = convert_coordinates_to_new_origin([new_wall.coords[0][0], new_wall.coords[0][1]], world_origin)
                new_wall_p2 = convert_coordinates_to_new_origin([new_wall.coords[1][0], new_wall.coords[1][1]], world_origin)
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


            objects = []
            for o in d["objects"]:
                object_poly = object_to_polygon([o["x"], o["y"]], o["shape"]["width"], o["shape"]["length"], o["angle"]) 
                inter_poly = shapely.intersection(room, object_poly)
                if not inter_poly.is_empty: #room.contains(object_poly): 
                    # print("orig", object_poly)
                    # print("inter", inter_poly)
                    new_object = o
                    # c, w, h, angle = polygon_to_object(inter_poly)
                    # new_p = convert_coordinates_to_new_origin([c[0], c[1]], world_origin)
                    # new_object["x"] = new_p[0]
                    # new_object["y"] = new_p[1]
                    # new_object["width"] = w
                    # new_object["height"] = h
                    # new_object["angle"] = angle
                    new_p = convert_coordinates_to_new_origin([o["x"], o["y"]], world_origin)
                    new_object["x"] = new_p[0]
                    new_object["y"] = new_p[1]
                    objects.append(new_object)

            people = []
            for p in d["people"]:
                if room.contains(Point(p["x"], p["y"])):
                    new_person = p
                    new_p = convert_coordinates_to_new_origin([p["x"], p["y"]], world_origin)
                    new_person["x"] = new_p[0]
                    new_person["y"] = new_p[1]
                    people.append(new_person)


            frame["objects"] = objects
            frame["people"] = people

            sequence.append(frame)
        
        new_data["sequence"] = sequence

        output_path = '.'.join(file_name.split('.')[:-1]) + "_split_" + str(i+1) + ".json"

        with open(output_path, 'w') as f:
            options = jsbeautifier.default_options()
            options.indent_size = 2
            f.write(jsbeautifier.beautify(json.dumps(new_data), options))
            # f.write(json.dumps(new_data))
            f.close()

