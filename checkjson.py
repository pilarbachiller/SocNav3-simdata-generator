import sys
import json
import fastjsonschema
import jsbeautifier

from copy import deepcopy

modification_made = False

def is_it_yes(s):
    if "-y" in sys.argv:
        print("👍 (fixing it)")
        return True
    print("")
    yn = input("😱 " + s)
    return yn == '' or yn[0].lower == 'y'

class DictToObject:
    '''
    Converts dictionaries to named structures.
    '''
    def __init__(self, dict_obj):
        '''
        Constructor.
        :param arg: The dictionary to convert to a named structure.
        :type arg: dict
        '''
        def process_list(l):
            ret = []
            for item in l:
                if isinstance(item, dict):
                    item = DictToObject(item)
                elif isinstance(item, list):
                    item = process_list(item)
                ret.append(item)
            return ret

        for key, value in dict_obj.items():
            if isinstance(value, dict):
                value = DictToObject(value)
            elif isinstance(value, list):
                value = process_list(value)
            self.__dict__[key] = value

    def __getitem__(self, item):
        return self.__dict__.get(item)



def check_grid(grid):
    '''
    Check grid's consistence between the data array and the width and height attributes.
    :param arg: The grid structure to check.
    :type arg: grid
    '''
    assert grid.height == len(grid.data), \
        f"💀 The number of lines in the grid ({len(grid.data)}) doesn't match the height attribute ({grid.height})."
    for idx, line in enumerate(grid.data):
        assert grid.width == len(line), \
            f"💀 The number of columns in the {idx}-th line of the grid ({len(line)}) doesn't match the width attribute ({grid.width})."



def check_timestamps(sequence):
    '''
    Check that timestamps are in order (if they aren't an error is raised) and that there is not too much space in between the smplaes (in that case we show a warning).
    :param arg: The sequence of snapshots to check.
    :type arg: list
    '''
    low_frequency = 0
    for idx, ss in enumerate(sequence):
        if idx == 0:
            last_timestamp = ss.timestamp
        else:
            assert ss.timestamp > last_timestamp, \
                f"💀 {idx}-th timestamp out of order. Timestamp {last_timestamp}, then {ss.timestamp}."
            if ss.timestamp >= 0.5 + last_timestamp:
                low_frequency += 1
            last_timestamp = ss.timestamp

    if low_frequency > 0:
        print(f"🤨 Warning: Found samples where the timestep was too long, {low_frequency} out of {len(sequence)}.")


def manage_fixes(d, e):
    '''
    Fix errors that we can fix programatically.
    '''
    errors_fixed = -1

    def fix__move_walls_into_root(d):
        '''1. Have walls been included in the sequence rather than a static property?'''
        global modification_made
        modification_made = True
        walls = deepcopy(d["sequence"][0]["walls"])
        d["walls"] = walls
        for i in range(len(d["sequence"])):
            del d["sequence"][i]["walls"]
        return d

    def fix__origin_is_centre(d):
        '''2. No origin was specified, but it's zero?'''
        global modification_made
        modification_made = True
        d["grid"]["angle_orig"] = 0.0
        d["grid"]["x_orig"] = -d["grid"]["cell_size"]*d["grid"]["width"]/2
        d["grid"]["y_orig"] = -d["grid"]["cell_size"]*d["grid"]["height"]/2
        return d

    def fix__move_goal_into_goal(d):
        '''3. Has the goal been included as a property of the robot?'''
        global modification_made
        modification_made = True
        for i in range(len(d["sequence"])):
            robot = d["sequence"][i]["robot"]
            goal = {}
            # assuming type 'go-to', and no human-focused target
            goal["type"] = "go-to"
            goal["human"] = None
            # x, y, angle
            goal["x"] = robot["goal_x"]
            del d["sequence"][i]["robot"]["goal_x"]
            goal["y"] = robot["goal_y"]
            del d["sequence"][i]["robot"]["goal_y"]
            goal["angle"] = robot["goal_angle"]
            del d["sequence"][i]["robot"]["goal_angle"]
            # thresholds
            goal["pos_threshold"] = robot["goal_pos_th"]
            del d["sequence"][i]["robot"]["goal_pos_th"]
            goal["angle_threshold"] = robot["goal_angle_th"]
            del d["sequence"][i]["robot"]["goal_angle_th"]
            # finally, add the goal to the item
            d["sequence"][i]["goal"] = goal
        return d

    def fix__make_radius_shape(d):
        '''4. The shape of the robot was assumed to be defined as a radius?'''
        global modification_made
        modification_made = True
        for i in range(len(d["sequence"])):
            robot = d["sequence"][i]["robot"]
            shape = {
                    "type": "circle",
                    "width": robot["radius"]*2,
                    "height": robot["radius"]*2
                }
            del d["sequence"][i]["robot"]["radius"]
            d["sequence"][i]["robot"]["shape"] = shape
        return d

    def fix__take_robot_pose_as_first_pose(d):
        '''5. The robot's pose is not initially defined.'''
        global modification_made
        modification_made = True
        for i in range(len(d["sequence"])):
            robot = d["sequence"][i]["robot"]
            if type(robot["x"]) is float:
                x = robot["x"]
                y = robot["y"]
                a = robot["angle"]
                for i_fix in range(i):
                    d["sequence"][i_fix]["robot"]["x"] = x
                    d["sequence"][i_fix]["robot"]["y"] = y
                    d["sequence"][i_fix]["robot"]["angle"] = a
                break
        return d

    def fix__objects_shape_not_size(d):
        '''6. Object's shape is defined using 'size' instead of 'shape'.'''
        global modification_made
        modification_made = True
        for idx_s in range(len(d["sequence"])):
            objects_i = d["sequence"][idx_s]["objects"]
            objects_o = []
            for an_object in objects_i:
                if an_object["size"][0] == an_object["size"][1]:
                    shape_type = 'circle'
                else:
                    shape_type = 'rectangle'
                shape = {
                    "type": shape_type,
                    "width": an_object["size"][0],
                    "height": an_object["size"][1]
                }
                an_object["shape"] = shape
                del an_object["size"]
                objects_o.append(an_object)
            d["sequence"][idx_s]["objects"] = objects_o
        return d

    def fix__objects_type(d):
        '''7. Object's type should be lower case.'''
        global modification_made
        modification_made = True
        for idx_s in range(len(d["sequence"])):
            objects_i = d["sequence"][idx_s]["objects"]
            objects_o = []
            for an_object in objects_i:
                an_object["type"] = an_object["type"].lower()
                objects_o.append(an_object)
            d["sequence"][idx_s]["objects"] = objects_o
        return d


    match str(e):
        # Known error: Walls are in the sequence rather than as a global property.
        case "data must contain ['walls'] properties":
            if "walls" in d["sequence"][0]:
                if is_it_yes("Do you want me to move the walls from the sequence into the root object? [Y/n]: "):
                    d = fix__move_walls_into_root(d)
                    errors_fixed = 1
                else:
                    errors_fixed = 0
        # Known error: The grid has no origin.
        case "data.grid must contain ['angle_orig', 'x_orig', 'y_orig'] properties":
            if is_it_yes("Do you want me to assume that the origin of the grid is the centre with, no angular offset? [Y/n]: "):
                d = fix__origin_is_centre(d)
                errors_fixed = 1
            else:
                errors_fixed = 0
        # Known error: Goal properties are in "robot".
        case str(x) if (x.startswith("data.sequence[") and x.endswith("] must contain ['goal'] properties")):
            for ss in reversed(d["sequence"]):
                if "goal_x" in ss["robot"]:
                    if is_it_yes("Do you want me to move the goal's properties from the 'robot' into the 'goal'? [Y/n]: "):
                        d = fix__move_goal_into_goal(d)
                        errors_fixed = 1
                    else:
                        errors_fixed = 0
        # Known error: Robot shape defined as a "radius" only.
        case str(x) if (x.startswith("data.sequence[") and x.endswith("].robot must contain ['shape'] properties")):
            for ss in reversed(d["sequence"]):
                if "radius" in ss["robot"]:
                    if is_it_yes("Do you want me to set the robot's shape from the 'radius' attribute? [Y/n]: "):
                        d = fix__make_radius_shape(d)
                        errors_fixed = 1
                    else:
                        errors_fixed = 0
        # Known error: Robot's pose is undefined initially
        case str(x) if (x.startswith("data.sequence[") and x.endswith("].robot.x must be number")): 
            if is_it_yes("Do you want me to set the robot's pose as the first valid one in the future? [Y/n]: "):
                d = fix__take_robot_pose_as_first_pose(d)
                errors_fixed = 1
            else:
                errors_fixed = 0
        # Known error: Object's shape is defined by size only
        case str(x) if (x.startswith("data.sequence[") and x.endswith("].objects[0] must contain ['shape'] properties")): 
            if is_it_yes("Do you want me to try to set the objects' shapes from a 'size' attribute? [Y/n]: "):
                d = fix__objects_shape_not_size(d)
                errors_fixed = 1
            else:
                errors_fixed = 0
        # Known error: Object's shape is defined by size only
        case str(x) if (x.startswith("data.sequence[") and "].type must be one of ['chair'" in x): 
            if is_it_yes("Do you want me to try to fix objects' types? [Y/n]: "):
                d = fix__objects_type(d)
                errors_fixed = 1
            else:
                errors_fixed = 0

    return d, errors_fixed


if __name__ == "__main__":
    schema = json.load(open("schema.json", "r"))
    validator = fastjsonschema.compile(schema)
    dict_instance = json.load(open(sys.argv[1], "r"))

    do_it = True
    while do_it:
        instance = DictToObject(dict_instance)

        try:
            validator(dict_instance)
            do_it = False
            print("Correct.")
        except fastjsonschema.JsonSchemaException as e:
            print(f"Data failed validation: {e}")

            dict_instance, errors_fixed = manage_fixes(dict_instance, e)
            if errors_fixed == 0:
                do_it = False

        # These errors cannot be fixed automatically
        check_grid(instance.grid)
        check_timestamps(instance.sequence)

    if modification_made:
        output_path = sys.argv[1]+".out"
        print("Saving output to:", output_path)
        with open(output_path, 'w') as f:
            options = jsbeautifier.default_options()
            options.indent_size = 2
            f.write(jsbeautifier.beautify(json.dumps(dict_instance), options))

        # json.dump(dict_instance, open(output_path, "w"))

