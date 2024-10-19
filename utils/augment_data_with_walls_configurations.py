import sys
import numpy as np
import json
import jsbeautifier
import copy
import random

for file_name in sys.argv[1:]:
    f = open(file_name, 'r')
    data = json.load(f)
    f.close()
    walls = copy.deepcopy(data["walls"])
    if len(walls)>0:
        # Remove all the walls
        data["walls"] = []
        output_path = '.'.join(file_name.split('.')[:-1])+"_no_walls.json"
        print("Saving a version with no walls to:", output_path)
        with open(output_path, 'w') as f:
            options = jsbeautifier.default_options()
            options.indent_size = 2
            f.write(jsbeautifier.beautify(json.dumps(data), options))

        if len(walls)>1:
            # Remove some walls
            wall_indices = list(range(len(walls)))
            n_walls_to_remove = random.randint(1, len(wall_indices)-1)
            for _ in range(n_walls_to_remove):
                wall_to_remove = random.choice(wall_indices)
                wall_indices.remove(wall_to_remove)

            data["walls"] = [walls[i] for i in wall_indices]
            output_path = '.'.join(file_name.split('.')[:-1])+"_some_walls.json"
            print("Saving a version with some walls to:", output_path)
            with open(output_path, 'w') as f:
                options = jsbeautifier.default_options()
                options.indent_size = 2
                f.write(jsbeautifier.beautify(json.dumps(data), options))

