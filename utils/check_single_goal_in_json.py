import sys
import numpy as np
import json
import jsbeautifier

for file_name in sys.argv[1:]:
    f = open(file_name, 'r')
    data = json.load(f)
    f.close()
    last_frame = data["sequence"][-1]
    goal = last_frame["goal"]
    modified = False
    for s in data["sequence"]:
        if s["goal"]["x"]!=goal["x"] or s["goal"]["x"]!=goal["x"]:
            s["goal"] = goal
            modified = True

    if modified:
        output_path = '.'.join(file_name.split('.')[:-1])+"_single_goal.json"
        print("Saving output to:", output_path)
        with open(output_path, 'w') as f:
            options = jsbeautifier.default_options()
            options.indent_size = 2
            f.write(jsbeautifier.beautify(json.dumps(data), options))

    

