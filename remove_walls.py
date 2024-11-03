import sys
import copy
import json
import random
import argparse
import multiprocessing

import jsbeautifier
import fastjsonschema


if __name__ == "__main__":
    schema = json.load(open("schema.json", "r"))
    validator = fastjsonschema.compile(schema)

    parser = argparse.ArgumentParser(
                        prog='checkjson',
                        description='Check json files for SocNav3')
    parser.add_argument('files', metavar='file', type=str, nargs="+")

    args = parser.parse_args()

    def do_it(input_file):
        # Open JSON file
        with open(input_file, "r") as f:
            instance = json.load(f)
        # Validate it
        try:
            validator(instance)
        except fastjsonschema.JsonSchemaException as e:
            print(f"Data failed validation: {e}")
            print("Exiting.")
            sys.exit(-10)


        print(f"Input file {input_file} [{len(instance['walls'])} walls]")

        # Remove ALL walls
        nowalls_instance = copy.deepcopy(instance)
        nowalls_instance["walls"].clear()
        output_path = '.'.join(input_file.split('.')[:-1])+"_no_walls.json"
        print(f"  none [{len(nowalls_instance['walls'])} walls]")
        with open(output_path, 'w') as f:
            options = jsbeautifier.default_options()
            options.indent_size = 2
            f.write(jsbeautifier.beautify(json.dumps(nowalls_instance), options))

        # Remove walls randomly P=0.5
        somewalls_instance = copy.deepcopy(instance)
        all_walls = somewalls_instance["walls"]
        somewalls_instance["walls"] = [x for x in all_walls if random.random() >= 0.5]
        output_path = '.'.join(input_file.split('.')[:-1])+"_some_walls.json"
        print(f"  some [{len(somewalls_instance['walls'])} walls]")
        with open(output_path, 'w') as f:
            options = jsbeautifier.default_options()
            options.indent_size = 2
            f.write(jsbeautifier.beautify(json.dumps(somewalls_instance), options))

    with multiprocessing.Pool(10) as pool:
        results = pool.map(do_it, args.files)
