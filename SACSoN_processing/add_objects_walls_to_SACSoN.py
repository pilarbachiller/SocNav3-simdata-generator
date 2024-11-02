import json
import jsbeautifier
import sys

if len(sys.argv)<2:
    print('Please, specify the name of the JSon files')
    exit()

files = sys.argv[1:]

for file_name in files:
    data = json.load(open(file_name, 'r'))
    objects_walls = json.load(open('SACSoN_objects_walls.json','r'))

    for d in data['sequence']:
        d['objects'] = objects_walls['objects']

    data['walls'] = objects_walls['walls']

    with open(file_name, 'w') as f:
        f.write(json.dumps(data))
        f.close()

        # options = jsbeautifier.default_options()
        # options.indent_size = 2
        # f.write(jsbeautifier.beautify(json.dumps(data), options))
        # # f.write(json.dumps(data))
        # f.close()

