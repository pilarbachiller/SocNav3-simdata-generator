import json
import sys

if len(sys.argv)<2:
    print('Please, specify the name of the JSon file')
    exit()

file_name = sys.argv[1]
data = json.load(open(file_name, 'r'))
objects_walls = json.load(open('SACSoN_objects_walls.json','r'))

for d in data['sequence']:
    d['objects'] = objects_walls['objects']

with open(file_name, 'w') as f:
    f.write(json.dumps(data))
    f.close()

