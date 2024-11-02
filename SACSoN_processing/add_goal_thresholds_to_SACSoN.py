import json
import jsbeautifier
import sys

if len(sys.argv)<2:
    print('Please, specify the name of the JSon files')
    exit()

files = sys.argv[1:]

for file_name in files:
    data = json.load(open(file_name, 'r'))

    for d in data['sequence']:
        d['goal']['pos_threshold'] = 0
        d['goal']['angle_threshold'] = 0


    with open(file_name, 'w') as f:
        options = jsbeautifier.default_options()
        options.indent_size = 2
        f.write(jsbeautifier.beautify(json.dumps(data), options))
        # f.write(json.dumps(data))
        f.close()

