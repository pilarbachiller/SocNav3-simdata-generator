import os
import sys
import json
import filecmp

JSON_DIR = "."


def remove_new_lines(path):
    with open(path, 'r') as file:
        data = "".join(file.readlines())
        data = data.replace("\n", "")
    with open(path, 'w') as file:
        file.write(data)
        file.close()


for filename in os.listdir(JSON_DIR):
    if filename.endswith('.json'):
        file_path = os.path.join(JSON_DIR, filename)
        file_path_nocolon = file_path.replace(":", "_")
        if file_path == file_path_nocolon:
            continue
        if os.path.exists(file_path) and os.path.exists(file_path_nocolon):
            print("File exists")
            remove_new_lines(file_path)
            remove_new_lines(file_path_nocolon)
        try:
            if filecmp.cmp(file_path_nocolon, file_path):
                print("remove file", file_path)
                os.remove(filename)
            else:
                print("Files differ:", file_path_nocolon, file_path)
        except FileNotFoundError:
            pass