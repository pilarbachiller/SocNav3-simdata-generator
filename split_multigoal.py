#!/usr/bin/python3
#
# -*- coding: utf-8 -*-
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# e.g.
# python3 view_data.py 2024-08-07T16-28-38_trj_checked.json --videowidth 1400 --videoheight 1400 --leftcrop 180 --rightcrop 430 --topcrop 170 --bottomcrop 520  --rotate 59.5 --ffwd --novideo
#
#

import time
import json
import copy

import argparse
import jsbeautifier

parser = argparse.ArgumentParser(
                    prog='view_data',
                    description='Displays social navigation interactions')
parser.add_argument('files', metavar='N', type=str, nargs="+")
args = parser.parse_args()



for file_name in args.files:
    print(file_name)

    read_goal = None

    data = json.load(open(file_name, 'r'))

    splits = [0]
    for seq_index, s in enumerate(data["sequence"]):
        force_split = False
        if read_goal is None or (abs(read_goal["x"]<=1e-5) and abs(read_goal["y"]<=1e-5) and abs(read_goal["angle"]<=1e-5)):
            read_goal = s["goal"]
        elif s["goal"] != read_goal:
            print(f"Change detected in {args.files}")
            force_split = True
            read_goal = s["goal"]
        if force_split:
            splits.append(seq_index)

    print("whole length:", len(data["sequence"]))
    splits.append(None)
    for chunk_idx, chunk_start in enumerate(splits[:-1]):
        try:
            chunk_end = splits[chunk_idx+1]
            # For every chunk corresponding to a single goal
            # We make a temporary copy of the data
            print(f"chunk number {chunk_idx} goes from {chunk_start} to {chunk_end}")
            copy_for_chunk = copy.deepcopy(data)
            copy_for_chunk["sequence"] = copy_for_chunk["sequence"][chunk_start:chunk_end]
            print("chunk length:", len(data["sequence"]))
            copy_for_chunk["sequence"][0]["goal"] = copy_for_chunk["sequence"][1]["goal"]
            chunk_file_name = f"{file_name.split('.')[0]}_g{chunk_idx}.json"
            print("Saving output to:", chunk_file_name)
            with open(chunk_file_name, 'w') as f:
                options = jsbeautifier.default_options()
                options.indent_size = 2
                f.write(jsbeautifier.beautify(json.dumps(copy_for_chunk), options))
        finally:
            time.sleep(1)