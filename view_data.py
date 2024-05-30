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

import time, sys
import numpy as np
import cv2
import json
import copy

def world_to_grid(pW, GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH):
    pGx = pW[0]*100/GRID_CELL_SIZE + GRID_WIDTH/2
    pGy = pW[1]*100/GRID_CELL_SIZE + GRID_HEIGHT/2
    return (int(pGx), int(pGy))


data = json.load(open(sys.argv[-1], 'r'))
grid = data["grid"]["data"]
GRID_HEIGHT = data["grid"]["height"]
GRID_WIDTH = data["grid"]["width"]
GRID_CELL_SIZE = data["grid"]["cell_size"]
grid = np.array(grid, np.int8)

v2gray = {-1:[128, 128, 128], 0: [255, 255, 255], 1: [0, 0, 0]}
global_grid = np.zeros((GRID_HEIGHT, GRID_WIDTH, 3), np.uint8)
for y in range(grid.shape[0]):
    for x in range(grid.shape[1]):
        global_grid[y][x] = v2gray[grid[y][x]]

last_timestamp = -1
for s in data["sequence"]:
    local_grid = copy.deepcopy(global_grid)

    for p in s["people"]:
        c = world_to_grid((p['x'], p['y']), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
        r_p = world_to_grid((p['x']+0.35, p['y']), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
        r = abs(c[0]-r_p[0])
        cv2.circle(local_grid, c, r, [0, 0, 255])


    visible_grid = cv2.resize(local_grid, (400, 400))
    visible_grid = cv2.flip(visible_grid, 0)                


    cv2.imshow("grid", visible_grid)
    cv2.waitKey(1)

    sleeptime = s["timestamp"]-last_timestamp
    if last_timestamp == -1:
        sleeptime = 0
    last_timestamp = s["timestamp"]
    time.sleep(sleeptime)



# for datum in data:

#     cv2.imshow(sys,argv[-1], picture)
#     k = cv2.waitKey(0)
#     if k%256 == 27:
#         print("Escape hit, closing...")
#         cv2.destroyAllWindows()
#         sys.exit(0)





