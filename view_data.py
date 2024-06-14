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

import argparse

IMAGE_SIDE = 1000
HUMAN_RADIUS = 0.55/2
HUMAN_DEPTH = 0.20 / 2.
ROBOT_RADIUS = 0.25
GOAL_RADIUS = 0.3

parser = argparse.ArgumentParser(
                    prog='view_data',
                    description='Displays social navigation interactions')
parser.add_argument('file', metavar='N', type=str, nargs="?")
parser.add_argument('--xoffset', type=float, nargs="?", default=0., help='how much to add to x')
parser.add_argument('--yoffset', type=float, nargs="?", default=0., help='how much to add to y')
args = parser.parse_args()

def world_to_grid(pW, GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH):
    pGx = (args.xoffset+pW[0])/GRID_CELL_SIZE + GRID_WIDTH/2
    pGy = (args.yoffset+pW[1])/GRID_CELL_SIZE + GRID_HEIGHT/2
    return (int(pGx), int(pGy))

def rotate_points(points, center, angle):
    r_points = []
    for p in points:        
        p_x = center[0] - np.sin(angle) * (p[0] - center[0]) + np.cos(angle) * (p[1] - center[1])
        p_y = center[1] + np.cos(angle) * (p[0] - center[0]) + np.sin(angle) * (p[1] - center[1])
        r_points.append((p_x, p_y))
    return r_points


def rotate(x, y, radians):
    xx = -x * np.sin(radians) + y * np.cos(radians)
    yy = x * np.cos(radians) + y * np.sin(radians)
    return [xx, yy]

def draw_person(p, canvas, map_mult):
    w = HUMAN_RADIUS
    d = HUMAN_DEPTH
    a = p["angle"]
    offset = np.array([p["x"], p["y"]])

    rr = 0.05
    pts = np.array([rotate( 0, -d, a),

                    rotate(-(w-rr), -d, a),
                    rotate(-w, -(d-0.05), a),

                    rotate(-w, +(d-rr), a),
                    rotate(-(w-rr), +d, a),

                    rotate(+(w-rr), +d, a),
                    rotate(+w, +(d-rr), a),

                    rotate(+w, -(d-rr), a),
                    rotate(+(w-rr), -d, a),

                    rotate(0, -d, a)])
    pts += offset
    pts[:,0] = ((pts[:,0])*map_mult)+canvas.shape[0]/2
    pts[:,1] = canvas.shape[0]/2-(-(pts[:,1])*map_mult)
    pts = pts.reshape((1,-1,2)).astype(np.int32)
    cv2.fillPoly(canvas, pts, (20, 20, 60))

    pts = np.array(rotate(0, 0.05, a)) + offset
    pts[0] = ((pts[0])*map_mult)+canvas.shape[0]/2
    pts[1] = canvas.shape[0]/2-(-(pts[1])*map_mult)
    pts = pts.astype(np.int32)
    cv2.circle(canvas, (pts[0], pts[1]), 7, (50,40,170), -1)

    pts = np.array(rotate(0, 0.12, a)) + offset
    pts[0] = ((pts[0])*map_mult)+canvas.shape[0]/2
    pts[1] = canvas.shape[0]/2-(-(pts[1])*map_mult)
    pts = pts.astype(np.int32)
    cv2.circle(canvas, (pts[0], pts[1]), 3, (50,40,170), -1)

def draw_robot_and_goal(r, canvas):
    # DRAW ROBOT
    c = world_to_grid((r['x'], r['y']), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
    r_p = world_to_grid((r['x']+ROBOT_RADIUS, r['y']), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
    x_a = r['x'] + (ROBOT_RADIUS-0.1)*np.cos(r['angle'])
    y_a = r['y'] + (ROBOT_RADIUS-0.1)*np.sin(r['angle'])
    a = world_to_grid((x_a, y_a), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
    x_pa1 = r['x'] - (ROBOT_RADIUS-0.1)*np.sin(r['angle'])
    y_pa1 = r['y'] + (ROBOT_RADIUS-0.1)*np.cos(r['angle'])
    x_pa2 = r['x'] + (ROBOT_RADIUS-0.1)*np.sin(r['angle'])
    y_pa2 = r['y'] - (ROBOT_RADIUS-0.1)*np.cos(r['angle'])
    pa1 = world_to_grid((x_pa1, y_pa1), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
    pa2 = world_to_grid((x_pa2, y_pa2), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
    rad = abs(c[0]-r_p[0])
    cv2.circle(local_grid, c, rad, [252, 220, 202], -1)
    cv2.circle(local_grid, c, rad, [107, 36, 0], 2)
    cv2.line(local_grid, c, a, [107, 36, 0], 2)
    cv2.line(local_grid, pa1, pa2, [107, 36, 0], 2)

    # DRAW GOAL
    c = world_to_grid((r['goal_x'], r['goal_y']), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
    r_p = world_to_grid((r['goal_x']+GOAL_RADIUS, r['goal_y']), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
    r = abs(c[0]-r_p[0])
    cv2.circle(local_grid, c, r, [0, 255, 0], 2)


def draw_rectangular_object(canvas, c, angle, w, h, colorF, colorL):
        points = []
        points.append((c[0]-w/2, c[1]-h/2))
        points.append((c[0]+w/2, c[1]-h/2))
        points.append((c[0]+w/2, c[1]+h/2))
        points.append((c[0]-w/2, c[1]+h/2))
        r_points = rotate_points(points, c, angle)
        g_points = []
        for p in r_points:
            w_p = world_to_grid(p, GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
            g_points.append([int(w_p[0]), int(w_p[1])])
        cv2.fillPoly(canvas, [np.array(g_points, np.int32)], colorF) 
        cv2.polylines(canvas, [np.array(g_points, np.int32)], True, colorL, 4) 


def draw_object(o, canvas):
    if o["type"] == "table":
        cF = (63,133,205)
        cL = (23,93,165)
        draw_rectangular_object(canvas, (o["x"], o["y"]), o["angle"], o["size"][0], o["size"][1], cF, cL)
    elif o["type"] == "shelf":
        cF = (205,133,63)
        cL = (165,93,23)
        draw_rectangular_object(canvas, (o["x"], o["y"]), o["angle"], o["size"][0], o["size"][1], cF, cL)
    elif o["type"] == "TV":
        cF = (100,100,100)
        cL = (100,100,100)
        draw_rectangular_object(canvas, (o["x"], o["y"]), o["angle"], o["size"][0], o["size"][1], cF, cL)
    elif o["type"] == "plant":
        c = world_to_grid((o['x'], o['y']), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
        r_p = world_to_grid((o['x']+o['size'][0]/2, o['y']), GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
        r = abs(c[0]-r_p[0])
        cv2.circle(canvas, c, r, (29, 67, 105), -1)
        cv2.circle(canvas, c, r//2, (0, 200, 0), -1)
    else:
        cF = (200,200,200)
        cL = (140,140,140)
        draw_rectangular_object(canvas, (o["x"], o["y"]), o["angle"], o["size"][0], o["size"][1], cF, cL)


print(args.file)
data = json.load(open(args.file, 'r'))
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

scale = IMAGE_SIDE/GRID_WIDTH
GRID_WIDTH = IMAGE_SIDE
GRID_HEIGHT = int(GRID_HEIGHT*scale)
GRID_CELL_SIZE = GRID_CELL_SIZE/scale

global_grid = cv2.resize(global_grid, (GRID_HEIGHT, GRID_WIDTH))

last_timestamp = -1
for s in data["sequence"]:
    local_grid = copy.deepcopy(global_grid)
    # cv2.line(local_grid, (0, IMAGE_SIDE//2), (IMAGE_SIDE-1, IMAGE_SIDE//2), [0, 0, 0], 1)
    # cv2.line(local_grid, (IMAGE_SIDE//2, 0), (IMAGE_SIDE//2, IMAGE_SIDE-1), [0, 0, 0], 1)


    # DRAW HUMANS
    for p in s["people"]:
        draw_person(p, local_grid, 1./GRID_CELL_SIZE)

    # DRAW ROBOT AND GOAL
    if s["robot"]['x'] is None:
        continue
        
    draw_robot_and_goal(s["robot"], local_grid)

    # DRAW OBJECTS
    for o in s["objects"]:
        draw_object(o, local_grid)

    # DRAW WALLS
    for w in s["walls"]:
        p1 = (w[0], w[1])
        p2 = (w[2], w[3])
        p1G = world_to_grid(p1, GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
        p2G = world_to_grid(p2, GRID_CELL_SIZE, GRID_HEIGHT, GRID_WIDTH)
        cv2.line(local_grid, p1G, p2G, [0, 0, 255], 8)

    visible_grid = cv2.flip(local_grid, 0)                


    cv2.imshow("grid", visible_grid)
    k = cv2.waitKey(1)
    if k==27:
        exit()

    sleeptime = s["timestamp"]-last_timestamp
    if last_timestamp == -1:
        sleeptime = 0
    last_timestamp = s["timestamp"]
    time.sleep(sleeptime)

