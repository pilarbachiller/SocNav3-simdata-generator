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

import time, sys, os
from turtle import width
import numpy as np
import cv2
import json
import copy

import argparse

HUMAN_RADIUS = 0.55 / 2.
HUMAN_DEPTH =  0.20 / 2.

parser = argparse.ArgumentParser(
                    prog='view_data',
                    description='Displays social navigation interactions')
parser.add_argument('files', metavar='N', type=str, nargs="+")
parser.add_argument('--leftcrop', type=int, nargs="?", default=0., help='left cropping')
parser.add_argument('--topcrop', type=int, nargs="?", default=0., help='top cropping')
parser.add_argument('--rightcrop', type=int, nargs="?", default=0., help='right cropping')
parser.add_argument('--bottomcrop', type=int, nargs="?", default=0., help='bottom cropping')
parser.add_argument('--rotate', type=float, nargs="?", default=0., help='how much to add to angle') # -120.5``
parser.add_argument('--videowidth', type=int, help='video width', required=True)
parser.add_argument('--videoheight', type=int, help='video height', required=True)
parser.add_argument('--dir', type=str, nargs="?", default="./videos", help="output directory for the generated videos")


args = parser.parse_args()

output_dir = args.dir
if not os.path.isdir(output_dir):
    os.mkdir(output_dir)
print("Videos will be saved in", output_dir)

def world_to_grid(pW, GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG):
    pGx, pGy = world_to_grid_float(pW, GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    return (int(pGx), int(pGy))

def world_to_grid_float(pW, GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG):
    tx = pW[0]-GRID_X_ORIG
    ty = -pW[1]-GRID_Y_ORIG
    rx = tx*np.cos(-GRID_ANGLE_ORIG) - ty*np.sin(-GRID_ANGLE_ORIG)
    ry = tx*np.sin(-GRID_ANGLE_ORIG) + ty*np.cos(-GRID_ANGLE_ORIG)
    pGx = rx/GRID_CELL_SIZEX 
    pGy = ry/GRID_CELL_SIZEY 

    return pGx, pGy

def rad_to_degrees(rad):
    deg = rad*180/np.pi
    return deg

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

def draw_person(p, canvas, map_multX, map_multY, color):
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
    g_points = []
    for p in pts.tolist():
        w_p = world_to_grid(p, GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
        g_points.append([int(w_p[0]), int(w_p[1])])
    cv2.fillPoly(canvas, [np.array(g_points, np.int32)], color)         

    pts = np.array(rotate(0, 0.05, a)) + offset
    g_p = world_to_grid((pts[0],pts[1]), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    cv2.circle(canvas, g_p, 7, (50,40,170), -1)

    pts = np.array(rotate(0, 0.12, a)) + offset
    g_p = world_to_grid((pts[0],pts[1]), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    cv2.circle(canvas, g_p, 3, (50,40,170), -1)


    # cv2.putText(canvas, str(p["id"]),
    # org=(pts[0], pts[1]),
    # fontFace=cv2.FONT_HERSHEY_SIMPLEX,
    # fontScale=1.5,
    # color=(0, 0, 255))


def draw_robot(r, local_grid):
    ROBOT_RADIUS = r["shape"]["width"]/2

    x_a = r['x'] + (ROBOT_RADIUS-0.1)*np.cos(r['angle'])
    y_a = r['y'] + (ROBOT_RADIUS-0.1)*np.sin(r['angle'])
    a = world_to_grid((x_a, y_a), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    x_pa1 = r['x'] - (ROBOT_RADIUS-0.1)*np.sin(r['angle'])
    y_pa1 = r['y'] + (ROBOT_RADIUS-0.1)*np.cos(r['angle'])
    x_pa2 = r['x'] + (ROBOT_RADIUS-0.1)*np.sin(r['angle'])
    y_pa2 = r['y'] - (ROBOT_RADIUS-0.1)*np.cos(r['angle'])
    pa1 = world_to_grid((x_pa1, y_pa1), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    pa2 = world_to_grid((x_pa2, y_pa2), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    c = world_to_grid_float((r['x'], r['y']), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    r_p = world_to_grid_float((r['x']+ROBOT_RADIUS, r['y']), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    rad = int(abs(c[0]-r_p[0]))
    c = world_to_grid((r['x'], r['y']), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    cv2.circle(local_grid, c, rad, [252, 220, 202], -1)
    cv2.circle(local_grid, c, rad, [107, 36, 0], 2)
    cv2.line(local_grid, c, a, [107, 36, 0], 2)
    cv2.line(local_grid, pa1, pa2, [107, 36, 0], 2)

def draw_goal(g, robot_radius, local_grid):
    GOAL_RADIUS = robot_radius
    if "pos_threhold" in g.keys():
        GOAL_RADIUS += g["pos_threshold"]

    ANGLE_THRESHOLD = 0
    if "angle_threshold" in g.keys():
        ANGLE_THRESHOLD += g["angle_threshold"]

    # DRAW GOAL
    c = world_to_grid_float((g['x'], g['y']), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    r_p = world_to_grid_float((g['x']+GOAL_RADIUS, g['y']), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    rad = int(abs(c[0]-r_p[0]))
    c = world_to_grid((g['x'], g['y']), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)

    startAngle = np.arctan2(np.sin(g['angle']-ANGLE_THRESHOLD), np.cos(g['angle']-ANGLE_THRESHOLD))
    startAngle = rad_to_degrees(startAngle)
    endAngle = np.arctan2(np.sin(g['angle']+ANGLE_THRESHOLD), np.cos(g['angle']+ANGLE_THRESHOLD))
    endAngle = rad_to_degrees(endAngle)
    cv2.ellipse(local_grid, c, (rad, rad), 0, startAngle, endAngle, [0, 180, 0], -1)

    cv2.circle(local_grid, c, rad, [0, 100, 0], 2)
    x_a = g['x'] + (GOAL_RADIUS)*np.cos(g['angle'])
    y_a = g['y'] + (GOAL_RADIUS)*np.sin(g['angle'])
    a = world_to_grid((x_a, y_a), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
    cv2.line(local_grid, c, a, [0, 100, 0], 2)


def draw_rectangular_object(canvas, c, angle, w, h, colorF, colorL):
        points = []
        points.append((c[0]-w/2, c[1]-h/2))
        points.append((c[0]+w/2, c[1]-h/2))
        points.append((c[0]+w/2, c[1]+h/2))
        points.append((c[0]-w/2, c[1]+h/2))
        r_points = rotate_points(points, c, angle)
        g_points = []
        for p in r_points:
            w_p = world_to_grid(p, GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
            g_points.append([int(w_p[0]), int(w_p[1])])
        cv2.fillPoly(canvas, [np.array(g_points, np.int32)], colorF) 
        cv2.polylines(canvas, [np.array(g_points, np.int32)], True, colorL, 4) 

def draw_circular_object(canvas, c, angle, w, h, colorF, colorL):
        g_c = world_to_grid(c, GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
        g_w = int(w/GRID_CELL_SIZEX)//2
        g_h = int(h/GRID_CELL_SIZEY)//2
        rot = angle*180/np.pi + 90
        cv2.ellipse(canvas, g_c, (g_w, g_h), rot, 0, 360, colorF, -1)
        cv2.ellipse(canvas, g_c, (g_w, g_h), rot, 0, 360, colorL, 4)

def draw_chair(canvas, c, angle, w, l, colorF, colorL, shape = "rectangle"):        

        if shape == "rectangle":
            draw_rectangular_object(canvas, c, angle, w, l, colorF, colorL)
        else:
            draw_circular_object(canvas, c, angle, w, l, colorF, colorL)

        object_points = []

        s1 = 0.1
        s2 = 0.2

        p1 = [-w / 2, -l / 2]
        p2 = [w / 2, -l / 2]
        p3 = [w / 2, -l / 2  + l * s1]        
        p4 = [-w / 2, -l / 2 + l * s1]

        part_points = [p1, p2, p3, p4]

        object_points.append(part_points)

        p5 = [-w / 2, -l / 2 + l * s2]
        p6 = [-w / 2 + w * s1 , -l / 2 + l * s2]
        p7 = [-w / 2 + w * s1 , l / 2]
        p8 = [-w / 2, l / 2]

        part_points = [p5, p6, p7, p8]

        object_points.append(part_points)

        p9 = [w / 2, -l / 2 + l * s2]
        p10 = [w / 2 - w * s1 , -l / 2 + l * s2]
        p11 = [w / 2 - w * s1 , l / 2]
        p12 = [w / 2, l / 2]

        part_points = [p9, p10, p11, p12]

        object_points.append(part_points)

        im_op = []
        for op in object_points:
            pp = []
            for p in op:
                ip = [0]*2
                ip[0] = (c[0] - p[0] * np.sin(angle) + p[1] * np.cos(angle))
                ip[1] = (c[1] + p[0] * np.cos(angle) + p[1] * np.sin(angle))
                ip = world_to_grid(tuple(ip), GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
                pp.append(ip)
            im_op.append(pp)

        for ip in im_op:
            points = np.array(ip)
            points = points.reshape((-1, 1, 2))
            cv2.fillPoly(canvas, [np.int32(points)], colorF, cv2.LINE_AA)  # filling the rectangle made from the points with the specified color
            cv2.polylines(canvas, [np.int32(points)], True, colorL, 2, cv2.LINE_AA)  # bordering the rectangle


def draw_object(o, canvas):
    w = o["shape"]["width"]
    h = o["shape"]["height"] 
    if o["type"] == "table":
        cF = (63,133,205)
        cL = (23,93,165)
    elif o["type"] == "shelf":
        cF = (205,133,63)
        cL = (165,93,23)
    elif o["type"] == "TV":
        cF = (100,100,100)
        cL = (100,100,100)
    elif o["type"] == "plant":
        cF = (0, 200, 0)
        cL = (29, 67, 105)
    elif o["type"] == "chair":        
        cF = (200,200,200)
        cL = (140,140,140)
    else:
        cF = (200,200,200)
        cL = (140,140,140)

    if o["type"] == "chair":
        draw_chair(canvas, (o["x"], o["y"]), o["angle"], w, h, cF, cL, o["shape"]["type"])
    else:
        if o["shape"]["type"] == "circle":
            draw_circular_object(canvas, (o["x"], o["y"]), o["angle"], w, h, cF, cL)
        else:
            draw_rectangular_object(canvas, (o["x"], o["y"]), o["angle"], w, h, cF, cL)



#INITIALIZATIONS

for file_name in args.files:

    print(file_name)

    data = json.load(open(file_name, 'r'))
    grid = data["grid"]["data"]
    GRID_HEIGHT = data["grid"]["height"]
    GRID_WIDTH = data["grid"]["width"]
    GRID_CELL_SIZE = data["grid"]["cell_size"]
    grid = np.array(grid, np.int8)
    print("grid shape", grid.shape)

    v2gray = {-1:[128, 128, 128], 0: [255, 255, 255], 1: [0, 0, 0]}
    global_grid = np.zeros((GRID_HEIGHT, GRID_WIDTH, 3), np.uint8)
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            global_grid[y][x] = v2gray[grid[y][x]]

    scaleX = args.videowidth/GRID_WIDTH
    scaleY = args.videoheight/GRID_HEIGHT
    GRID_WIDTH = args.videowidth
    GRID_HEIGHT = args.videoheight
    GRID_CELL_SIZEX = GRID_CELL_SIZE/scaleX
    GRID_CELL_SIZEY = GRID_CELL_SIZE/scaleY

    # GRID_X_ORIG = data["grid"]["origin"][1]
    # GRID_Y_ORIG = data["grid"]["origin"][0]
    GRID_X_ORIG = data["grid"]["x_orig"]
    GRID_Y_ORIG = data["grid"]["y_orig"]
    GRID_ANGLE_ORIG = data["grid"]["angle_orig"]

    global_grid = cv2.resize(global_grid, (GRID_HEIGHT, GRID_WIDTH))

    # DRAW WALLS
    for w in data["walls"]:
        p1 = (w[0], w[1])
        p2 = (w[2], w[3])
        p1G = world_to_grid(p1, GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
        p2G = world_to_grid(p2, GRID_CELL_SIZEX, GRID_CELL_SIZEY, GRID_X_ORIG, GRID_Y_ORIG, GRID_ANGLE_ORIG)
        cv2.line(global_grid, p1G, p2G, [0, 0, 255], 8)


    if args.leftcrop > 0:
        if args.leftcrop < global_grid.shape[1]-args.rightcrop:
            lcrop = args.leftcrop
        else:
            print("ignoring left crop")
    else:
        lcrop = 0
    if args.rightcrop > 0:
        if args.rightcrop < global_grid.shape[1]-args.leftcrop:
            rcrop = args.rightcrop
        else:
            print("ignoring right crop")
    else:
        rcrop = 0
    if args.topcrop > 0:
        if args.topcrop < global_grid.shape[0]-args.bottomcrop:
            tcrop = args.topcrop
        else:
            print("ignoring top crop")
    else:
        tcrop = 0
    if args.bottomcrop > 0:
        if args.bottomcrop < global_grid.shape[0]-args.topcrop:
            bcrop = args.bottomcrop
        else:
            print("ignoring bottom crop")
    else:
        bcrop = 0

    images_for_video = []
    last_timestamp = -1
    human_colors = {}
    for s in data["sequence"]:
        local_grid = copy.deepcopy(global_grid)

        if s["robot"]['x'] is None:
            continue

        # DRAW ROBOT AND GOAL
        draw_goal(s["goal"], s["robot"]["shape"]["width"]/2, local_grid)    
        draw_robot(s["robot"], local_grid)

        # DRAW OBJECTS
        for o in s["objects"]:
            draw_object(o, local_grid)

        # DRAW HUMANS
        for p in s["people"]:
            if p["id"] in human_colors.keys():
                color = human_colors[p["id"]]
            else:
                color = tuple(np.random.choice(range(256), size=3).astype(np.uint8))
                human_colors[p["id"]] = color
            draw_person(p, local_grid, 1./GRID_CELL_SIZEX, 1./GRID_CELL_SIZEY, (int(color[0]), int(color[1]), int(color[2])))


        # visible_grid = cv2.flip(local_grid, 0)              
        visible_grid = local_grid  

        R = cv2.getRotationMatrix2D((visible_grid.shape[0]//2, visible_grid.shape[1]//2), args.rotate, 1.0)
        to_show = cv2.warpAffine(visible_grid, R, (visible_grid.shape[0], visible_grid.shape[1]), borderValue=(127,127,127))


        to_show = to_show[tcrop:-bcrop-1,lcrop:-rcrop-1]

        images_for_video.append(to_show)
        cv2.imshow("grid", to_show)
        k = cv2.waitKey(1)
        if k==27:
            exit()

        sleeptime = s["timestamp"]-last_timestamp
        if last_timestamp == -1:
            sleeptime = 0
        last_timestamp = s["timestamp"]
        time.sleep(sleeptime)

    ini_episode = data["sequence"][0]["timestamp"]
    end_episode = data["sequence"][-1]["timestamp"]
    fps = len(images_for_video)/(end_episode-ini_episode)
    fourcc =  cv2.VideoWriter_fourcc(*'MP4V')
    output_file = file_name.split("/")[-1].split(".")[0] + ".mp4"
    writer = cv2.VideoWriter(os.path.join(output_dir, output_file), fourcc, fps, (images_for_video[0].shape[1], images_for_video[0].shape[0])) 
    for image in images_for_video:
        writer.write(image)
    writer.release()
