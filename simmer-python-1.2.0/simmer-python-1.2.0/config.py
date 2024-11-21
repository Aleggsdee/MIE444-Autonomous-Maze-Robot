'''
This file stores the configuration information for the simulator.
'''
# This file is part of SimMeR, an educational mechatronics robotics simulator.
# Initial development funded by the University of Toronto MIE Department.
# Copyright (C) 2023  Ian G. Bennett
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import pygame.math as pm
import math
from devices.motors import MotorSimple
from devices.ToF import ToF
from devices.drive import Drive
import numpy as np
import random

# Control Flags and Setup
rand_error = False          # Use either true random error generator (True) or repeatable error generation (False)
rand_bias = False           # Use a randomized, normally distributed set of bias values for drives (placeholder, not implemented)
bias_strength = [0.05, 1]   # How intense the random drive bias is, if enabled (placeholder, not implemented)

# Network configuration for sockets
host = '127.0.0.1'
port_rx = 61200
port_tx = 61201
timeout = 300
str_encoding = 'ascii'
frame_start = '['
frame_end = ']'

# General communication settings
round_digits = 3

# Block information
block_position = [21, 3]        # Block starting location
block_rotation = 0              # Block rotation (deg)
block_size = 1.5                  # Block side length in inches

# Robot information
robot_start_position = [6, 24]  # Robot starting location (in)
robot_start_rotation = 180      # Robot starting rotation (deg)
robot_width = 6                 # Robot width in inches
robot_height = 6                # Robot height in inches
radius = 3.54
num_points = 20  # Number of points to approximate the circle

# Generate robot outline as a circle
robot_outline = [
    pm.Vector2(math.cos(2 * math.pi / num_points * i) * radius, 
               math.sin(2 * math.pi / num_points * i) * radius)
    for i in range(num_points)
]

# Particle information
particle_width = 6                 # Particle width in inches
particle_height = 6                # Particle height in inches
radius = 3.54
num_points = 20  # Number of points to approximate the circle

# Generate robot outline as a circle
particle_outline = [
    pm.Vector2(0, 0.5),
    pm.Vector2(0, -0.5)
]

# Maze definition information
wall_segment_length = 12    # Length of maze wall segments (inches)
floor_segment_length = 3    # Size of floor pattern squares (inches)
walls = [[3,3,1,1,0,2,0,2],
         [3,3,0,1,1,1,1,1],
         [1,0,2,0,0,1,0,1],
         [1,1,1,1,1,1,0,2]] # Matrix to define the maze walls
floor_seed = 5489           # Randomization seed for generating correctfloor pattern
maze_dim_x = len(walls[0])*wall_segment_length
maze_dim_y = len(walls)*wall_segment_length


# Graphics information
frame_rate = 60             # Target frame rate (Hz)
ppi = 12                    # Number of on-screen pixels per inch on display
border_pixels = floor_segment_length * ppi  # Size of the border surrounding the maze area

background_color = (43, 122, 120)

wall_thickness = 0.25       # Thickness to draw wall segments, in inches
wall_color = (255, 0, 0)    # Tuple with wall color in (R,G,B) format

robot_thickness = 0.25      # Thickness to draw robot perimeter, in inches
robot_color = (0, 0, 255)   # Tuple with robot perimeter color in (R,G,B) format

particle_thickness = 0.25      # Thickness to draw particle perimeter, in inches
particle_color = (255, 0, 0)   # Tuple with particle perimeter color in (R,G,B) format

block_thickness = 0.25      # Thickness to draw block perimeter, in inches
block_color = (127, 127, 0) # Tuple with block perimeter color in (R,G,B) format



### DEVICE CONFIGURATION ###
# Motors
m1_info = {
    'id': 'm1',
    'position': [2.898, 1.673],
    'rotation': 30,
    'visible': True
}

m2_info = {
    'id': 'm2',
    'position': [-2.898, 1.673],
    'rotation': 150,
    'visible': True
}

m3_info = {
    'id': 'm3',
    'position': [0, -3.3465],
    'rotation': -90,
    'visible': True
}

motors = {
    'm1': MotorSimple(m1_info),
    'm2': MotorSimple(m2_info),
    'm3': MotorSimple(m3_info)
}

# Drives
w0_info = {
    'id': 'w0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 4.67],
    'ang_velocity': 0,
    'motors': [motors['m1'], motors['m2']],
    'motor_direction': [1, -1], # TODO: might need to check these positive negative values
    # 'bias': {'x': 0.01, 'y': 0, 'rotation': 0.02},
    # 'error': {'x': 0.01, 'y': 0.01, 'rotation': 0.01}
}

d0_info = {
    'id': 'd0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [-4.67, 0],
    'ang_velocity': 0,
    'motors': [motors['m1'], motors['m2'], motors['m3']],
    'motor_direction': [1, 1, -1], # TODO: might need to check these positive negative values
    # 'bias': {'x': 0.01, 'y': 0.01, 'rotation': 0.02},
    # 'error': {'x': 0.01, 'y': 0.01, 'rotation': 0.01}
}

r0_info = {
    'id': 'r0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 0],
    'ang_velocity': 100,
    'motors': [motors['m1'], motors['m2'], motors['m3']],
    'motor_direction': [1, 1, 1],
    # 'bias': {'x': 0, 'y': 0, 'rotation': 0.01},
    # 'error': {'x': 0.003, 'y': 0.003, 'rotation': 0.02}
}

drives = {
    'w0': Drive(w0_info),
    'd0': Drive(d0_info),
    'r0': Drive(r0_info)
}

# Sensors
t0_info = { # Left sensor
    'id': 't0',
    'position': [3, 0],
    'height': 8,
    'rotation': -90,
    'error': 0.0,
    'outline': [
        pm.Vector2(-1, -0.5),
        pm.Vector2(-1, 0.5),
        pm.Vector2(1, 0.5),
        pm.Vector2(1, -0.5)
    ],
    'visible': True,
    'visible_measurement': True
}

t1_info = { # Left 60 sensor
    'id': 't1',
    'position': [2.6, 1.5],
    'height': 8,
    'rotation': -60,
    'error': 0.0,
    'outline': [
        pm.Vector2(-1, -0.5),
        pm.Vector2(-1, 0.5),
        pm.Vector2(1, 0.5),
        pm.Vector2(1, -0.5)
    ],
    'visible': True,
    'visible_measurement': True
}

t2_info = { # Left 30 sensor
    'id': 't2',
    'position': [1.5, 2.6],
    'height': 8,
    'rotation': -30,
    'error': 0.0,
    'outline': [
        pm.Vector2(-1, -0.5),
        pm.Vector2(-1, 0.5),
        pm.Vector2(1, 0.5),
        pm.Vector2(1, -0.5)
    ],
    'visible': True,
    'visible_measurement': True
}

t3_info = { # front sensor
    'id': 't3',
    'position': [0, 3],
    'height': 8,
    'rotation': 0,
    'error': 0.0,
    'outline': [
        pm.Vector2(-1, -0.5),
        pm.Vector2(-1, 0.5),
        pm.Vector2(1, 0.5),
        pm.Vector2(1, -0.5)
    ],
    'visible': True,
    'visible_measurement': True
}

t4_info = { # right 30 sensor
    'id': 't4',
    'position': [-1.5, 2.6],
    'height': 8,
    'rotation': 30,
    'error': 0.0,
    'outline': [
        pm.Vector2(-1, -0.5),
        pm.Vector2(-1, 0.5),
        pm.Vector2(1, 0.5),
        pm.Vector2(1, -0.5)
    ],
    'visible': True,
    'visible_measurement': True
}

t5_info = { # right 60 sensor
    'id': 't5',
    'position': [-2.6, 1.5],
    'height': 8,
    'rotation': 60,
    'error': 0.0,
    'outline': [
        pm.Vector2(-1, -0.5),
        pm.Vector2(-1, 0.5),
        pm.Vector2(1, 0.5),
        pm.Vector2(1, -0.5)
    ],
    'visible': True,
    'visible_measurement': True
}

t6_info = { # right sensor
    'id': 't6',
    'position': [-3, 0],
    'height': 8,
    'rotation': 90,
    'error': 0.0,
    'outline': [
        pm.Vector2(-1, -0.5),
        pm.Vector2(-1, 0.5),
        pm.Vector2(1, 0.5),
        pm.Vector2(1, -0.5)
    ],
    'visible': True,
    'visible_measurement': True
}

t7_info = { # bottom sensor
    'id': 't7',
    'position': [0, 3],
    'height': 1,
    'rotation': 0,
    'error': 0.0,
    'outline': [
        pm.Vector2(-1, -0.5),
        pm.Vector2(-1, 0.5),
        pm.Vector2(1, 0.5),
        pm.Vector2(1, -0.5)
    ],
    'visible': True,
    'visible_measurement': True
}

sensors = {
    't0': ToF(t0_info),
    't1': ToF(t1_info),
    't2': ToF(t2_info),
    't3': ToF(t3_info),
    't4': ToF(t4_info),
    't5': ToF(t5_info),
    't6': ToF(t6_info),
    't7': ToF(t7_info),
}

### TESTING AND DEBUG SETTINGS ###
simulate_list = ['t0', 't1', 't2', 't3', 't4', 't5', 't6', 't7']