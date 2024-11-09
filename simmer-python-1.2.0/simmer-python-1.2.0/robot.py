'''
Defines the SimMeR Robot class.
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

import math
import pygame
import pygame.math as pm
import numpy as np
from pygame.locals import (
    K_w,
    K_a,
    K_s,
    K_d,
    K_q,
    K_e,
    K_t
)

import config as CONFIG
import utilities

class Robot():
    '''This class represents the robot'''

    def __init__(self):
        '''Initialize the robot class'''

        # Position information (stored in inches)
        self.position = pm.Vector2(CONFIG.robot_start_position[0], CONFIG.robot_start_position[1])
        self.rotation = CONFIG.robot_start_rotation

        # Robot size (rectangular)
        self.width = float(CONFIG.robot_width)
        self.height = float(CONFIG.robot_height)

        # Define the outline of the robot as a polygon
        self.outline = CONFIG.robot_outline

        self.outline_global = []
        self.outline_global_segments = []
        self.update_outline()

        # Is the robot currently colliding with a maze wall?
        self.collision = False

        # A trail of points where the robot has moved
        self.trail = [{
            'position': self.position,
            'rotation': self.rotation,
            'collision': self.collision
        }]

        # Import the list of motors from the config file
        self.motors = CONFIG.motors

        # Import the list of drives from the config file
        self.drives = CONFIG.drives

        # Import the list of sensors from the config file
        self.sensors = CONFIG.sensors

        # All devices
        self.devices = self.motors | self.drives | self.sensors
        
        # Initialize directional movement tracking
        self.y_direction = 0 
        self.x_direction = 0
        self.rotation_change = 0  
        
        # Robot size and wheel properties
        self.wheel_diameter = 2.28346  # in inches
        self.wheel_radius = self.wheel_diameter / 2
        self.wheel_midline_distance = 3.3464567  # Distance from center to each wheel, in inches
        self.wheel_circumference = math.pi * self.wheel_diameter
        
        # Track rotation for each motor in degrees
        self.wheel_rotations = {
            'm1': 0,  # Front-left motor rotation in degrees
            'm2': 0,  # Front-right motor rotation in degrees
            'm3': 0   # Rear motor rotation in degrees
        }


    def append_trail(self):
        '''Appends current position information to the robot's trail'''

        self.trail.append({
            'position': self.position,
            'rotation': self.rotation,
            'collision': self.collision
        })

    def update_outline(self):
        '''
        Define the absolute outline points of the robot, in inches, relative
        to the center point of the robot.
        '''

        # Rotate the outline
        outline_global = [point.rotate(self.rotation) for point in self.outline]

        # Place the outline in the right location
        self.outline_global = [point + self.position for point in outline_global]

        # Convert the outline points to line segments
        segments = []
        for ct in range(-1, len(self.outline_global) - 1):
            segments.append((self.outline_global[ct], self.outline_global[ct+1]))

        self.outline_global_segments = segments

    def draw(self, canvas):
        '''Draws the robot outline on the canvas'''

        # Graphics
        THICKNESS = int(CONFIG.robot_thickness * CONFIG.ppi)
        COLOR = CONFIG.robot_color

        # Convert the outline from inches to pixels
        outline = [point * CONFIG.ppi + [CONFIG.border_pixels, CONFIG.border_pixels]
                   for point in self.outline_global]

        # Draw the polygon
        pygame.draw.polygon(canvas, COLOR, outline, THICKNESS)

    def update_device_positions(self):
        '''
        Updates the global positions and outlines of all the robot's devices.
        '''
        for device in self.devices.values():
            device.pos_update(self.position, self.rotation)
            device.update_outline()

    def draw_devices(self, canvas):
        '''
        Draws all devices on the robot onto the canvas unless marked otherwise.
        '''

        for device in self.devices.values():
            if device.visible:
                device.draw(canvas)
                if device.visible_measurement:
                    device.draw_measurement(canvas)

    def move_manual(self, keypress, walls):
        '''Determine the direction to move & rotate the robot based on keypresses.'''

        move_vector = pm.Vector2(0, 0)
        rotation = 0
        speed = 6 / CONFIG.frame_rate               # inch/s / frame/s
        rotation_speed = 120 / CONFIG.frame_rate    # deg/s / frame/s

        # Forward/backward movement
        if keypress[K_w]:
            move_vector += [0, speed]
        if keypress[K_s]:
            move_vector += [0, -speed]

        # Left/right movement
        if keypress[K_q]:
            move_vector += [speed, 0]
        if keypress[K_e]:
            move_vector += [-speed, 0]

        # Rotation
        if keypress[K_d]:
            rotation += rotation_speed
        if keypress[K_a]:
            rotation += -rotation_speed

        # Print total movement if "z" key is pressed
        if keypress[pygame.K_z]:
            print("\nTotal Movement Summary:")
            print(f"Total Y Movement: {self.y_direction:.2f} inches") 
            print(f"Total X Movement: {self.x_direction:.2f} inches")
            print(f"Total Rotation: {self.rotation_change:.2f} degrees")
            
        # Print total wheel rotations if "c" key is pressed
        if keypress[pygame.K_c]:
            print("\nTotal Wheel Rotations:")
            print(f"Motor m1 Rotation: {self.wheel_rotations['m1']:.2f} degrees") # front left
            print(f"Motor m2 Rotation: {self.wheel_rotations['m2']:.2f} degrees") # front right
            print(f"Motor m3 Rotation: {self.wheel_rotations['m3']:.2f} degrees") # back
            
        
        # Print steps for each motor if "V" key is pressed
        if keypress[pygame.K_v]:
            print("\nSteps for each motor based on current rotation:")

            # Calculate steps for each motor separately - may need to adust microstepping
            steps_m1 = self.degrees_to_steps(self.wheel_rotations['m1']) # front left
            steps_m2 = self.degrees_to_steps(self.wheel_rotations['m2']) # front right
            steps_m3 = self.degrees_to_steps(self.wheel_rotations['m3']) # back

            # Print the steps for each motor
            print(f"Motor m1 Steps: {steps_m1}") # front left
            print(f"Motor m2 Steps: {steps_m2}") # front right
            print(f"Motor m3 Steps: {steps_m3}") # back
            
        # Move the robot
        self.move(move_vector, rotation, walls)

    def move_from_command(self, walls):
        '''Move the robot based on all the movement "stored" in the drives'''

        move_vector = pm.Vector2(0, 0)
        rotation = 0
        for drive in self.drives.values():
            # Get the movement amount from the drive, incrementing odometers
            if drive.move_buffer == 0:
                continue
            movement = drive.move_update()
            move_vector += movement[0]
            rotation += movement[1]

        # print(f"{self.motors['m1'].odometer}, {self.motors['m2'].odometer}, {self.motors['m3'].odometer}")
        
        # Move the robot
        self.move(move_vector, rotation, walls)
        
    def track_movement(self, position_change, rotation_change, direction):
        '''Tracks the cumulative position and rotation changes.'''
        # Track position based on direction
        if direction == "forward":
            self.y_direction += position_change.length() 
        elif direction == "backward":
            self.y_direction -= position_change.length()
        elif direction == "left":
            self.x_direction -= position_change.length()
        elif direction == "right":
            self.x_direction += position_change.length()
    
        # Track total rotation
        self.rotation_change += rotation_change
    
    def track_wheel_rotation(self, x_movement, y_movement, rotation_change):
        '''
        Calculates and updates the rotation for each wheel in degrees
        based on the robot's X and Y movement and rotation.
        '''
        # Decompose movement into each motor's contribution for a kiwi drive
        # m1_movement = (-x_movement / 2) + (y_movement / math.sqrt(2)) + (math.radians(rotation_change) * self.wheel_midline_distance) # front left
        # m2_movement = (x_movement / 2) + (y_movement / math.sqrt(2)) + (math.radians(rotation_change) * self.wheel_midline_distance) # front right
        # m3_movement = -x_movement + (math.radians(rotation_change) * self.wheel_midline_distance) # back
        
        # Decompose movement into each motor's contribution for a kiwi drive
        # Y-movement, X-movement, Rotation
        
        
        # Convert distance traveled to rotation in degrees for each motor (define CW as positive)
        # m1_rotation_degrees = (m1_movement / self.wheel_circumference) * 360 # front left
        # m2_rotation_degrees = (-1)*(m2_movement / self.wheel_circumference) * 360 # front right
        # m3_rotation_degrees = (-1)*(m3_movement / self.wheel_circumference) * 360  # back
        
        # Decompose movement into each motor's contribution for a kiwi drive
        # Y-movement, X-movement, Rotation
        m1_rotation_degrees = -(y_movement * math.cos(math.radians(30)) * 360 / (self.wheel_diameter * math.pi)) + (x_movement * math.cos(math.radians(60)) * 360 / (self.wheel_diameter  * math.pi)) + ((self.wheel_midline_distance / (self.wheel_diameter/2)) * rotation_change)
        m2_rotation_degrees = (y_movement * math.cos(math.radians(30)) * 360 / (self.wheel_diameter * math.pi)) + (x_movement * math.cos(math.radians(60)) * 360 / (self.wheel_diameter  * math.pi)) + ((self.wheel_midline_distance / (self.wheel_diameter/2)) * rotation_change)
        m3_rotation_degrees = 0 + -((x_movement * 360) / (self.wheel_diameter * math.pi)) + ((self.wheel_midline_distance / (self.wheel_diameter/2)) * rotation_change)
        
        # # Check if both m1_rotation_degrees and m2_rotation_degrees have the same sign
        # if np.sign(m2_rotation_degrees) == np.sign(m3_rotation_degrees):
        #     m1_rotation_degrees = np.sign(m2_rotation_degrees)*(m1_movement / self.wheel_circumference) * 360 # front left 
        # else:
        #     m1_rotation_degrees = (m1_movement / self.wheel_circumference) * 360  # back, negative rotation 

        # Update cumulative rotations for each motor, # everything is good except for pure rotation
        self.wheel_rotations['m1'] += m1_rotation_degrees # front left
        self.wheel_rotations['m2'] += m2_rotation_degrees # front right
        self.wheel_rotations['m3'] += m3_rotation_degrees # back

    def move(self, velocity, rotation, walls):
        '''Moves the robot, checking for collisions.'''
        # Track changes before updating position
        position_change = pm.Vector2.rotate(velocity, self.rotation)
        rotation_change = rotation
        
        x_movement = position_change.x
        y_movement = position_change.y

        # Determine direction based on velocity
        if velocity.y > 0:  # Moving forward
            direction = "forward"
        elif velocity.y < 0:  # Moving backward
            direction = "backward"
        elif velocity.x > 0:  # Moving left
            direction = "left"
        elif velocity.x < 0:  # Moving right
            direction = "right"
        else:
            direction = None  # No movement

        # Track cumulative changes only once
        if direction:
            self.track_movement(position_change, rotation_change, direction)
        else:
            self.track_movement(pm.Vector2(0, 0), rotation_change, "rotation")

        # Update robot position and rotation
        self.position += position_change
        self.rotation += rotation_change
        self.update_outline()
        
        # Track wheel rotations based on movement
        self.track_wheel_rotation(x_movement, y_movement, rotation_change)


        # Reset the position if a collision is detected
        # collisions = self.check_collision_walls_fast(walls)
        # if collisions:
        #     self.position -= pm.Vector2.rotate(velocity, self.rotation)
        #     self.rotation -= rotation
        #     self.update_outline()
        
    def get_total_movement(self):
        '''Returns the total cumulative position and rotation changes.'''
        return {
            "total_position_change": self.total_position_change.length(),  # Total distance traveled
            "total_rotation_change": self.total_rotation_change  # Total rotation in degrees
        }
        
    def get_wheel_rotations(self):
        '''Returns the current rotation for each wheel in degrees.'''
        return self.wheel_rotations
    
    def degrees_to_steps(self, rotation_degrees, microstepping=1):
        """
        Converts a rotation in degrees to the required number of steps
        for the NEMA 17 motor with the specified microstepping.

        Parameters:
        - rotation_degrees: float - The rotation in degrees for the motor.
        - microstepping: int - The microstepping factor of the DRV8825 driver
          (1 for full steps, 2 for half steps, up to 32 for 1/32 microstepping).
        
        Returns:
        - steps: int - The equivalent number of steps for the specified rotation.
        """
        # Define steps per revolution and step angle in degrees for a NEMA 17 motor
        steps_per_revolution = 200*16
        step_angle_degrees = 360 / steps_per_revolution  # 1.8 degrees per full step

        # Calculate the total steps with microstepping
        steps = (rotation_degrees / step_angle_degrees) * microstepping
        
        # Round to the nearest integer for actual motor steps and return
        return int(round(steps))
    
    def get_motor_steps(self):
        """
        Retrieves the current rotation in degrees for each motor, converts it to steps,
        and returns the steps as a list.

        Returns:
        - list of int: [steps_m1, steps_m2, steps_m3] 
        """
        # Get the current wheel rotations in degrees
        wheel_rotations = self.get_wheel_rotations()

        # Convert each motor's rotation to steps using degrees_to_steps
        steps_m1 = self.degrees_to_steps(wheel_rotations['m1'], microstepping=1) # front left
        steps_m2 = self.degrees_to_steps(wheel_rotations['m2'], microstepping=1) # front right
        steps_m3 = self.degrees_to_steps(wheel_rotations['m3'], microstepping=1) # back

        # Return the steps as a list
        return [steps_m1, steps_m2, steps_m3] # front left, front right, back

    def teleport(self, x, y, angle, walls):
        '''Attempts to teleport the robot to a location,
        returns True if successful
        if collision, reverts to previous location and returns False'''

        original_position = [self.position, self.rotation]

        self.position = pm.Vector2(x, y)
        self.rotation = angle
        self.update_outline()
        print(CONFIG.maze_dim_x, CONFIG.maze_dim_y)

        # Returns False if the selected position is outside of the bounds of the map
        if not (0 < self.position.x < CONFIG.maze_dim_x and 0 < self.position.y < CONFIG.maze_dim_y):
            self.position = original_position[0]
            self.rotation = original_position[1]
            self.update_outline()
            return False

        # Returns True if the robot isn't inside a block and if there's no intersection with walls.
        if not utilities.in_block(self.position) and not self.check_collision_walls_fast(walls):
            return True
        else:
            self.position = original_position[0]
            self.rotation = original_position[1]
            self.update_outline()
            return False

    def stop_drives(self):
        '''Stops all drives from moving, used as an emergency stop.'''
        for drive in self.drives.values():
            drive.move_buffer = 0

    def check_collision_walls(self, walls: list):
        '''
        Checks for a collision between the robot's perimeter segments
        and a set of wall line segments.
        '''

        # Loop through all the robot outline line segments, checking for collisions
        for segment_bot in self.outline_global_segments:
            for square in walls:
                for segment_wall in square:
                    collision_points = utilities.collision(segment_bot, segment_wall)
                    if collision_points:
                        return collision_points

    def check_collision_walls_fast(self, walls: list)->bool:
        '''
        Checks for a collision between the robot's perimeter segments
        and a set of wall line segments.
        '''

        # Loop through all the robot outline line segments, checking for collisions
        for segment_bot in self.outline_global_segments:
            for segment_wall in walls:
                collides = utilities.check_collision_fast(
                    segment_bot, segment_wall
                )  # bool value
                if collides:
                    return True

        return False



    def command(self, cmds: list, environment: dict):
        '''
        Parse text string of commands and act on them, sending them to the appropriate
        device.
        '''

        responses = []
        for cmd in cmds:
            # Get the target device based on ID string, return False if it doesn't exist
            target_device = self.devices.get(cmd[0], False)
            # just add the odometer value to responses
            

            if target_device:
                try:
                    value = float(cmd[1])
                except ValueError:
                    print('Command data (' + cmd[1] + ') not in valid float format. Trying with 0.')
                    value = 0
                
                responses.append([cmd[0], target_device.simulate(value, environment)])
            else:
                if cmd[0] == 'xx':
                    self.stop_drives()
                    responses.append([cmd[0], 'DRIVE STOP'])
                else:
                    print('Target device ' + cmd[0] + ' not found.')
                    responses.append([cmd[0], 'Not Found'])

        return responses