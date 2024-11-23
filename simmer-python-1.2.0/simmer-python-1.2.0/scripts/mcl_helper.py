import pygame
import random
import math
import numpy as np
import copy
from filterpy.monte_carlo import stratified_resample
import time
import pickle

# Constants
GRID_WIDTH = 96 + 2 # inches (one extra inch on each side for border)
GRID_HEIGHT = 48 + 2 # inches (one extra inch on each side for border)
ppi = 3
WINDOW_WIDTH = GRID_WIDTH * ppi
WINDOW_HEIGHT = GRID_HEIGHT * ppi
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
MAX_SENSOR_READING = 6000.0 / 127.0
MIN_SENSOR_READING = 3.0 / 2.54
MIN_SENSOR_STD = 1 # 1, 1.5, 2 (usually 1)
MAX_SENSOR_STD = 2.5 # 2, 2.5, 3 (usually 2.5)
certainty = 0


# Simulation parameters
NUM_PARTICLES = 5000

# Noise parameters
MOVEMENT_NOISE = 0.025


with open('sensor_data_3_ppi_15_beam_angle.pkl', 'rb') as f:
    loaded_sensor_readings = pickle.load(f)


def init_grid():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space
    grid[4:46, 4:10] = 0
    grid[4:22, 10:22] = 0
    grid[4:10, 22:40] = 0
    grid[28:40, 28:34] = 0
    grid[4:22, 40:46] = 0
    grid[16:22, 46:64] = 0
    grid[4:46, 64:70] = 0
    grid[40:46, 10:64] = 0
    grid[16:22, 64:88] = 0
    grid[4:46, 88:94] = 0

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid


# navigation matrix to loading zone
def init_loading_zone_grid():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set Loading Zone
    grid[3:23, 3:23] = 7
    
    # Set free movable space
    grid[20:47, 3:11] = 270
    grid[3:11, 20:47] = 180
    grid[11:23, 39:47] = 270
    grid[15:23, 47:95] = 180
    grid[3:15, 63:71] = 90
    grid[39:47, 11:71] = 180
    grid[27:39, 27:35] = 90
    grid[23:39, 63:71] = 270
    grid[3:15, 87:95] = 90
    grid[23:47, 87:95] = 270

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid

def init_drop_off_zone_grid1():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[4:46, 4:10] = 270 #270
    grid[3:11, 22:47] = 180 #180
    grid[11:23, 39:47] = 270 #270
    grid[15:23, 47:95] = 180 #180
    grid[3:15, 63:71] = 90 #90
    grid[39:47, 10:71] = 180 #180
    grid[23:39, 63:71] = 270 #270
    grid[3:15, 87:95] = 90 #90
    grid[23:47, 87:95] = 270 #270
    
    
    grid[3:22, 4:23] = 180 #180
    grid[3:40, 3:11] = 90 # 90
    grid[39:47, 3:28] = 360 #360
    grid[28:47, 27:35] = 270 #270
    
    # Define drop off zone 1
    grid[27:37, 27:35] = 8
    

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1
    

    return grid

def init_drop_off_zone_grid2():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    
    grid[4:10, 22:46] = 180 #180
    grid[10:22, 40:46] = 270 #270
    grid[15:23, 46:95] = 180 #180
    grid[4:16, 64:70] = 90 #90
    grid[39:47, 10:71] = 180 #180
    grid[22:40, 63:71] = 270 #270
    grid[3:15, 87:95] = 90 #90
    grid[23:47, 87:95] = 270 #270
    grid[4:47, 3:11] = 270 #270
    
    
    grid[4:23, 3:23] = 270 #270
    grid[4:10, 4:40] = 360 #360
    grid[3:17, 39:47] = 90 #90
    grid[15:23, 39:64] = 360 #360
    grid[4:23, 63:71] = 270 #270
    grid[3:11, 3:39] = 360 #360
    
    # Define drop off zone 2
    grid[4:13, 63:71] = 8
    

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid


def init_drop_off_zone_grid3():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[3:47, 3:11] = 270
    grid[3:10, 21:47] = 180
    grid[9:23, 39:47] = 270
    grid[15:23, 45:95] = 180
    grid[3:16, 63:71] = 90
    grid[39:47, 11:71] = 180
    grid[22:39, 63:71] = 270
    grid[3:16, 87:95] = 90
    grid[3:47, 87:95] = 270
    
    
    grid[4:23, 4:23] = 270
    grid[3:11, 3:40] = 360
    grid[3:16, 39:47] = 90
    grid[15:23, 39:87] = 360
    
    # Define drop off zone 3
    grid[3:11, 87:95] = 8

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid

def init_drop_off_zone_grid4():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[3:47, 3:11] = 270
    grid[3:10, 21:47] = 180
    grid[9:23, 39:47] = 270
    grid[15:23, 45:95] = 180
    grid[3:16, 63:71] = 90
    grid[39:47, 11:71] = 180
    grid[22:39, 63:71] = 270
    grid[3:16, 87:95] = 90
    grid[3:47, 87:95] = 90
    
    grid[4:23, 4:23] = 270
    grid[3:11, 3:40] = 360
    grid[3:16, 39:47] = 90
    grid[15:23, 39:87] = 360
   
    # Define drop off zone 4
    grid[39:47, 87:95] = 8
    

    # Set borders as obstacles (1)
    grid[0, 0:GRID_WIDTH] = 1
    grid[GRID_HEIGHT - 1, 0:GRID_WIDTH] = 1

    grid[0:GRID_HEIGHT, 0] = 1
    grid[0:GRID_HEIGHT, GRID_WIDTH - 1] = 1

    # Set obstacles to match maze
    grid[25:37, 13:25] = 1
    grid[13:25, 25:37] = 1
    grid[25:37, 37:61] = 1
    grid[1:13, 49:61] = 1
    grid[1:13, 73:85] = 1
    grid[25:49, 73:85] = 1

    return grid


def expand_grid(grid, factor=12):
    """Expands each cell in the 2D grid into a larger cell of `factor x factor`.
    
    Args:
        grid (np.array): The original 2D numpy array (grid).
        factor (int): The size to expand each cell into (default is 12).
    
    Returns:
        np.array: A new 2D numpy array where each cell from the original grid is expanded.
    """
    # Use numpy's repeat function to expand rows and columns
    expanded_grid = np.repeat(np.repeat(np.array(grid), factor, axis=0), factor, axis=1)
    return expanded_grid


def display_grid(reduced_grid):
    for i in range(len(reduced_grid)):
        for j in range(len(reduced_grid[0])):
            if reduced_grid[i][j] == 0:
                print(" ", end=" ")
            elif reduced_grid[i][j] == 1:
                print("-", end=" ")
            elif reduced_grid[i][j] == 2:
                print(" ", end=" ")
            elif reduced_grid[i][j] == 7:
                print("L", end=" ")
            elif reduced_grid[i][j] == 8:
                print("B", end=" ")
            elif reduced_grid[i][j] == 90:
                print("↓", end=" ")
            elif reduced_grid[i][j] == 180:
                print("←", end=" ")
            elif reduced_grid[i][j] == 270:
                print("↑", end=" ")
            elif reduced_grid[i][j] == 360:
                print("→", end=" ")
        print()
        
        
def create_valid_positions(grid):
    valid_positions = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 0:
                valid_positions.append([j, i])
    return valid_positions


def draw_grid(window, grid):
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid[y][x] == 1:
                color = BLACK
            elif grid[y][x] == 2:
                color = BLUE
            elif grid[y][x] != 0:
                color = GREEN
            else:
                color = WHITE
            pygame.draw.rect(window, color, (x * ppi, y * ppi, ppi, ppi))


def normal_pdf(x, mean, std_dev):
    """Calculate the PDF of a normal distribution at x."""
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = np.exp(-0.5 * ((x - mean) / std_dev) ** 2)
    return coefficient * exponent


def particle_variance(particles, weights, pred_x, pred_y):    
    variance_x = np.average([(p.x - pred_x)**2 / ppi**2 for p in particles], weights = weights)
    variance_y = np.average([(p.y - pred_y)**2 / ppi**2 for p in particles], weights = weights)
    
    # Average the variances to get a single spread measure
    return (variance_x + variance_y) / 2

def particle_angular_variance(particles, pred_theta):   
    weights = [p.weight for p in particles]
    variance_theta = np.average([(normalize_angle(p.theta) - pred_theta)**2 / ppi**2 for p in particles], weights = weights)
    
    # Average the variances to get a single spread measure
    return variance_theta


def estimate(particles):
    particles_x = [p.x for p in particles]
    particles_y = [p.y for p in particles]
    particles_theta = [p.theta for p in particles]
    weights = [p.weight for p in particles]
    mean_x = np.average(particles_x, weights = weights)
    mean_y = np.average(particles_y, weights = weights)
    mean_theta = np.average(particles_theta, weights = weights)
    
    return mean_x, mean_y, mean_theta


def normalize_angle(angle):
    return int(angle % 360)


def resample_particles(particles, grid, valid_positions, pred_x, pred_y):
    weights = [p.weight for p in particles]
    total_weight = sum(weights)    
    # Normalize weights
    weights = [w / total_weight for w in weights]
    variance = particle_variance(particles, weights, pred_x, pred_y)
    certainty = ppi**2 * 2 / (variance)
    print(f"Certainty: {certainty}")
    adjusted_num_particles = max(int(NUM_PARTICLES * max(1 - certainty, 0)), 1000)

    # resampling algorithm
    indexes = stratified_resample(weights)
        
    # resample from index
    particles = [particles[i] for i in indexes[:adjusted_num_particles]]
    
    # Jitter the particles' positions and orientations to keep them close to their original pose
    def jitter_particle(particle):
        # Add a small random noise to position (pose.x, pose.y) and orientation (pose.theta)
        noise_scale = max(1 - certainty, 0)
        new_x = particle.x + np.random.normal(0, 0.1 + 0.5 * noise_scale)  # Jitter in x-axis
        new_y = particle.y + np.random.normal(0, 0.1 + 0.5 * noise_scale)  # Jitter in y-axis
        new_theta = particle.theta + np.random.normal(0, 0.1)  # Jitter in orientation
        
        # Create a new particle with a similar pose
        new_particle = Particle(grid, valid_positions)
        if 0 <= int(new_x) < len(grid[0]) and 0 <= int(new_y) < len(grid) and grid[int(new_y)][int(new_x)] == 0:
            new_particle.x = new_x
            new_particle.y = new_y
        else:
            new_particle.x = particle.x
            new_particle.y = particle.y
        new_particle.theta = new_theta
        new_particle.weight = 1.0 / len(particles)  # Reset weights to be equal
        return new_particle
    
    # Apply jitter to resampled particles and also add some random jitter for new exploration
    jittered_particles = [jitter_particle(p) for p in particles]
    
    return jittered_particles, variance


def update_particles(particles, drive_type, drive_value, grid):
    """Move particle"""
    for particle in particles:
        particle.move(drive_type, drive_value, grid)


# Helper function to draw orientation lines
def draw_orientation(window, x, y, theta, length=10, color=BLACK):
    """Draw an orientation line based on the angle theta."""
    end_x = x + length * math.cos(theta)
    end_y = y + length * math.sin(theta)
    pygame.draw.line(window, color, (x, y), (end_x, end_y), 2)
    

# Particle class
class Particle:
    def __init__(self, grid, valid_positions):
        while True:
            pos = random.choice(valid_positions)
            self.x = pos[0]
            self.y = pos[1]
            if grid[self.y][self.x] == 0:
                break
        self.theta = random.uniform(0, 2 * math.pi)
        self.weight = 1.0 / NUM_PARTICLES

    def move(self, drive_type, drive_value, grid):
        # Add movement noise
        translation_noise = random.gauss(0, MOVEMENT_NOISE) * drive_value
        angular_noise = random.gauss(0, math.radians(0.02)) * drive_value

        new_x = 0
        new_y = 0
        new_theta = 0
        # Calculate new position
        if drive_type == 'w0':
            new_x = self.x + (drive_value + translation_noise) * math.cos(self.theta) * ppi
            new_y = self.y + (drive_value + translation_noise) * math.sin(self.theta) * ppi
            new_theta = self.theta + angular_noise
            # angular noise scaled relative to drive value
            
            # print(f"w0:{drive_value} resulted in {max(abs(new_y - self.y) / ppi, abs(new_x - self.x) / ppi)} inches") # TODO DEBUGGING
        elif drive_type == 'd0':
            new_x = self.x + (drive_value + translation_noise) * math.cos(self.theta + math.radians(90)) * ppi
            new_y = self.y + (drive_value + translation_noise) * math.sin(self.theta + math.radians(90)) * ppi
            new_theta = self.theta + angular_noise
            # print(f"d0:{drive_value} resulted in {max(abs(new_y - self.y) / ppi, abs(new_x - self.x) / ppi)} inches") # TODO DEBUGGING
        elif drive_type == 'r0':
            new_theta = self.theta + math.radians(drive_value) + angular_noise
        else:
            pass

        # Ensure particle stays within the drivable area
        if 0 <= int(new_x) < len(grid[0]) and 0 <= int(new_y) < len(grid) and grid[int(new_y)][int(new_x)] == 0:
            self.x = new_x
            self.y = new_y
        self.theta = new_theta
        
        
    def update_weight(self, lidar_distances, expected_distances):
        """Update particle weight based on the lidar readings."""
        # Calculate how closely the lidar readings match expected points        
        sensor_std = MIN_SENSOR_STD + (MAX_SENSOR_STD - MIN_SENSOR_STD) * max(1 - certainty, 0)
        for lidar, expected in zip(lidar_distances, expected_distances):
            self.weight *= normal_pdf(expected, lidar, sensor_std * ppi)
        self.weight += 1.e-300

    def lidar_scan(self, grid):
        """Simulate a lidar scan with a 180-degree beam width for the particle,
        using a 25-degree beam width (fan) for each scan angle."""
        # scan_angles = np.linspace(-90, 90, 7) * (math.pi / 180)  # Main scan angles
        scan_angles = np.linspace(0, 359, 360) * (math.pi / 180)  # Main scan angles
        beam_half_angle = 12.5 * (math.pi / 180)  # Half of 25 degrees in radians
        lidar_distances = []  # List to hold shortest lidar hit distance for each scan angle

        for angle in scan_angles:
            scan_angle = self.theta + angle
            sensor_x = self.x + 3.0 * ppi * math.cos(scan_angle)
            sensor_y =  self.y + 3.0 * ppi * math.sin(scan_angle)
            
            # Fan out with multiple scan lines within Â±12.5 degrees
            beam_angles = np.linspace(scan_angle - beam_half_angle, scan_angle + beam_half_angle, 10)

            # Track the shortest distance found within this beam
            min_distance = MAX_SENSOR_READING
            hit_found = False

            for beam_angle in beam_angles:
                for dist in np.linspace(0, (MAX_SENSOR_READING) * ppi, 500):  # High-resolution scan range
                    scan_x = dist * math.cos(beam_angle) + sensor_x
                    scan_y = dist * math.sin(beam_angle) + sensor_y

                    # Check for boundaries or obstacles
                    if 0 <= int(scan_x) < len(grid[0]) and 0 <= int(scan_y) < len(grid):
                        if grid[int(scan_y)][int(scan_x)] == 1:  # Hit an obstacle
                            # Return the exact pixel point for the lidar
                            boundary_x = int(scan_x)
                            boundary_y = int(scan_y)
                            lidar_distance = math.sqrt((boundary_x - sensor_x)**2 + (boundary_y - sensor_y)**2)

                            # Keep track of the closest hit distance within this beam
                            min_distance = min(min_distance, lidar_distance / float(ppi))
                            hit_found = True
                            break  # Stop scanning further in this direction
                # If no obstacle was found within the beam angle, move to the next one

            # Append the shortest distance found for this scan angle
            if hit_found:
                lidar_distances.append(max(min_distance, MIN_SENSOR_READING))
            else:
                # No obstacle found within the beam; use -1 to indicate max range
                lidar_distances.append(MAX_SENSOR_READING)

        return lidar_distances
    
    
    def lidar_scan_fast(self):
        """Simulate a lidar scan with a 180-degree beam width for the particle,
        using a 25-degree beam width (fan) for each scan angle."""
        # scan_angles = np.linspace(0, 359, 360) * (math.pi / 180)  # Main scan angles
        scan_angles = np.linspace(-90, 90, 7)  # Main scan angles
        # scan_angles = [0] # for only front sensor
        lidar_distances = []  # List to hold shortest lidar hit distance for each scan angle

        for angle in scan_angles:
            scan_angle = self.theta * (180 / math.pi) + angle
            lidar_distances.append(loaded_sensor_readings[int(self.x)][int(self.y)][normalize_angle(scan_angle)])
        
        return lidar_distances
    
    
#### SEBASTIAN CODE
def steps_to_degrees(steps):
    """
    Convert stepper motor steps to degrees of rotation.
    
    :param steps: int, number of steps motor has taken
    :return: float, degrees rotated
    """
    # Define the constant number of steps per full revolution
    steps_per_revolution = 800  # 200 steps * 4 microsteps
    degrees_per_step = 360 / steps_per_revolution
    degrees_rotated = steps * degrees_per_step
    return degrees_rotated

def steps_to_movement(m1_steps, m2_steps, m3_steps):
    """
    Calculate the movement or rotation of the rover based on the amount of steps each stepper motor has taken.

    CW (LOOKING AT WHEEL) IS POSITIVE

    The return format is a tuple of two strings:
    - The first string describes the type of movement ('Forward', 'Backward', 'Right', 'Left', 'Rotate CW', 'Rotate CCW', 'No Movement', 'Undefined Movement').
    - The second string provides the drive command (e.g., 'w:2.5' for forward movement of 2.5 inches).

    """
    
    # Fixed constants
    wheel_diameter = 2.28346  # in inches
    wheel_midline_distance = 2.965  # in inches
    # steps_per_revolution = 200*16  # Assuming microstepping
    # wheel_circumference = math.pi * wheel_diameter  # Circumference of each wheel in inches
    
    # Convert steps to degrees rotated
    m1_rotation_degrees = steps_to_degrees(m1_steps)
    m2_rotation_degrees = steps_to_degrees(m2_steps)
    m3_rotation_degrees = steps_to_degrees(m3_steps)
    
    drive_type = ''
    drive_value = 0.0

    # Determining the type of movement based on motor steps
    if m1_steps > 0 and m2_steps < 0 and m3_steps == 0:
        # Forward
        drive_value = (m1_rotation_degrees * wheel_diameter * math.pi) / (360 * math.cos(math.radians(30)))
        drive_type = "w0"
    elif m1_steps < 0 and m2_steps > 0 and m3_steps == 0:
        # Backward
        drive_value = (m1_rotation_degrees * wheel_diameter * math.pi) / (360 * math.cos(math.radians(30)))
        drive_type = "w0"
    elif m1_steps > 0 and m2_steps > 0 and m3_steps < 0:
        # Right
        drive_value = (m1_rotation_degrees * wheel_diameter * math.pi) / 180
        drive_type = "d0"
    elif m1_steps < 0 and m2_steps < 0 and m3_steps > 0:
        # Left
        drive_value = (m1_rotation_degrees * wheel_diameter * math.pi) / 180
        drive_type = "d0"
    elif m1_steps > 0 and m2_steps > 0 and m3_steps > 0:
        # Rotate CW
        drive_value = (m3_rotation_degrees * wheel_diameter) / (2 * wheel_midline_distance)
        drive_type = "r0"
    elif m1_steps < 0 and m2_steps < 0 and m3_steps < 0:
        # Rotate CCW
        drive_value = (m3_rotation_degrees * wheel_diameter) / (2 * wheel_midline_distance)
        drive_type = "r0"
    elif m1_steps == 0 and m2_steps == 0 and m3_steps == 0:
        drive_value = 0
        drive_type = "w0"
    else:
        drive_type = drive_command = "Undefined Movement"
        print("Undefined Movement")
        print(f"Motor Steps: {m1_steps}, {m2_steps}, {m3_steps}")

    return drive_type, drive_value

def shortest_rotation_distance(target_angle, current_angle):
    # Ensure the angles are within 0-359 degrees
    target_angle = target_angle % 360
    current_angle = current_angle % 360
    
    # Calculate the clockwise and counterclockwise distances
    cw_distance = (target_angle - current_angle) % 360  # CW direction
    ccw_distance = (current_angle - target_angle) % 360 # CCW direction
    
    # Return the shortest distance; negative if CCW is shorter
    if cw_distance <= ccw_distance:
        return cw_distance  # Positive for CW
    else:
        return -ccw_distance  # Negative for CCW
    
def particle_angular_variance(particles, pred_theta):
    shortest_distances = [abs(shortest_rotation_distance(normalize_angle(p.theta), normalize_angle(pred_theta))) for p in particles]
    variance_theta = np.average(shortest_distances)
    return variance_theta