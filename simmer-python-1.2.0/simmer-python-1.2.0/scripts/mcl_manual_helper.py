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

# Simulation parameters
NUM_PARTICLES = 5000
FORWARD_VELOCITY = 4 * ppi  # units per second
ANGULAR_VELOCITY = math.radians(120)  # 60 degrees per second

# Noise parameters
MOVEMENT_NOISE = 0.1


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
    
    # Set free movable space
    grid[4:46, 4:10] = 270
    grid[4:10, 22:46] = 180
    grid[28:40, 28:34] = 90
    grid[10:22, 40:46] = 270
    grid[16:22, 46:94] = 180
    grid[4:16, 64:70] = 90
    grid[40:46, 10:70] = 180
    grid[22:40, 64:70] = 270
    grid[4:16, 88:94] = 90
    grid[22:46, 88:94] = 270

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
    
    # Set Loading Zone
    grid[4:22, 4:22] = 7

    return grid

# navigation to bottom left drop off zone
def init_drop_off_zone_grid1():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[4:46, 4:10] = 270
    grid[4:10, 22:46] = 180
    grid[10:22, 40:46] = 270
    grid[16:22, 46:94] = 180
    grid[4:16, 64:70] = 90
    grid[40:46, 10:70] = 180
    grid[22:40, 64:70] = 270
    grid[4:16, 88:94] = 90
    grid[22:46, 88:94] = 270
    
    
    grid[4:22, 4:22] = 180
    grid[4:40, 4:10] = 90
    grid[40:46, 4:28] = 360
    grid[28:46, 28:34] = 270
    

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

# navigation to top left drop off zone
def init_drop_off_zone_grid2():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[4:46, 4:10] = 270
    grid[4:10, 22:46] = 180
    grid[10:22, 40:46] = 270
    grid[16:22, 46:94] = 180
    grid[4:16, 64:70] = 90
    grid[40:46, 10:70] = 180
    grid[22:40, 64:70] = 270
    grid[4:16, 88:94] = 90
    grid[22:46, 88:94] = 270
    
    
    grid[4:22, 4:22] = 270
    grid[4:10, 4:40] = 360
    grid[4:16, 40:46] = 90
    grid[16:22, 40:64] = 360
    grid[4:22, 64:70] = 270
    

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


# navigation to top right drop off zone
def init_drop_off_zone_grid3():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[4:46, 4:10] = 270
    grid[4:10, 22:46] = 180
    grid[10:22, 40:46] = 270
    grid[16:22, 46:94] = 180
    grid[4:16, 64:70] = 90
    grid[40:46, 10:70] = 180
    grid[22:40, 64:70] = 270
    grid[4:16, 88:94] = 90
    grid[4:46, 88:94] = 270
    
    
    grid[4:22, 4:22] = 270
    grid[4:10, 4:40] = 360
    grid[4:16, 40:46] = 90
    grid[16:22, 40:88] = 360
    

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

# navigation to bottom right drop off zone
def init_drop_off_zone_grid4():
    # Set unmovable space that is not an obstacle
    grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)
    
    # Set free movable space   
    grid[4:46, 4:10] = 270
    grid[4:10, 22:46] = 180
    grid[10:22, 40:46] = 270
    grid[16:22, 46:94] = 180
    grid[4:16, 64:70] = 90
    grid[40:46, 10:70] = 180
    grid[22:40, 64:70] = 270
    grid[4:16, 88:94] = 90
    grid[4:46, 88:94] = 90
    
    
    grid[4:22, 4:22] = 270
    grid[4:10, 4:40] = 360
    grid[4:16, 40:46] = 90
    grid[16:22, 40:88] = 360
    

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
    # Calculate variance in x and y directions
    # variance_x = np.var([p.x for p in particles])
    # variance_y = np.var([p.y for p in particles])
    
    variance_x = np.average([(p.x - pred_x)**2 / ppi**2 for p in particles], weights = weights)
    variance_y = np.average([(p.y - pred_y)**2 / ppi**2 for p in particles], weights = weights)
    
    # Average the variances to get a single spread measure
    return (variance_x + variance_y) / 2


def estimate(particles):
    particles_x = [p.x for p in particles]
    particles_y = [p.y for p in particles]
    particles_theta = [p.theta for p in particles]
    weights = [p.weight for p in particles]
    print(len(particles_x))
    print(len(weights))
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
    print(variance)
    certainty = ppi**2 * 10 / (variance)
    adjusted_num_particles = max(int(NUM_PARTICLES * max(1 - certainty, 0)), 300)

    # resampling algorithm
    indexes = stratified_resample(weights)
        
    # resample from index
    particles = [particles[i] for i in indexes[:adjusted_num_particles]]
    # print(len(particles))
    
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
        elif drive_type == 'd0':
            new_x = self.x + (drive_value + translation_noise) * math.cos(self.theta + math.radians(90)) * ppi
            new_y = self.y + (drive_value + translation_noise) * math.sin(self.theta + math.radians(90)) * ppi
            new_theta = self.theta + angular_noise
        elif drive_type == 'r0':
            new_theta = self.theta + math.radians(drive_value) + angular_noise
        else:
            print("not a drive command")

        # Ensure particle stays within the drivable area
        if 0 <= int(new_x) < len(grid[0]) and 0 <= int(new_y) < len(grid) and grid[int(new_y)][int(new_x)] == 0:
            self.x = new_x
            self.y = new_y
        self.theta = new_theta
        
    def update_weight(self, lidar_distances, expected_distances):
        """Update particle weight based on the lidar readings."""
        # Calculate how closely the lidar readings match expected points        
        for lidar, expected in zip(lidar_distances, expected_distances):
            self.weight *= normal_pdf(expected, lidar, 2.5 * ppi)
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