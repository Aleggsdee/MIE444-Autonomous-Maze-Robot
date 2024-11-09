import pygame
import random
import math
import numpy as np
import copy
from filterpy.monte_carlo import stratified_resample
import time
import pickle

# Initialize pygame
pygame.init()

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
NUM_PARTICLES = 1
FORWARD_VELOCITY = 4 * ppi  # units per second
ANGULAR_VELOCITY = math.radians(120)  # 60 degrees per second

# Noise parameters
MOVEMENT_NOISE = 0.2
WEIGHT_NOISE = 0.5

# Create grid with obstacles only on edges and corners
grid = np.full((GRID_HEIGHT, GRID_WIDTH), 2, dtype=int)

# Set free movable space
for x in range(4, 10):
    for y in range(4, 46):
        grid[y][x] = 0
        
for x in range(10, 22):
    for y in range(4, 22):
        grid[y][x] = 0
        
for x in range(22, 40):
    for y in range(4, 10):
        grid[y][x] = 0
        
for x in range(28, 34):
    for y in range(28, 40):
        grid[y][x] = 0
        
for x in range(40, 46):
    for y in range(4, 22):
        grid[y][x] = 0
        
for x in range(46, 64):
    for y in range(16, 22):
        grid[y][x] = 0
        
for x in range(64, 70):
    for y in range(4, 46):
        grid[y][x] = 0
        
for x in range(10, 64):
    for y in range(40, 46):
        grid[y][x] = 0
        
for x in range(64, 88):
    for y in range(16, 22):
        grid[y][x] = 0
        
for x in range(88, 94):
    for y in range(4, 46):
        grid[y][x] = 0

# Set borders as obstacles (1)
for i in range(GRID_WIDTH):
    grid[0][i] = 1          # Top border
    grid[GRID_HEIGHT - 1][i] = 1  # Bottom border
for i in range(GRID_HEIGHT):
    grid[i][0] = 1          # Left border
    grid[i][GRID_WIDTH - 1] = 1  # Right border
    
# Set obstacles to match maze
for x in range(13, 25):
    for y in range(25, 37):
        grid[y][x] = 1
        
for x in range(25, 37):
    for y in range(13, 25):
        grid[y][x] = 1
        
for x in range(37, 61):
    for y in range(25, 37):
        grid[y][x] = 1
        
for x in range(49, 61):
    for y in range(1, 13):
        grid[y][x] = 1
        
for x in range(73, 85):
    for y in range(1, 13):
        grid[y][x] = 1
        
for x in range(73, 85):
    for y in range(25, 49):
        grid[y][x] = 1
        
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

reduced_grid = copy.deepcopy(grid)
grid = expand_grid(grid, ppi)

valid_positions = []
for i in range(len(reduced_grid)):
    for j in range(len(reduced_grid[0])):
        if reduced_grid[i][j] == 0:
            print(reduced_grid[i][j], end=" ")
            # valid_positions.append([j, i])
        elif reduced_grid[i][j] == 1:
            print("-", end=" ")
        else:
            print("*", end=" ")
    print()


for i in range(len(grid)):
    for j in range(len(grid[0])):
        if grid[i][j] == 0:
            valid_positions.append([j, i])
            
print(random.choice(valid_positions))

# Function to draw the grid
def draw_grid(window, grid):
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            if grid[y][x] == 1:
                color = BLACK
            elif grid[y][x] == 2:
                color = RED
            elif grid[y][x] != 0:
                color = GREEN
            else:
                color = WHITE
            pygame.draw.rect(window, color, (x * ppi, y * ppi, ppi, ppi))

def neff(weights):
    return 1. / np.sum(np.square(weights))

def normal_pdf(x, mean, std_dev):
    """Calculate the PDF of a normal distribution at x."""
    coefficient = 1 / (std_dev * np.sqrt(2 * np.pi))
    exponent = np.exp(-0.5 * ((x - mean) / std_dev) ** 2)
    return coefficient * exponent

def particle_variance(particles):
    # Calculate variance in x and y directions
    variance_x = np.var([p.x for p in particles])
    variance_y = np.var([p.y for p in particles])
    
    # Average the variances to get a single spread measure
    return (variance_x + variance_y) / 2

def check_convergence(particles, threshold):
    spread = particle_variance(particles)
    return spread < threshold

# Rover class
class Rover:
    def __init__(self, grid):
        while True:
            pos = random.choice(valid_positions)
            # self.x = pos[0]
            self.x = 4 * ppi
            # self.y = pos[1]
            self.y = 4 * ppi
            if grid[self.y][self.x] == 0:
                break
        # self.theta = random.uniform(0, 2 * math.pi)
        # self.theta = (0 - 3 * math.pi / 2)
        self.theta = math.pi /4 # positive rotation is clockwise
        self.vel_forward = 0
        self.vel_angular = 0

    def move(self, dt, grid):
        # Apply noise to movement
        forward_noise = random.gauss(0, MOVEMENT_NOISE)
        angular_noise = random.gauss(0, math.radians(2))

        # Calculate proposed new position
        new_x = self.x + (self.vel_forward + forward_noise) * math.cos(self.theta) * dt
        new_y = self.y + (self.vel_forward + forward_noise) * math.sin(self.theta) * dt
        new_theta = self.theta + (self.vel_angular + angular_noise) * dt

        # Check if new position is within drivable area (not in a black/1 area)
        if 0 <= int(new_x) < len(grid[0])  and 0 <= int(new_y) < len(grid)  and grid[int(new_y)][int(new_x)] == 0:
            self.x = new_x
            self.y = new_y
        self.theta = new_theta

    # # STRAIGHT LINES ONLY (beam angle of 0)
    # def lidar_scan(self, grid):
    #     """Simulate a lidar scan with a 180-degree beam width for the particle."""
    #     scan_angles = np.linspace(-90, 90, 7) * (math.pi / 180)
    #     lidar_distances = []  # List to hold lidar hit distances
    #     hit_found = False  # Flag to indicate if an obstacle is hit

    #     for angle in scan_angles:
    #         scan_angle = self.theta + angle
    #         sensor_x = self.x + 3.0 * ppi * math.cos(scan_angle)
    #         sensor_y =  self.y + 3.0 * ppi * math.sin(scan_angle)
    #         for dist in np.linspace(0, MAX_SENSOR_READING * ppi, 500):  # Scan range with higher resolution
    #             scan_x = dist * math.cos(scan_angle) + sensor_x
    #             scan_y = dist * math.sin(scan_angle) + sensor_y

    #             # Check for boundaries or obstacles
    #             if 0 <= int(scan_x) < len(grid[0]) and 0 <= int(scan_y) < len(grid):
    #                 if grid[int(scan_y)][int(scan_x)] == 1:  # Hit an obstacle
    #                     # Return the exact pixel point for the lidar
    #                     boundary_x = int(scan_x)
    #                     boundary_y = int(scan_y)
    #                     # print(f"Boundary X: {boundary_x}")
    #                     # print(f"Boundary Y: {boundary_y}")
    #                     # print(f"Self X: {self.x}")
    #                     # print(f"Self Y: {self.y}")
    #                     lidar_distance = math.sqrt((boundary_x - sensor_x)**2 + (boundary_y - sensor_y)**2)
    #                     # lidar_distances.append(max(lidar_distance / ppi - 3, MIN_SENSOR_READING))
    #                     lidar_distances.append(lidar_distance / float(ppi))
    #                     hit_found = True
    #                     break  # Stop scanning further in this direction
    #         if not hit_found:
    #             # If no obstacle was found, return the maximum range point
    #             lidar_distances.append(MAX_SENSOR_READING)
    #     return lidar_distances
    
    # TODO
    def lidar_scan(self, grid):
        """Simulate a lidar scan with a 180-degree beam width for the particle,
        using a 25-degree beam width (fan) for each scan angle."""
        num_scan_angles = 360
        scan_angles = np.linspace(0, 359, num_scan_angles) * (math.pi / 180)  # Main scan angles
        # scan_angles = np.linspace(-90, 90, num_scan_angles) * (math.pi / 180)  # Main scan angles
        beam_half_angle = 7.5 * (math.pi / 180)  # Half of 15 degrees in radians
        # lidar_distances = []  # List to hold shortest lidar hit distance for each scan angle
        lidar_distances = np.zeros(num_scan_angles)

        for index, angle in enumerate(scan_angles):
            scan_angle = self.theta + angle
            sensor_x = self.x + 3.0 * ppi * math.cos(scan_angle)
            sensor_y =  self.y + 3.0 * ppi * math.sin(scan_angle)
            
            # Fan out with multiple scan lines within ±12.5 degrees
            num_beams = 10
            beam_angles = np.linspace(scan_angle - beam_half_angle, scan_angle + beam_half_angle, num_beams)

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
                # lidar_distances.append(max(min_distance, MIN_SENSOR_READING))
                lidar_distances[index] = max(min_distance, MIN_SENSOR_READING)
                # lidar_distances[index] = min_distance
            else:
                # No obstacle found within the beam
                lidar_distances[index] = MAX_SENSOR_READING
                # lidar_distances.append(MAX_SENSOR_READING)

        return lidar_distances
    
    # TODO
    def lidar_scan2(self, grid):
        """Simulate a lidar scan with a 180-degree beam width for the particle,
        using a 25-degree beam width (fan) for each scan angle."""
        # scan_angles = np.linspace(0, 359, 360) * (math.pi / 180)  # Main scan angles
        scan_angles = np.linspace(-90, 90, 7) * (math.pi / 180)  # Main scan angles
        # scan_angles = [0] # for only front sensor
        beam_half_angle = 7.5 * (math.pi / 180)  # Half of 15 degrees in radians
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
                # lidar_distances.append(min_distance)
            else:
                # No obstacle found within the beam; use -1 to indicate max range
                lidar_distances.append(MAX_SENSOR_READING)

        return lidar_distances
    

# Particle class
class Particle:
    def __init__(self, grid):
        while True:
            pos = random.choice(valid_positions)
            self.x = pos[0]
            self.y = pos[1]
            if grid[self.y][self.x] == 0:
                break
        self.theta = random.uniform(0, 2 * math.pi)
        # self.theta = -math.pi / 2
        # self.theta = random.choice([0, math.pi / 2, math.pi, 3 * math.pi / 2])
        self.weight = 1.0 / NUM_PARTICLES

    def move(self, forward_velocity, angular_velocity, dt, grid):
        # Add movement noise
        forward_noise = random.gauss(0, MOVEMENT_NOISE)
        angular_noise = random.gauss(0, math.radians(2))

        # Calculate new position
        new_x = self.x + (forward_velocity + forward_noise) * math.cos(self.theta) * dt
        new_y = self.y + (forward_velocity + forward_noise) * math.sin(self.theta) * dt
        new_theta = self.theta + (angular_velocity + angular_noise) * dt

        # Ensure particle stays within the drivable area
        if 0 <= int(new_x) < len(grid[0])  and 0 <= int(new_y) < len(grid)  and grid[int(new_y)][int(new_x)] == 0:
            self.x = new_x
            self.y = new_y
        self.theta = new_theta
        
    def update_weight(self, lidar_distances, expected_distances):
        """Update particle weight based on the lidar readings."""
        # Calculate how closely the lidar readings match expected points
        # self.weight = sum([1.0 / (1 + abs(lidar - expected)) for lidar, expected in zip(lidar_distances, expected_distances)])
        
        for lidar, expected in zip(lidar_distances, expected_distances):
            self.weight *= normal_pdf(expected, lidar, 25)
        self.weight += 1.e-300

    # TODO
    def lidar_scan(self, grid):
        """Simulate a lidar scan with a 180-degree beam width for the particle,
        using a 25-degree beam width (fan) for each scan angle."""
        # scan_angles = np.linspace(-90, 90, 7) * (math.pi / 180)  # Main scan angles
        scan_angles = np.linspace(0, 359, 360) * (math.pi / 180)  # Main scan angles
        # scan_angles = [0] # for only front sensor
        beam_half_angle = 7.5 * (math.pi / 180)  # Half of 15 degrees in radians
        lidar_distances = []  # List to hold shortest lidar hit distance for each scan angle

        for angle in scan_angles:
            scan_angle = self.theta + angle
            sensor_x = self.x + 3.0 * ppi * math.cos(scan_angle)
            sensor_y =  self.y + 3.0 * ppi * math.sin(scan_angle)
            
            # Fan out with multiple scan lines within ±12.5 degrees
            beam_angles = np.linspace(scan_angle - beam_half_angle, scan_angle + beam_half_angle, 7)

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
    
# Initialize rover and particles
rover = Rover(grid)
particles = [Particle(grid) for _ in range(NUM_PARTICLES)]

def resample_particles(particles, grid, NUM_PARTICLES):
    weights = [p.weight for p in particles]
    total_weight = sum(weights)    
    # Normalize weights
    weights = [w / total_weight for w in weights]
    variance = particle_variance(particles)
    # print(variance)
    certainty = 1.0 / (variance)
    adjusted_num_particles = max(int(NUM_PARTICLES * max(1 - certainty, 0)), 100)

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
        new_particle = Particle(grid)
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
    jittered_particles = [
        jitter_particle(p) for p in particles
    ]
    
    return jittered_particles


def update_particles(particles, rover, dt):
    """Move and update particle weights."""
    for particle in particles:
        particle.move(rover.vel_forward, rover.vel_angular, dt, grid)

# Helper function to draw orientation lines
def draw_orientation(window, x, y, theta, length=10, color=BLACK):
    """Draw an orientation line based on the angle theta."""
    end_x = x + length * math.cos(theta)
    end_y = y + length * math.sin(theta)
    pygame.draw.line(window, color, (x, y), (end_x, end_y), 2)

# Initialize window
print(len(grid[0]))
print(len(grid))
window = pygame.display.set_mode((len(grid[0]), len(grid)))
pygame.display.set_caption("Rover Simulation with Edges as Obstacles")

# # Main loop
# running = True
# clock = pygame.time.Clock()

# start_time = time.time()
# count = 0
# while running:
#     if count == 100:
#         break
    
#     window.fill(WHITE)
#     draw_grid(window, reduced_grid)

#     # User input for rover control
#     keys = pygame.key.get_pressed()
#     if keys[pygame.K_w]:
#         rover.vel_forward = FORWARD_VELOCITY
#     elif keys[pygame.K_s]:
#         rover.vel_forward = -FORWARD_VELOCITY
#     else:
#         rover.vel_forward = 0

#     if keys[pygame.K_a]:
#         rover.vel_angular = -ANGULAR_VELOCITY
#     elif keys[pygame.K_d]:
#         rover.vel_angular = ANGULAR_VELOCITY
#     else:
#         rover.vel_angular = 0

#     # Move rover and particles
#     dt = clock.get_time() / 1000  # Time in seconds
#     rover.move(dt, grid)
#     update_particles(particles, rover, dt)  # Pass dt to particles

#     # Draw particles with orientation lines
#     for particle in particles:
#         particle_pos = (int(particle.x), int(particle.y))
#         pygame.draw.circle(window, RED, particle_pos, 2)
#         draw_orientation(window, particle_pos[0], particle_pos[1], particle.theta, length=10, color=RED)

#     # Draw rover as a square and its orientation
#     rover_pos = (int(rover.x), int(rover.y))
#     rover_rect = pygame.Rect(rover_pos[0] - ppi, rover_pos[1] - ppi, 2 * ppi, 2 * ppi)  # Rover as a larger square
#     pygame.draw.rect(window, BLACK, rover_rect)  # Draw rover in black
#     draw_orientation(window, rover_pos[0], rover_pos[1], rover.theta, length=ppi, color=GREEN)

#     # Lidar scan for the rover and draw points
#     lidar_distances = rover.lidar_scan(grid)
#     lidar_distances2 = rover.lidar_scan2(grid)
#     # print(lidar_distances2)
#     particle_lidar_distances = []
#     # print(f"rover x: {rover.x}")
#     # print(f"rover y: {rover.y}")

#     # Update particle weights and resample
#     # for particle in particles:
#     #     particle_lidar_distances = particle.lidar_scan(grid)
#         # print(particle_lidar_distances)
#     #     particle_pos = (int(particle.x), int(particle.y))
#     #     particle.update_weight(particle_lidar_distances, lidar_distances)
    
#     # if rover.vel_forward != 0 or rover.vel_angular != 0:
#     #     particles = resample_particles(particles, grid, NUM_PARTICLES)
#         # sum_abs_diff = sum(abs(a - b) for a, b in zip(particle_lidar_distances, lidar_distances))
#         # print(f"Sum of distance differences: {sum_abs_diff}")
#         # print(f"Rover Distance Readings: {lidar_distances}")
#         # print(f"Particle Distance Readings: {particle_lidar_distances}")
        

#     # Event handling
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#     # print("hello")
#     pygame.display.flip()
#     clock.tick(60)  # 60 FPS
#     # count += 1
# end_time = time.time()
# print(end_time - start_time)
# print(len(valid_positions))

# Loop for gathering data
clock = pygame.time.Clock()

start_time = time.time()
all_sensor_readings = np.zeros((len(grid[0]), len(grid), 360)) # x, y, angle
count = 0
for x, y in valid_positions:
    
    window.fill(WHITE)
    draw_grid(window, reduced_grid)
    rover.x = x
    rover.y = y
    rover.theta = 0    

    # Draw rover as a square and its orientation
    rover_pos = (int(rover.x), int(rover.y))
    rover_rect = pygame.Rect(rover_pos[0] - ppi, rover_pos[1] - ppi, 2 * ppi, 2 * ppi)  # Rover as a larger square
    pygame.draw.rect(window, BLACK, rover_rect)  # Draw rover in black
    draw_orientation(window, rover_pos[0], rover_pos[1], rover.theta, length=ppi, color=GREEN)

    # Lidar scan for the rover and draw points
    all_sensor_readings[x][y] = rover.lidar_scan(grid)

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(60)  # 60 FPS
    count += 1
    if count % 100 == 0:
        print(f"{count} / {len(valid_positions)}")
end_time = time.time()
print(f"Time Taken (s): {end_time - start_time}")

with open('sensor_data_3_ppi_15_beam_angle.pkl', 'wb') as f:
    pickle.dump(all_sensor_readings, f)

with open('sensor_data_3_ppi_15_beam_angle.pkl', 'rb') as f:
    loaded_sensor_readings = pickle.load(f)
    
print(len(loaded_sensor_readings))

pygame.quit()