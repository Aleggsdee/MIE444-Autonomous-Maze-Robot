import pygame
import random
import math
import numpy as np

# Initialize pygame
pygame.init()

# Constants
GRID_WIDTH = 82 
GRID_HEIGHT = 42 
CELL_SIZE = 10  # Adjust cell size for better visualization with the new dimensions
WINDOW_WIDTH = GRID_WIDTH * CELL_SIZE
WINDOW_HEIGHT = GRID_HEIGHT * CELL_SIZE
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

# Simulation parameters
NUM_PARTICLES = 500
FORWARD_VELOCITY = 4  # units per second
ANGULAR_VELOCITY = math.radians(120)  # 60 degrees per second

# Noise parameters
MOVEMENT_NOISE = 0.2
WEIGHT_NOISE = 0.5

# Create grid with obstacles only on edges and corners
grid = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype=int)  # Initialize all as free space (0)

# Set borders as obstacles (1)
for i in range(GRID_WIDTH):
    grid[0][i] = 1          # Top border
    grid[GRID_HEIGHT - 1][i] = 1  # Bottom border
for i in range(GRID_HEIGHT):
    grid[i][0] = 1          # Left border
    grid[i][GRID_WIDTH - 1] = 1  # Right border
    
# Set obstacles to match maze
for i in range(11, 21):
    for j in range(21, 31):
        grid[j][i] = 1
        
for i in range(21, 31):
    for j in range(11, 21):
        grid[j][i] = 1
        
for i in range(31, 51):
    for j in range(21, 31):
        grid[j][i] = 1
        
for i in range(41, 51):
    for j in range(1, 11):
        grid[j][i] = 1
        
for i in range(61, 71):
    for j in range(1, 11):
        grid[j][i] = 1
        
for i in range(61, 71):
    for j in range(21, 41):
        grid[j][i] = 1

# Initialize window
window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Rover Simulation with Edges as Obstacles")

# Function to draw the grid
def draw_grid(window, grid):
    for y in range(GRID_HEIGHT):
        for x in range(GRID_WIDTH):
            color = WHITE if grid[y][x] == 0 else BLACK
            pygame.draw.rect(window, color, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))
            
def absolute_to_relative(point, reference_position, reference_angle):
    """
    Convert absolute coordinates to relative coordinates based on the reference point's position and orientation.
    
    This version accounts for the Pygame coordinate system, where the origin is at the top left, 
    positive X goes right, and positive Y goes down.

    :param point: Tuple (absolute_x, absolute_y) of the lidar point.
    :param reference_position: Tuple (ref_x, ref_y) of the reference point (rover or particle).
    :param reference_angle: Orientation angle of the reference point in radians.
    :return: Tuple (relative_x, relative_y) of the lidar point relative to the reference point.
    """
    # Calculate relative position
    relative_x = point[0] - reference_position[0]  # Subtract reference x coordinate
    relative_y = point[1] - reference_position[1]  # Subtract reference y coordinate

    # Rotate CW according to Pygame axis by Rover angle theta - 90 degrees
    cos_angle = math.cos(reference_angle - math.pi / 2) 
    sin_angle = math.sin(reference_angle - math.pi / 2)

    # Rotate the coordinates to align with the reference point's orientation (taking Y-axis flip into account)
    rotated_x = relative_x * cos_angle + relative_y * sin_angle
    rotated_y = -relative_x * sin_angle + relative_y * cos_angle

    return (rotated_x, rotated_y)

# Rover class
class Rover:
    def __init__(self, grid):
        while True:
            self.x = random.randint(1, GRID_WIDTH - 2)  # Avoid placing on boundary
            self.y = random.randint(1, GRID_HEIGHT - 2)  # Avoid placing on boundary
            if grid[self.y][self.x] == 0:
                break
        self.theta = random.uniform(0, 2 * math.pi)
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
        if 0 <= int(new_x) < GRID_WIDTH  and 0 <= int(new_y) < GRID_HEIGHT  and grid[int(new_y)][int(new_x)] == 0:
            self.x = new_x
            self.y = new_y
        self.theta = new_theta

    def lidar_scan(self, grid):
        """Simulate a lidar scan with a 30-degree beam width."""
        scan_angles = np.linspace(-90, 90, 5) * (math.pi / 180)  # 30 degrees in total
        lidar_points = []  # List to hold lidar hit points
        hit_found = False  # Flag to indicate if an obstacle is hit

        for angle in scan_angles:
            scan_angle = self.theta + angle
            for dist in np.linspace(0, 2 * max(GRID_WIDTH, GRID_HEIGHT), 500):  # Scan range with higher resolution
                scan_x = self.x + dist * math.cos(scan_angle)
                scan_y = self.y + dist * math.sin(scan_angle)

                # Check for boundaries or obstacles
                if 0 <= int(scan_x) < GRID_WIDTH  and 0 <= int(scan_y) < GRID_HEIGHT:
                    if grid[int(scan_y)][int(scan_x)] == 1:  # Hit an obstacle
                        # Return the exact pixel point for the lidar
                        boundary_x = int(scan_x * CELL_SIZE)
                        boundary_y = int(scan_y * CELL_SIZE)
                        lidar_points.append((boundary_x, boundary_y))
                        hit_found = True
                        break  # Stop scanning further in this direction
            if not hit_found:
                # If no obstacle was found, return the maximum range point
                lidar_points.append((int(scan_x * CELL_SIZE), int(scan_y * CELL_SIZE)))

        return lidar_points

# Particle class
class Particle:
    def __init__(self, grid):
        while True:
            self.x = random.uniform(1, GRID_WIDTH - 2)
            self.y = random.uniform(1, GRID_HEIGHT - 2)
            if grid[int(self.y)][int(self.x)] == 0:
                break
        self.theta = random.uniform(0, 2 * math.pi)
        self.weight = 1.0

    def move(self, forward_velocity, angular_velocity, dt, grid):
        # Add movement noise
        forward_noise = random.gauss(0, MOVEMENT_NOISE)
        angular_noise = random.gauss(0, math.radians(2))

        # Calculate new position
        new_x = self.x + (forward_velocity + forward_noise) * math.cos(self.theta) * dt
        new_y = self.y + (forward_velocity + forward_noise) * math.sin(self.theta) * dt
        new_theta = self.theta + (angular_velocity + angular_noise) * dt

        # Ensure particle stays within the drivable area
        if 0 <= int(new_x) < GRID_WIDTH and 0 <= int(new_y) < GRID_HEIGHT and grid[int(new_y)][int(new_x)] == 0:
            self.x = new_x
            self.y = new_y
        self.theta = new_theta
        
    def update_weight(self, lidar_points, expected_points):
        """Update particle weight based on the lidar readings."""
        if not lidar_points or not expected_points:
            self.weight = 0
            return
        
        # Calculate how closely the lidar readings match expected points
        self.weight = sum([1.0 / (1 + np.linalg.norm(np.array(lidar) - np.array(expected)) ** 2) for lidar, expected in zip(lidar_points, expected_points)]) / len(expected_points)
              

    def lidar_scan(self, grid):
        """Simulate a lidar scan with a 30-degree beam width for the particle."""
        scan_angles = np.linspace(-90, 90, 5) * (math.pi / 180)  # 30 degrees in total
        lidar_points = []  # List to hold lidar hit points
        hit_found = False  # Flag to indicate if an obstacle is hit

        for angle in scan_angles:
            scan_angle = self.theta + angle
            for dist in np.linspace(0, 2 * max(GRID_WIDTH, GRID_HEIGHT), 500):  # Scan range with higher resolution
                scan_x = self.x + dist * math.cos(scan_angle)
                scan_y = self.y + dist * math.sin(scan_angle)

                # Check for boundaries or obstacles
                if 0 <= int(scan_x) < GRID_WIDTH and 0 <= int(scan_y) < GRID_HEIGHT:
                    if grid[int(scan_y)][int(scan_x)] == 1:  # Hit an obstacle
                        # Return the exact pixel point for the lidar
                        boundary_x = int(scan_x * CELL_SIZE)
                        boundary_y = int(scan_y * CELL_SIZE)
                        lidar_points.append((boundary_x, boundary_y))
                        hit_found = True
                        break  # Stop scanning further in this direction
            if not hit_found:
                # If no obstacle was found, return the maximum range point
                lidar_points.append((int(scan_x * CELL_SIZE), int(scan_y * CELL_SIZE)))

        return lidar_points

# Initialize rover and particles
rover = Rover(grid)
particles = [Particle(grid) for _ in range(NUM_PARTICLES)]

def resample_particles(particles, grid, NUM_PARTICLES):
    weights = [p.weight for p in particles]
    total_weight = sum(weights)
    # if weights:
    #     certainty = max(weights)
    # else:
    #     certainty = 0
    certainty = 0 # don't use adaptive monte carlo because convergence just takes too long or never happens
    # print(certainty)
    
    if total_weight == 0:
        # If all weights are zero, generate new random particles
        return [Particle(grid) for _ in range(NUM_PARTICLES)]
    
    # Normalize weights
    weights = [w / total_weight for w in weights]
    
    
    
    # Adjust particle count based on certainty
    adjusted_num_particles = int(NUM_PARTICLES * (1 - certainty))  # Fewer particles as certainty increases
    
    # Resample particles based on normalized weights
    new_particles = random.choices(particles, weights, k=adjusted_num_particles)
    
    # Jitter the particles' positions and orientations to keep them close to their original pose
    def jitter_particle(particle):
        # Add a small random noise to position (pose.x, pose.y) and orientation (pose.theta)
        noise_scale = 1 - certainty
        new_x = particle.x + np.random.normal(0, 0.5 * noise_scale)  # Jitter in x-axis
        new_y = particle.y + np.random.normal(0, 0.5 * noise_scale)  # Jitter in y-axis
        new_theta = particle.theta + np.random.normal(0, 0.1 * noise_scale)  # Jitter in orientation
        
        # Create a new particle with a similar pose
        new_particle = Particle(grid)
        new_particle.x = new_x
        new_particle.y = new_y
        new_particle.theta = new_theta
        new_particle.weight = particle.weight  # Keep the same weight initially
        
        return new_particle
    
    # Apply jitter to resampled particles and also add some random jitter for new exploration
    jittered_particles = [
        # inject some randomness and avoid the filter getting stuck in local minima, where all particles may converge to an incorrect position.
        Particle(grid) if random.random() < 0.01 else jitter_particle(p) for p in new_particles
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

# Main loop
running = True
clock = pygame.time.Clock()

while running:
    window.fill(WHITE)
    draw_grid(window, grid)

    # User input for rover control
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        rover.vel_forward = FORWARD_VELOCITY
    elif keys[pygame.K_s]:
        rover.vel_forward = -FORWARD_VELOCITY
    else:
        rover.vel_forward = 0

    if keys[pygame.K_a]:
        rover.vel_angular = -ANGULAR_VELOCITY
    elif keys[pygame.K_d]:
        rover.vel_angular = ANGULAR_VELOCITY
    else:
        rover.vel_angular = 0

    # Move rover and particles
    dt = clock.get_time() / 1000  # Time in seconds
    rover.move(dt, grid)
    update_particles(particles, rover, dt)  # Pass dt to particles

    # Draw particles with orientation lines
    for particle in particles:
        particle_pos = (int(particle.x * CELL_SIZE), int(particle.y * CELL_SIZE))
        pygame.draw.circle(window, RED, particle_pos, 2)
        draw_orientation(window, particle_pos[0], particle_pos[1], particle.theta, length=10, color=RED)

        # # Lidar scan for particles and draw points
        # particle_lidar_points = particle.lidar_scan(grid)
        # for point in particle_lidar_points:
        #     pygame.draw.circle(window, GREEN, point, 3)  # Draw green dots for particle lidar readings

    # Draw rover as a square and its orientation
    rover_pos = (int(rover.x * CELL_SIZE), int(rover.y * CELL_SIZE))
    rover_rect = pygame.Rect(rover_pos[0] - 10, rover_pos[1] - 10, 20, 20)  # Rover as a larger square
    pygame.draw.rect(window, BLACK, rover_rect)  # Draw rover in black
    draw_orientation(window, rover_pos[0], rover_pos[1], rover.theta, length=15, color=GREEN)

    # Lidar scan for the rover and draw points
    lidar_points = rover.lidar_scan(grid)
    for point in lidar_points:
        pygame.draw.circle(window, GREEN, point, 3)  # Smaller green dots
    
    rover_relative_lidar_points = [absolute_to_relative(lidar_point, rover_pos, rover.theta) for lidar_point in lidar_points]
    

    # Update particle weights and resample
    for particle in particles:
        particle_lidar_points = particle.lidar_scan(grid)
        particle_pos = (int(particle.x * CELL_SIZE), int(particle.y * CELL_SIZE))
        particle_relative_lidar_points = [absolute_to_relative(lidar_point, particle_pos, particle.theta) for lidar_point in particle_lidar_points]
        particle.update_weight(particle_relative_lidar_points, rover_relative_lidar_points)
    
    particles = resample_particles(particles, grid, NUM_PARTICLES)

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.display.flip()
    clock.tick(30)  # 30 FPS

pygame.quit()
