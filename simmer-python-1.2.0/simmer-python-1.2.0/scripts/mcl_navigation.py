import pygame
import random
import math
import numpy as np
import copy
from filterpy.monte_carlo import stratified_resample
import time
import pickle
import mcl_helper as mh
from mcl_helper import Particle

import socket
import time
from datetime import datetime
import serial

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
NUM_PARTICLES = 5000

# Noise parameters
MOVEMENT_NOISE = 0.2
WEIGHT_NOISE = 0.5
    
    
grid = mh.init_grid()
reduced_grid = copy.deepcopy(grid)
grid = mh.expand_grid(grid, ppi)
valid_positions = mh.create_valid_positions(grid)


# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.'''

    start_time = time.time()
    response_raw = ''
    while time.time() < start_time + TIMEOUT_SERIAL:
        if SER.in_waiting:
            response_char = SER.read().decode('ascii')
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    print(f'Raw response was: {response_raw}')

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
def depacketize(data_raw: str):
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid


############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 115200         # Baudrate in bps
PORT_SERIAL = 'COM5'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True



############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.3
else:
    TRANSMIT_PAUSE = 0

######################################### OBSTACLE AVOIDANCE IN CASE WE CAN'T LOCALIZE ###########################
def obstacle_avoidance(duration):

    LOOP_PAUSE_TIME = 0.05 # seconds

    # Main loop
    RUN_DEAD_RECKONING = True # If true, run this. If false, skip it

    # Function to check all ToF sensors (will be in mm!)

    BIG_FRONT_THRESHOLD = 7.5
    SMALL_FRONT_THRESHOLD = 4
    FRONT_THRESHOLD = SMALL_FRONT_THRESHOLD
    SIDE_THRESHOLD = 2.75 # distance_t0, distance_t6
    ANGLED_FRONT_THRESHOLD = 2.5 # distance_t2, distance_t4
    ANGLED_SIDE_THRESHOLD = 2.75 # distance_t1, distance_t5


    start_time = time.time()
    transmit(packetize('xx'))
    [responses, time_rx] = receive()
    while RUN_DEAD_RECKONING:
        # Pause for a little while so as to not spam commands insanely fast
        time.sleep(LOOP_PAUSE_TIME)
        
        current_time = time.time()
        if current_time - start_time > duration:
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            # update particles
            RUN_DEAD_RECKONING = False
            break # replace with return
        
        
        transmit(packetize('t0,t1,t2,t3,t4,t5,t6,m1,m2,m3'))
        [responses, time_rx] = receive()
        lidar_distances = [float(response[1]) for response in responses[:7]]
        
        distance_t0 = lidar_distances[0]
        distance_t1 = lidar_distances[1]
        distance_t2 = lidar_distances[2]
        distance_t3 = lidar_distances[3]
        distance_t4 = lidar_distances[4]
        distance_t5 = lidar_distances[5]
        distance_t6 = lidar_distances[6]
        
        
        if distance_t0 > SIDE_THRESHOLD and distance_t1 > ANGLED_SIDE_THRESHOLD and distance_t2 > ANGLED_FRONT_THRESHOLD and distance_t3 > FRONT_THRESHOLD and distance_t4 > ANGLED_FRONT_THRESHOLD and distance_t5 > ANGLED_SIDE_THRESHOLD and distance_t6 > SIDE_THRESHOLD:
            print('moving forward')
            print('FRONT_THRESHOLD = ' + f'{FRONT_THRESHOLD}')
            if FRONT_THRESHOLD == BIG_FRONT_THRESHOLD:
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                # update particles
                FRONT_THRESHOLD = SMALL_FRONT_THRESHOLD
            transmit(packetize('w0:50'))
            [responses, time_rx] = receive()
        else:
            if (distance_t3 < 2 or (distance_t4 < ANGLED_FRONT_THRESHOLD and distance_t2 < ANGLED_FRONT_THRESHOLD)) and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD:
                print('backing up')
                print(FRONT_THRESHOLD)
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                transmit(packetize('w0:-0.5'))
                [responses, time_rx] = receive()
                # update particles
            elif (distance_t6 < SIDE_THRESHOLD or distance_t5 < ANGLED_SIDE_THRESHOLD or distance_t4 < ANGLED_FRONT_THRESHOLD) and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD:
                print('moving left')
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                transmit(packetize('d0:-0.5'))
                [responses, time_rx] = receive()
                # update particles
                transmit(packetize('r0:-15'))
                [responses, time_rx] = receive()
                # update particles
            elif (distance_t0 < SIDE_THRESHOLD or distance_t1 < ANGLED_SIDE_THRESHOLD or distance_t2 < ANGLED_FRONT_THRESHOLD) and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD:
                print('moving right')
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                transmit(packetize('d0:0.5'))
                [responses, time_rx] = receive()
                # update particles
                transmit(packetize('r0:15'))
                [responses, time_rx] = receive()
                # update particles
            elif distance_t3 < FRONT_THRESHOLD and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD:
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                # update particles
                if distance_t4 > distance_t0:
                    print('rotating CW')
                    transmit(packetize('r0:90'))
                    [responses, time_rx] = receive()
                else:
                    print('rotating CCW')
                    transmit(packetize('r0:-90'))
                    [responses, time_rx] = receive()     
                # update particles 
                FRONT_THRESHOLD = BIG_FRONT_THRESHOLD
                print('FRONT_THRESHOLD = ' + f'{FRONT_THRESHOLD}')
            else:
                transmit(packetize('r0:30'))
                [responses, time_rx] = receive()  




# Initialize window and particles
particles = [Particle(grid, valid_positions) for _ in range(NUM_PARTICLES)]
window = pygame.display.set_mode((len(grid[0]), len(grid)))
pygame.display.set_caption("Rover Simulation with Edges as Obstacles")
window.fill(WHITE)
mh.draw_grid(window, reduced_grid)

# Draw particles with orientation lines
for particle in particles:
    particle_pos = (int(particle.x), int(particle.y))
    pygame.draw.circle(window, RED, particle_pos, ppi / 2)
    mh.draw_orientation(window, particle_pos[0], particle_pos[1], particle.theta, length=ppi * 2, color=RED)

# Main loop
running = True

# Localization variables
pred_x = 0
pred_y = 0
pred_theta = 0
is_localized = False
rotation_degrees = 0

# Navigation variables
arrived_at_loading_zone = False
navigation_matrix = mh.init_loading_zone_grid()
angular_velocity = 60 # deg/s

# Obstacle avoidance variables
FRONT_THRESHOLD = 2 # distance_t3
SIDE_THRESHOLD = 2 # distance_t0, distance_t6
ANGLED_FRONT_THRESHOLD = 2 # distance_t2, distance_t4
ANGLED_SIDE_THRESHOLD = 2 # distance_t1, distance_t5

clock = pygame.time.Clock()

while running:
    window.fill(WHITE)
    mh.draw_grid(window, reduced_grid)
    
    ################# LOCALIZATION ########################
    if not is_localized:
        if rotation_degrees <= 460:
            transmit(packetize('r0:10'))
            [responses, time_rx] = receive()
            time.sleep(2)
            # can add a time.sleep() here if necessary
        else:
            # run obstacle for a few seconds, then rotate to localize again
            obstacle_avoidance(25)
            particles = [Particle(grid, valid_positions) for _ in range(NUM_PARTICLES)]
            rotation_degrees = 0
        
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,m1,m2,m3'))
    [responses, time_rx] = receive()
    lidar_distances = [float(response[1]) for response in responses[:7]]
    motor_steps = [int(response[1]) for response in responses[7:]]
    drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
    print(f"Drive Value: {drive_value}")
    mh.update_particles(particles, drive_type, drive_value, grid)
    rotation_degrees += drive_value

    # Draw particles with orientation lines
    for particle in particles:
        particle_pos = (int(particle.x), int(particle.y))
        pygame.draw.circle(window, RED, particle_pos, ppi / 2)
        mh.draw_orientation(window, particle_pos[0], particle_pos[1], particle.theta, length=ppi * 2, color=RED)

    # Update particle weights and resample
    particle_lidar_distances = []
    for particle in particles:
        particle_lidar_distances = particle.lidar_scan_fast()
        particle.update_weight(particle_lidar_distances, lidar_distances)
    
    if drive_value != 0 or True:
        particles, variance = mh.resample_particles(particles, grid, valid_positions, pred_x, pred_y)
        pred_x, pred_y, pred_theta = mh.estimate(particles)
        pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)
        angular_variance = mh.particle_angular_variance(particles, pred_theta)
        print(f"Particle Position: {pred_x / ppi - 1}, {pred_y / ppi - 1}")
        print(f"Particle Angle: {pred_theta}")
        print(f"Angular Variance: {angular_variance}")
        if not is_localized and variance < 1:
            is_localized = True
            print("LOCALIZED!")
            # transmit(packetize('l'))
            # [responses, time_rx] = receive()
            
            navigation_dir = mh.normalize_angle(navigation_matrix[int(pred_y / ppi - 1)][int(pred_x / ppi - 1)]) # to set 360 to 0
            print(f"Navigation Direction: {navigation_dir}")
            if navigation_dir == 0 or navigation_dir == 90 or navigation_dir == 180 or navigation_dir == 270:
                angle_difference = mh.shortest_rotation_distance(navigation_dir, pred_theta)
                transmit(packetize(f'r0:{angle_difference}'))
                [responses, time_rx] = receive()
                time.sleep(abs(angle_difference) / angular_velocity + 0.25)
        # elif not is_localized and angular_variance < 10:
        #     navigation_dir = mh.normalize_angle(navigation_matrix[int(particles[0].y / ppi - 1)][int(particles[0].x / ppi - 1)]) # to set 360 to 0
        #     print(f"Navigation Direction: {navigation_dir}")
        #     angle_difference = mh.shortest_rotation_distance(navigation_dir, pred_theta)
        #     transmit(packetize(f'r0:{angle_difference}'))
        #     [responses, time_rx] = receive()
        #     time.sleep(abs(angle_difference) / angular_velocity + 0.25)
            
        #     # update particle positions
        #     transmit(packetize('m1,m2,m3'))
        #     [responses, time_rx] = receive()
        #     motor_steps = [int(response[1]) for response in responses]
        #     drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
        #     mh.update_particles(particles, drive_type, drive_value, grid)
            
        #     transmit(packetize('w0:12'))
        #     [responses, time_rx] = receive()
        #     time.sleep(5)
            
        #     # update particle positions
        #     transmit(packetize('m1,m2,m3'))
        #     [responses, time_rx] = receive()
        #     motor_steps = [int(response[1]) for response in responses]
        #     drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
        #     mh.update_particles(particles, drive_type, drive_value, grid)
            
        #     rotation_degrees = 0
            
            

    ################## NAVIGATION + OBSTACLE AVOIDANCE ############################
    if is_localized:
        navigation_dir = mh.normalize_angle(navigation_matrix[int(pred_y / ppi - 1)][int(pred_x / ppi - 1)]) # to set 360 to 0
        print(f"Navigation Direction: {navigation_dir}")
        
        # Check if arrived
        if navigation_dir == 7 or navigation_dir == 8: # set loading zone grids to 7 and drop off zone grids to 8
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            if lidar_distances[3] > FRONT_THRESHOLD + 5:
                transmit(packetize('w0:4'))
                [responses, time_rx] = receive()
                time.sleep(1)
                
            if not arrived_at_loading_zone:
                arrived_at_loading_zone = True
                print("arrived at loading zone")
                time.sleep(2)
                navigation_matrix = mh.init_drop_off_zone_grid1() ## TODO MAKE SURE YOU CHANGE THIS FOR CORRECT DROP OFF!!!!!
            else:
                print("arrived at drop off zone")
                running = False
                break
        
        # Obstacle avoidance/Navigation
        distance_t0 = lidar_distances[0]
        distance_t1 = lidar_distances[1]
        distance_t2 = lidar_distances[2]
        distance_t3 = lidar_distances[3]
        distance_t4 = lidar_distances[4]
        distance_t5 = lidar_distances[5]
        distance_t6 = lidar_distances[6]
        
        
        # TODO: add updates to particle positions every time xx is called or new drive command type is called
        # Navigation (forward drive only happens here!)       
        if distance_t0 > SIDE_THRESHOLD and distance_t1 > ANGLED_SIDE_THRESHOLD and distance_t2 > ANGLED_FRONT_THRESHOLD and distance_t3 > FRONT_THRESHOLD and distance_t4 > ANGLED_FRONT_THRESHOLD and distance_t5 > ANGLED_SIDE_THRESHOLD and distance_t6 > SIDE_THRESHOLD:
            angle_difference = mh.shortest_rotation_distance(navigation_dir, pred_theta)
            if abs(angle_difference) <= 15:  
                transmit(packetize('w0:50'))
                [responses, time_rx] = receive()
            else:
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                if distance_t3 > FRONT_THRESHOLD + 2:
                    transmit(packetize('w0:2'))
                    [responses, time_rx] = receive()
                    time.sleep(1)
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                
                transmit(packetize(f'r0:{angle_difference}'))
                [responses, time_rx] = receive()
                time.sleep(abs(angle_difference) / angular_velocity + 0.25)
        else:
            if distance_t3 < FRONT_THRESHOLD or (distance_t2 < ANGLED_FRONT_THRESHOLD and distance_t4 < ANGLED_FRONT_THRESHOLD):
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                
                transmit(packetize('w0:-0.5'))
                [responses, time_rx] = receive()
            elif distance_t6 < SIDE_THRESHOLD or distance_t5 < ANGLED_SIDE_THRESHOLD or distance_t4 < ANGLED_FRONT_THRESHOLD:
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                    
                transmit(packetize('d0:-0.25'))
                [responses, time_rx] = receive()
                # time.sleep(0.25) # make sure drive command finishes
                
                # hardcode particle position update
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                mh.update_particles(particles, "d0", -0.25, grid)
                
                transmit(packetize('r0:-2.5'))
                [responses, time_rx] = receive()
                # time.sleep(0.25) # make sure drive command finishes
                # update hardcode
            elif distance_t0 < SIDE_THRESHOLD or distance_t1 < ANGLED_SIDE_THRESHOLD or distance_t2 < ANGLED_FRONT_THRESHOLD:
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                
                transmit(packetize('d0:0.25'))
                [responses, time_rx] = receive()
                time.sleep(0.25) # make sure drive command finishes
                
                # hardcode particle position update
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                mh.update_particles(particles, "d0", 0.25, grid)
                
                transmit(packetize('r0:2.5'))
                [responses, time_rx] = receive()
                # time.sleep(0.25) # make sure drive command finishes
            else: # this should never happen in theory
                print("womp womp")

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # print("hello")
    pygame.display.flip()
    clock.tick(60)  # 60 FPS
    
pygame.quit()