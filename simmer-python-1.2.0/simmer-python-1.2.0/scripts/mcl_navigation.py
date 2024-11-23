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
SIMULATE = False



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
    
    
################### BLOCK DETECTION #####################
def find_and_move_to_block_bottom():
    """
    Function to scan for a block using sensors t3 (top) and t7 (bottom),
    rotate back to the position with the maximum difference, and move towards the block
    in two stages: first to 4 inches away, then re-scan and adjust, and finally move to less than 1 inch away.
    """

    block_located = False # Flag variable to indicate whether block was found during search routine
    COARSE_SWEEP_ANGLE = 100 # degrees
    COARSE_SWEEP_INCREMENT = 5 # degrees
    MAX_BOTTOM_SENSOR_READING = 14 # 19 inches
    TOP_BOTTOM_SENSOR_DISTANCE = 1.55 # inches
    MIN_DELTA = 2 + TOP_BOTTOM_SENSOR_DISTANCE # minimum difference in top and bottom sensor readings to be considered significant for block detection
    FINE_SWEEP_ANGLE = 90 # degrees
    FINE_SWEEP_INCREMENT = 3 # degrees
    MAX_T3_VALUE = 24 # inches

    angular_velocity = 90.0 # rad/s (actually ~100 rad/s)
    forward_velocity = 4.0 # in/s (actually ~4.67 in/s)
    
    # Loop to repeat Step 1 to Step 3 until block is located (loop will either run once, or twice)
    
    # Step 1: Turn 90 degrees with the command r0:90
    print("Turning 90 degrees.")
    transmit(packetize('r0:90'))
    [responses, time_rx] = receive()
    time.sleep(90 / angular_velocity)
    
    while not block_located:
        # Step 2: Perform a coarse sweep scan in increments, storing sensor differences
        print(f"Starting initial {COARSE_SWEEP_ANGLE}-degree scan in {COARSE_SWEEP_INCREMENT}-degree increments.")
        angle_differences = []  # List to store (angle, difference) tuples
        for i in range(int(COARSE_SWEEP_ANGLE / COARSE_SWEEP_INCREMENT)):  # TODO this might be too fast/not giving sensor data enough time to update
            # Turn 5 degrees
            transmit(packetize(f'r0:-{COARSE_SWEEP_INCREMENT}'))
            [responses, time_rx] = receive()
            time.sleep(6 * COARSE_SWEEP_INCREMENT / angular_velocity + 0.25)

            # Read sensors t3 (top) and t7 (bottom)
            transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
            [responses, time_rx] = receive()
            ToF_distances = [float(response[1]) for response in responses[:8]]
            sensor_t3_reading = ToF_distances[3]
            sensor_t7_reading = ToF_distances[7]

            # Store the angle and the difference
            if sensor_t3_reading > MAX_T3_VALUE or sensor_t7_reading > MAX_BOTTOM_SENSOR_READING:
                angle_differences.append((COARSE_SWEEP_INCREMENT * (i + 1), 0))
                print(f"Sensor difference at angle {COARSE_SWEEP_INCREMENT * (i + 1)} degrees: 0 inches")
            else:
                # Calculate the difference
                difference = abs(sensor_t7_reading - sensor_t3_reading)
                print(f"Sensor difference at angle {COARSE_SWEEP_INCREMENT * (i + 1)} degrees: {difference} inches")
                angle_differences.append((COARSE_SWEEP_INCREMENT * (i + 1), difference))
                

        # Step 3: Find the angle with the maximum difference
        
        # CHANGE: Find average angle for which the delta is above 1.5"
        valid_angles = [angle for angle, diff in angle_differences if diff > MIN_DELTA]

        if not valid_angles:
            print(f"No valid sensor differences found above {MIN_DELTA} inches. Block may not be in the vicinity.")
            print("Moving to new location and attempting search routine again.")
            
            # Rotate 90, go forward 3 inches, rotate 45, then go 12 inches to new position 
            transmit(packetize('w0:3')) # Clears the near corner
            [responses, time_rx] = receive()
            time.sleep(3 / forward_velocity + 0.25)
            
            transmit(packetize('r0:45')) # Points in general direction of unsearched zone
            [responses, time_rx] = receive()
            time.sleep(45 / angular_velocity + 0.25)
            
            transmit(packetize('w0:6')) # Moves forward to get block within sensor range
            [responses, time_rx] = receive()
            time.sleep(8 / forward_velocity + 0.5)
            
            COARSE_SWEEP_ANGLE = 220
            MAX_T3_VALUE = 18
            MAX_BOTTOM_SENSOR_READING = 14
            
            transmit(packetize(f'r0:{COARSE_SWEEP_ANGLE / 2}')) # Points in general direction of unsearched zone
            [responses, time_rx] = receive()
            time.sleep(45 / angular_velocity + 0.25)
        else:
            # Calculate the average angle
            avg_angle = sum(valid_angles) / len(valid_angles)
            print(f"Average angle with sensor difference above {MIN_DELTA} inches: {avg_angle} degrees.")
            
            block_located = True # Flag variable to indicate whether block was found during search routine
        

    # Step 4: Rotate back to the angle with the maximum difference
    # Calculate the angle to rotate back (since we've already turned 180 degrees)
    rotation_angle = (COARSE_SWEEP_ANGLE - avg_angle)
    print(f"Rotating back {rotation_angle} degrees to face the block.")
    transmit(packetize(f'r0:{rotation_angle}'))
    [responses, time_rx] = receive()
    time.sleep(rotation_angle / angular_velocity + 0.25)

    # Step 5: Move forward closer to block for fine sweep search (Stage 1)
    print("Stage 1: Moving closer towards block.")
    
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    ToF_distances = [float(response[1]) for response in responses[:8]]
    sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)
    print(f"Moving Forward: {sensor_t7_reading / 3} inches")
    
    transmit(packetize(f'w0:{sensor_t7_reading / 2}'))
    [responses, time_rx] = receive()
    
    time.sleep(abs(sensor_t7_reading / 3) / forward_velocity)
    
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    ToF_distances = [float(response[1]) for response in responses[:8]]
    sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)
    
    if sensor_t7_reading < 3:
        transmit(packetize(f'w0:-2'))
        [responses, time_rx] = receive()
        time.sleep(2 / forward_velocity)

    # Step 6: Re-scan and adjust alignment (repeat Steps 2 to 4)
    print("Starting adjustment scan to refine alignment.")

    # Optional: Turn 30 degrees before starting the adjustment scan if needed
    print(f"Turning {FINE_SWEEP_ANGLE / 2} degrees for adjustment scan.")
    transmit(packetize(f'r0:{FINE_SWEEP_ANGLE / 2}'))
    [responses, time_rx] = receive()
    time.sleep((FINE_SWEEP_ANGLE / 2) / angular_velocity)

    # Perform fine sweep scan
    angle_differences = []
    for i in range(int(FINE_SWEEP_ANGLE / FINE_SWEEP_INCREMENT)):
        # Turn 1 degree
        transmit(packetize(f'r0:-{FINE_SWEEP_INCREMENT}'))
        [responses, time_rx] = receive()
        time.sleep(6 * FINE_SWEEP_INCREMENT / angular_velocity + 0.1)

        # Read sensors t3 and t7
        transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
        [responses, time_rx] = receive()
        ToF_distances = [float(response[1]) for response in responses[:8]]
        sensor_t3_reading = min(ToF_distances[3], MAX_BOTTOM_SENSOR_READING)
        sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)

        if sensor_t3_reading is None or sensor_t7_reading is None:
            print(f"Failed to read sensors at adjustment angle {FINE_SWEEP_INCREMENT * (i + 1)} degrees. Storing difference as 0.")
            difference = 0
        else:
            # Calculate the difference
            difference = abs(sensor_t7_reading - sensor_t3_reading)
            print(f"Sensor difference at adjustment angle {FINE_SWEEP_INCREMENT * (i + 1)} degrees: {difference} inches")

        # Store the angle and the difference
        if sensor_t7_reading > 12:
            angle_differences.append((FINE_SWEEP_INCREMENT * (i + 1), 0))
        else:
            angle_differences.append((FINE_SWEEP_INCREMENT * (i + 1), difference))

    valid_angles = [angle for angle, diff in angle_differences if diff > MIN_DELTA]

    if not valid_angles:
        print(f"No valid sensor differences found above {MIN_DELTA} inches. Block may not be in the vicinity.")
        return
    else:
        # Calculate the average angle
        avg_angle = sum(valid_angles) / len(valid_angles)
        print(f"Average angle with sensor difference above {MIN_DELTA} inches: {avg_angle} degrees.")
        

    # Step 6: Rotate back to the angle with the maximum difference
    # Calculate the angle to rotate back (since we've already turned 180 degrees)
    rotation_angle = (FINE_SWEEP_ANGLE - avg_angle)
    print(f"Rotating back {rotation_angle} degrees to refine alignment.")
    transmit(packetize(f'r0:{rotation_angle}'))
    [responses, time_rx] = receive()
    time.sleep(rotation_angle / angular_velocity + 0.25)
    
    # Step 7: Move forward until the bottom sensor reads less than 1 inch (Stage 2)
    print("Stage 2: Drive to the block.")
    
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    ToF_distances = [float(response[1]) for response in responses[:8]]
    sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)
    
    transmit(packetize(f'w0:-2'))
    [responses, time_rx] = receive()
    time.sleep(2 / forward_velocity + 0.25)
    transmit(packetize('o'))
    [responses, time_rx] = receive()
    time.sleep(1)
    
    print(f"Moving Forward: {sensor_t7_reading + 2 + 0.5} inches")
    
    ################# LOWER THE GRIPPER HERE!!!!! ########################
    
    transmit(packetize(f'w0:{sensor_t7_reading + 2 + 0.5}'))
    [responses, time_rx] = receive()
    time.sleep(abs(sensor_t7_reading + 2 + 0.5) / forward_velocity) # 4 inches/sec speed 
    transmit(packetize('c'))
    [responses, time_rx] = receive()
    
    
def find_and_move_to_block_top():
    """
    Function to scan for a block using sensors t3 (top) and t7 (bottom),
    rotate back to the position with the maximum difference, and move towards the block
    in two stages: first to 4 inches away, then re-scan and adjust, and finally move to less than 1 inch away.
    """

    block_located = False # Flag variable to indicate whether block was found during search routine
    COARSE_SWEEP_ANGLE = 100 # degrees
    COARSE_SWEEP_INCREMENT = 5 # degrees
    MAX_BOTTOM_SENSOR_READING = 14 # 19 inches
    TOP_BOTTOM_SENSOR_DISTANCE = 1.55 # inches
    MIN_DELTA = 2 + TOP_BOTTOM_SENSOR_DISTANCE # minimum difference in top and bottom sensor readings to be considered significant for block detection
    FINE_SWEEP_ANGLE = 90 # degrees
    FINE_SWEEP_INCREMENT = 3 # degrees
    MAX_T3_VALUE = 24 # inches

    angular_velocity = 90.0 # rad/s (actually ~100 rad/s)
    forward_velocity = 4.0 # in/s (actually ~4.67 in/s)
    
    # Loop to repeat Step 1 to Step 3 until block is located (loop will either run once, or twice)
    
    # Step 1: Turn 90 degrees with the command r0:90
    print("Turning 90 degrees.")
    transmit(packetize('r0:-90'))
    [responses, time_rx] = receive()
    time.sleep(90 / angular_velocity)
    
    while not block_located:
        # Step 2: Perform a coarse sweep scan in increments, storing sensor differences
        print(f"Starting initial {COARSE_SWEEP_ANGLE}-degree scan in {COARSE_SWEEP_INCREMENT}-degree increments.")
        angle_differences = []  # List to store (angle, difference) tuples
        for i in range(int(COARSE_SWEEP_ANGLE / COARSE_SWEEP_INCREMENT)):  # TODO this might be too fast/not giving sensor data enough time to update
            # Turn 5 degrees
            transmit(packetize(f'r0:{COARSE_SWEEP_INCREMENT}'))
            [responses, time_rx] = receive()
            time.sleep(6 * COARSE_SWEEP_INCREMENT / angular_velocity + 0.25)

            # Read sensors t3 (top) and t7 (bottom)
            transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
            [responses, time_rx] = receive()
            ToF_distances = [float(response[1]) for response in responses[:8]]
            sensor_t3_reading = ToF_distances[3]
            sensor_t7_reading = ToF_distances[7]

            # Store the angle and the difference
            if sensor_t3_reading > MAX_T3_VALUE or sensor_t7_reading > MAX_BOTTOM_SENSOR_READING:
                angle_differences.append((COARSE_SWEEP_INCREMENT * (i + 1), 0))
                print(f"Sensor difference at angle {COARSE_SWEEP_INCREMENT * (i + 1)} degrees: 0 inches")
            else:
                # Calculate the difference
                difference = abs(sensor_t7_reading - sensor_t3_reading)
                print(f"Sensor difference at angle {COARSE_SWEEP_INCREMENT * (i + 1)} degrees: {difference} inches")
                angle_differences.append((COARSE_SWEEP_INCREMENT * (i + 1), difference))
                

        # Step 3: Find the angle with the maximum difference
        
        # CHANGE: Find average angle for which the delta is above 1.5"
        valid_angles = [angle for angle, diff in angle_differences if diff > MIN_DELTA]

        if not valid_angles:
            print(f"No valid sensor differences found above {MIN_DELTA} inches. Block may not be in the vicinity.")
            print("Moving to new location and attempting search routine again.")
            
            # Rotate 90, go forward 3 inches, rotate 45, then go 12 inches to new position 
            transmit(packetize('w0:3')) # Clears the near corner
            [responses, time_rx] = receive()
            time.sleep(3 / forward_velocity + 0.25)
            
            transmit(packetize('r0:-45')) # Points in general direction of unsearched zone
            [responses, time_rx] = receive()
            time.sleep(45 / angular_velocity + 0.25)
            
            transmit(packetize('w0:6')) # Moves forward to get block within sensor range
            [responses, time_rx] = receive()
            time.sleep(8 / forward_velocity + 0.5)
            
            COARSE_SWEEP_ANGLE = 220
            MAX_T3_VALUE = 18
            MAX_BOTTOM_SENSOR_READING = 14
            
            transmit(packetize(f'r0:-{COARSE_SWEEP_ANGLE / 2}')) # Points in general direction of unsearched zone
            [responses, time_rx] = receive()
            time.sleep(45 / angular_velocity + 0.25)
        else:
            # Calculate the average angle
            avg_angle = sum(valid_angles) / len(valid_angles)
            print(f"Average angle with sensor difference above {MIN_DELTA} inches: {avg_angle} degrees.")
            
            block_located = True # Flag variable to indicate whether block was found during search routine
        

    # Step 4: Rotate back to the angle with the maximum difference
    # Calculate the angle to rotate back (since we've already turned 180 degrees)
    rotation_angle = (COARSE_SWEEP_ANGLE - avg_angle)
    print(f"Rotating back {rotation_angle} degrees to face the block.")
    transmit(packetize(f'r0:-{rotation_angle}'))
    [responses, time_rx] = receive()
    time.sleep(rotation_angle / angular_velocity + 0.25)

    # Step 5: Move forward closer to block for fine sweep search (Stage 1)
    print("Stage 1: Moving closer towards block.")
    
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    ToF_distances = [float(response[1]) for response in responses[:8]]
    sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)
    print(f"Moving Forward: {sensor_t7_reading / 3} inches")
    
    transmit(packetize(f'w0:{sensor_t7_reading / 2}'))
    [responses, time_rx] = receive()
    
    time.sleep(abs(sensor_t7_reading / 3) / forward_velocity)
    
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    ToF_distances = [float(response[1]) for response in responses[:8]]
    sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)
    
    if sensor_t7_reading < 3:
        transmit(packetize(f'w0:-2'))
        [responses, time_rx] = receive()
        time.sleep(2 / forward_velocity)

    # Step 6: Re-scan and adjust alignment (repeat Steps 2 to 4)
    print("Starting adjustment scan to refine alignment.")

    # Optional: Turn 30 degrees before starting the adjustment scan if needed
    print(f"Turning {FINE_SWEEP_ANGLE / 2} degrees for adjustment scan.")
    transmit(packetize(f'r0:-{FINE_SWEEP_ANGLE / 2}'))
    [responses, time_rx] = receive()
    time.sleep((FINE_SWEEP_ANGLE / 2) / angular_velocity)

    # Perform fine sweep scan
    angle_differences = []
    for i in range(int(FINE_SWEEP_ANGLE / FINE_SWEEP_INCREMENT)):
        # Turn 1 degree
        transmit(packetize(f'r0:{FINE_SWEEP_INCREMENT}'))
        [responses, time_rx] = receive()
        time.sleep(6 * FINE_SWEEP_INCREMENT / angular_velocity + 0.1)

        # Read sensors t3 and t7
        transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
        [responses, time_rx] = receive()
        ToF_distances = [float(response[1]) for response in responses[:8]]
        sensor_t3_reading = min(ToF_distances[3], MAX_BOTTOM_SENSOR_READING)
        sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)

        if sensor_t3_reading is None or sensor_t7_reading is None:
            print(f"Failed to read sensors at adjustment angle {FINE_SWEEP_INCREMENT * (i + 1)} degrees. Storing difference as 0.")
            difference = 0
        else:
            # Calculate the difference
            difference = abs(sensor_t7_reading - sensor_t3_reading)
            print(f"Sensor difference at adjustment angle {FINE_SWEEP_INCREMENT * (i + 1)} degrees: {difference} inches")

        # Store the angle and the difference
        if sensor_t7_reading > 12:
            angle_differences.append((FINE_SWEEP_INCREMENT * (i + 1), 0))
        else:
            angle_differences.append((FINE_SWEEP_INCREMENT * (i + 1), difference))

    valid_angles = [angle for angle, diff in angle_differences if diff > MIN_DELTA]

    if not valid_angles:
        print(f"No valid sensor differences found above {MIN_DELTA} inches. Block may not be in the vicinity.")
        return
    else:
        # Calculate the average angle
        avg_angle = sum(valid_angles) / len(valid_angles)
        print(f"Average angle with sensor difference above {MIN_DELTA} inches: {avg_angle} degrees.")
        

    # Step 6: Rotate back to the angle with the maximum difference
    # Calculate the angle to rotate back (since we've already turned 180 degrees)
    rotation_angle = (FINE_SWEEP_ANGLE - avg_angle)
    print(f"Rotating back {rotation_angle} degrees to refine alignment.")
    transmit(packetize(f'r0:-{rotation_angle - 7.5}'))
    [responses, time_rx] = receive()
    time.sleep(rotation_angle / angular_velocity + 0.25)
    
    # Step 7: Move forward until the bottom sensor reads less than 1 inch (Stage 2)
    print("Stage 2: Drive to the block.")
    
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    ToF_distances = [float(response[1]) for response in responses[:8]]
    sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)
    
    transmit(packetize(f'w0:-2'))
    [responses, time_rx] = receive()
    time.sleep(2 / forward_velocity + 0.25)
    transmit(packetize('o'))
    [responses, time_rx] = receive()
    time.sleep(1)
    
    print(f"Moving Forward: {sensor_t7_reading + 2 + 0.5} inches")
    
    ################# LOWER THE GRIPPER HERE!!!!! ########################
    
    transmit(packetize(f'w0:{sensor_t7_reading + 2 + 0.5}'))
    [responses, time_rx] = receive()
    time.sleep(abs(sensor_t7_reading + 2 + 0.5) / forward_velocity) # 4 inches/sec speed 
    transmit(packetize('c'))
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
angular_velocity = 90.0 # actually ~100 deg/s but we estimate slower just to be safe
forward_velocity = 4.0 # actually 4.67 in/s but we estimate slower just to be safe

# Obstacle avoidance variables
FRONT_THRESHOLD = 3.5 # distance_t3
SIDE_THRESHOLD = 2 # distance_t0, distance_t6
ANGLED_FRONT_THRESHOLD = 3.5 # distance_t2, distance_t4
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
            time.sleep(1) # used to be 1
            # can add a time.sleep() here if necessary
        else:
            # run obstacle for a few seconds, then rotate to localize again
            particles = [Particle(grid, valid_positions) for _ in range(NUM_PARTICLES)]
            rotation_degrees = 0
        
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    lidar_distances = [float(response[1]) for response in responses[:8]]
    motor_steps = [int(response[1]) for response in responses[8:]]
    drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
    print(f"{drive_type}:{drive_value}")
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
    
    if motor_steps[0] != 0 or motor_steps[1] != 0 or motor_steps[2] != 0:
        particles, variance = mh.resample_particles(particles, grid, valid_positions, pred_x, pred_y)
        pred_x, pred_y, pred_theta = mh.estimate(particles)
        pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)
        print(f"Particle Position: {pred_x / ppi - 1}, {pred_y / ppi - 1}")
        print(f"Particle Angle: {pred_theta}")
        print(f"Particle Position Variance: {variance}")
        if not is_localized and variance < 3: # usually 3
            is_localized = True
            print("LOCALIZED!")
            
            navigation_dir = mh.normalize_angle(navigation_matrix[int(pred_y / ppi - 1)][int(pred_x / ppi - 1)]) # to set 360 to 0
            print(f"Navigation Direction: {navigation_dir}")
            if navigation_dir == 0 or navigation_dir == 90 or navigation_dir == 180 or navigation_dir == 270:
                angle_difference = mh.shortest_rotation_distance(navigation_dir, pred_theta)
                transmit(packetize(f'r0:{angle_difference}'))
                [responses, time_rx] = receive()
                time.sleep(abs(angle_difference / angular_velocity) + 0.25)
                continue
                
            
            

    ################## NAVIGATION + OBSTACLE AVOIDANCE ############################
    if is_localized:
        navigation_dir = mh.normalize_angle(navigation_matrix[int(pred_y / ppi - 1)][int(pred_x / ppi - 1)]) # to set 360 to 0
        print(f"Navigation Direction: {navigation_dir}")
        
        # Check if arrived
        if navigation_dir == 7 or navigation_dir == 8: # set loading zone grids to 7 and drop off zone grids to 8
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            time.sleep(0.25)
            
            # update particle positions
            transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
            [responses, time_rx] = receive()
            lidar_distances = [float(response[1]) for response in responses[:8]]
            motor_steps = [int(response[1]) for response in responses[8:]]
            drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
            mh.update_particles(particles, drive_type, drive_value, grid)    
            pred_x, pred_y, pred_theta = mh.estimate(particles)
            pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)
                
            if not arrived_at_loading_zone:
                arrived_at_loading_zone = True
                print("arrived at loading zone")
                
                ##### BLOCK DETECTION CODE (MAKE SURE TO CHOOSE THE RIGHT ONE) ######
                find_and_move_to_block_top()
                navigation_matrix = mh.init_drop_off_zone_grid1() ## TODO MAKE SURE YOU CHANGE THIS FOR CORRECT DROP OFF!!!!!
                is_localized = False
                particles = [Particle(grid, valid_positions) for _ in range(NUM_PARTICLES)]
                rotation_degrees = 0
                continue
            else:
                print("arrived at drop off zone")
                running = False
                
                distance_t3 = lidar_distances[3]
                centering_distance = distance_t3 % 12 - 6
                if distance_t3 > FRONT_THRESHOLD + 1 and centering_distance > 0:
                    transmit(packetize(f'w0:{centering_distance}'))
                    [responses, time_rx] = receive()
                    time.sleep(abs(centering_distance) / forward_velocity + 0.25)
                transmit(packetize('o'))
                [responses, time_rx] = receive()
                break
        
        # Obstacle avoidance/Navigation
        distance_t0 = lidar_distances[0]
        distance_t1 = lidar_distances[1]
        distance_t2 = lidar_distances[2]
        distance_t3 = lidar_distances[3]
        distance_t4 = lidar_distances[4]
        distance_t5 = lidar_distances[5]
        distance_t6 = lidar_distances[6]
        
        
        # Navigation (forward drive only happens here!)       
        if distance_t0 > SIDE_THRESHOLD and distance_t1 > ANGLED_SIDE_THRESHOLD and distance_t2 > ANGLED_FRONT_THRESHOLD and distance_t3 > FRONT_THRESHOLD and distance_t4 > ANGLED_FRONT_THRESHOLD and distance_t5 > ANGLED_SIDE_THRESHOLD and distance_t6 > SIDE_THRESHOLD:
            angle_difference = mh.shortest_rotation_distance(navigation_dir, pred_theta)
            if abs(angle_difference) <= 20:  
                transmit(packetize('w0:50'))
                [responses, time_rx] = receive()
            else:
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                time.sleep(0.25)
                
                # update particle positions
                transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
                [responses, time_rx] = receive()
                lidar_distances = [float(response[1]) for response in responses[:8]]
                motor_steps = [int(response[1]) for response in responses[8:]]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)       
                pred_x, pred_y, pred_theta = mh.estimate(particles)
                pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)      
            
                print("CENTERING")
                distance_t3 = lidar_distances[3]
                centering_distance = distance_t3 % 12 - 3
                if distance_t3 > FRONT_THRESHOLD + 1 and centering_distance > 0:
                    transmit(packetize(f'w0:{max(centering_distance,0)}'))
                    [responses, time_rx] = receive()
                else:
                    print("ROTATING")
                    transmit(packetize(f'r0:{angle_difference}'))
                    [responses, time_rx] = receive()
                    time.sleep(abs(angle_difference / angular_velocity) + 0.5)
        else:
            if distance_t3 < FRONT_THRESHOLD or (distance_t2 < ANGLED_FRONT_THRESHOLD and distance_t4 < ANGLED_FRONT_THRESHOLD):
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                time.sleep(0.25)
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                pred_x, pred_y, pred_theta = mh.estimate(particles)
                pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)
                
                transmit(packetize('w0:-0.5'))
                [responses, time_rx] = receive()
                time.sleep(0.25)
            elif distance_t6 < SIDE_THRESHOLD or distance_t5 < ANGLED_SIDE_THRESHOLD or distance_t4 < ANGLED_FRONT_THRESHOLD:
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                pred_x, pred_y, pred_theta = mh.estimate(particles)
                pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)
                    
                transmit(packetize('d0:-0.25'))
                [responses, time_rx] = receive()
                time.sleep(0.25) # make sure drive command finishes
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                pred_x, pred_y, pred_theta = mh.estimate(particles)
                pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)
                
                transmit(packetize('r0:-2'))
                [responses, time_rx] = receive()
                time.sleep(0.25) # make sure drive command finishes
                
            elif distance_t0 < SIDE_THRESHOLD or distance_t1 < ANGLED_SIDE_THRESHOLD or distance_t2 < ANGLED_FRONT_THRESHOLD:
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                pred_x, pred_y, pred_theta = mh.estimate(particles)
                pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)
                
                transmit(packetize('d0:0.25'))
                [responses, time_rx] = receive()
                time.sleep(0.25) # make sure drive command finishes
                
                # update particle positions
                transmit(packetize('m1,m2,m3'))
                [responses, time_rx] = receive()
                motor_steps = [int(response[1]) for response in responses]
                drive_type, drive_value = mh.steps_to_movement(motor_steps[0], motor_steps[1], motor_steps[2])
                mh.update_particles(particles, drive_type, drive_value, grid)
                pred_x, pred_y, pred_theta = mh.estimate(particles)
                pred_theta = mh.normalize_angle(pred_theta * 180 / math.pi)
                
                transmit(packetize('r0:2'))
                [responses, time_rx] = receive()
                time.sleep(0.25) # make sure drive command finishes
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