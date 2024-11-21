'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import socket
import time
from datetime import datetime
import serial

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

def approximately_equal(a: float, b: float, tolerance: float):
    return abs(a - b) <= tolerance


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
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0




############## Main section for the communication client ##############
RUN_COMMUNICATION_CLIENT = False # If true, run this. If false, skip it
while RUN_COMMUNICATION_CLIENT:
    # Input a command
    cmd = input('Type in a string to send: ')

    # Send the command
    packet_tx = packetize(cmd)
    if packet_tx:
        transmit(packet_tx)

    # Receive the response
    [responses, time_rx] = receive()
    if responses[0]:
        print(f"At time '{time_rx}' received from {SOURCE}:\n{response_string(cmd, responses)}")
    else:
        print(f"At time '{time_rx}' received from {SOURCE}:\nMalformed Packet")


COARSE_SWEEP_ANGLE = 100 # degrees
COARSE_SWEEP_INCREMENT = 5 # degrees
MAX_BOTTOM_SENSOR_READING = 19 # 19 inches
MIN_DELTA = 1.5 # minimum difference in top and bottom sensor readings to be considered significant for block detection
FINE_SWEEP_ANGLE = 60 # degrees
FINE_SWEEP_INCREMENT = 2 # degrees

angular_velocity = 90.0 # rad/s (actually ~100 rad/s)
forward_velocity = 4.0 # in/s (actually ~4.67 in/s)

def find_and_move_to_block():
    """
    Function to scan for a block using sensors t3 (top) and t7 (bottom),
    rotate back to the position with the maximum difference, and move towards the block
    in two stages: first to 4 inches away, then re-scan and adjust, and finally move to less than 1 inch away.
    """

    block_located = False # Flag variable to indicate whether block was found during search routine
    
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
            time.sleep(COARSE_SWEEP_INCREMENT / angular_velocity)

            # Read sensors t3 (top) and t7 (bottom)
            transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
            [responses, time_rx] = receive()
            ToF_distances = [float(response[1]) for response in responses[:8]]
            sensor_t3_reading = min(ToF_distances[3], MAX_BOTTOM_SENSOR_READING)
            sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)

            if sensor_t3_reading is None or sensor_t7_reading is None:
                print(f"Failed to read sensors at angle {COARSE_SWEEP_INCREMENT * (i + 1)} degrees. Storing difference as 0.")
                difference = 0
            else:
                # Calculate the difference
                difference = abs(sensor_t7_reading - sensor_t3_reading)
                print(f"Sensor difference at angle {COARSE_SWEEP_INCREMENT * (i + 1)} degrees: {difference} inches")

            # Store the angle and the difference
            angle_differences.append((COARSE_SWEEP_INCREMENT * (i + 1), difference))

        # Step 3: Find the angle with the maximum difference
        
        # CHANGE: Find average angle for which the delta is above 1.5"
        valid_angles = [angle for angle, diff in angle_differences if diff > MIN_DELTA]

        if not valid_angles:
            print(f"No valid sensor differences found above {MIN_DELTA} inches. Block may not be in the vicinity.")
            print("Moving to new location and attempting search routine again.")
            
            # Rotate 90, go forward 3 inches, rotate 45, then go 12 inches to new position 
            transmit(packetize(f'r0:{COARSE_SWEEP_ANGLE - 90}')) # Points robot forward
            [responses, time_rx] = receive()
            time.sleep((COARSE_SWEEP_ANGLE - 90) / angular_velocity)
            
            transmit(packetize('w0:3')) # Clears the near corner
            [responses, time_rx] = receive()
            time.sleep(3 / forward_velocity)
            
            transmit(packetize('r0:45')) # Points in general direction of unsearched zone
            [responses, time_rx] = receive()
            time.sleep(45 / angular_velocity)
            
            transmit(packetize('w0:8')) # Moves forward to get block within sensor range
            [responses, time_rx] = receive()
            time.sleep(8 / forward_velocity)
            
            transmit(packetize('r0:45')) # Points in general direction of unsearched zone
            [responses, time_rx] = receive()
            time.sleep(45 / angular_velocity)
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
    time.sleep(rotation_angle / angular_velocity)

    # Step 5: Move forward closer to block for fine sweep search (Stage 1)
    print("Stage 1: Moving closer towards block.")
    
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    ToF_distances = [float(response[1]) for response in responses[:8]]
    sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)
    print(f"Moving Forward: {sensor_t7_reading - 3} inches")
    
    transmit(packetize(f'w0:{sensor_t7_reading - 3}'))
    [responses, time_rx] = receive()
    
    time.sleep(abs(sensor_t7_reading - 3) / forward_velocity)

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
        time.sleep(FINE_SWEEP_INCREMENT / angular_velocity)

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
    time.sleep(rotation_angle / angular_velocity)
    
    # Step 7: Move forward until the bottom sensor reads less than 1 inch (Stage 2)
    print("Stage 2: Drive to the block.")
    
    transmit(packetize('t0,t1,t2,t3,t4,t5,t6,t7,m1,m2,m3'))
    [responses, time_rx] = receive()
    ToF_distances = [float(response[1]) for response in responses[:8]]
    sensor_t7_reading = min(ToF_distances[7], MAX_BOTTOM_SENSOR_READING)
    print(f"Moving Forward: {sensor_t7_reading + 0.5} inches")
    
    ################# LOWER THE GRIPPER HERE!!!!! ########################
    
    transmit(packetize(f'w0:{sensor_t7_reading + 0.5}'))
    [responses, time_rx] = receive()
    time.sleep(abs(sensor_t7_reading - 3) / 4) # 4 inches/sec speed 



    
    
############## Main section for the open loop control algorithm ##############
# The sequence of commands to run
LOOP_PAUSE_TIME = 0.1 # seconds

# Main loop
RUN_DEAD_RECKONING = True # If true, run this. If false, skip it

while RUN_DEAD_RECKONING:
    # Pause for a little while so as to not spam commands insanely fast
    time.sleep(LOOP_PAUSE_TIME)
    find_and_move_to_block()
    break