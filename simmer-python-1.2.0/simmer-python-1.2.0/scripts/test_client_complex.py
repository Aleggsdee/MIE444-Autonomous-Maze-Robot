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

############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM6'    # COM port identification
TIMEOUT_SERIAL = 0.5      # Serial port timeout, in seconds

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
    TRANSMIT_PAUSE = 0.05
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




############## Main section for the open loop control algorithm ##############
# The sequence of commands to run
LOOP_PAUSE_TIME = 0.05 # seconds

# Main loop
RUN_DEAD_RECKONING = True # If true, run this. If false, skip it

# Function to check all ToF sensors (will be in mm!)
def get_sensor_distances():
    sensors = ['t0', 't1', 't2', 't3', 't4']
    sensor_distances = {}
    for sensor in sensors:
        packet_tx = packetize(sensor)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print("ToF " + sensor[1] + f" reading: {response_string('t2',responses)}")
            ToF_reading = float(responses[0][1])
            sensor_distances[sensor] = ToF_reading
    return sensor_distances


BIG_FRONT_THRESHOLD = 7.5
SMALL_FRONT_THRESHOLD = 6
FRONT_THRESHOLD = SMALL_FRONT_THRESHOLD
SIDE_MIN_THRESHOLD = 2.75 # 2.5
ANGLED_THRESHOLD = 5 # 4

previous_time = time.time()
transmit(packetize('xx'))
[responses, time_rx] = receive()
while RUN_DEAD_RECKONING:
    current_time = time.time()
    # Pause for a little while so as to not spam commands insanely fast
    time.sleep(LOOP_PAUSE_TIME)
    
    sensor_distances = get_sensor_distances()
    
    if SIMULATE:
        distance_t0 = sensor_distances['t0']
        distance_t1 = sensor_distances['t1']
        distance_t2 = sensor_distances['t2']
        distance_t3 = sensor_distances['t3']
        distance_t4 = sensor_distances['t4']
    else:
        # Convert to inches
        distance_t0 = sensor_distances['t0'] / 25.4
        distance_t1 = sensor_distances['t1'] / 25.4
        distance_t2 = sensor_distances['t2'] / 25.4
        distance_t3 = sensor_distances['t3'] / 25.4
        distance_t4 = sensor_distances['t4'] / 25.4
    
    
    if distance_t2 > FRONT_THRESHOLD and distance_t4 > SIDE_MIN_THRESHOLD and distance_t0 > SIDE_MIN_THRESHOLD and distance_t3 > ANGLED_THRESHOLD and distance_t1 > ANGLED_THRESHOLD:
        print('moving forward')
        print('FRONT_THRESHOLD = ' + f'{FRONT_THRESHOLD}')
        if FRONT_THRESHOLD == BIG_FRONT_THRESHOLD:
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            FRONT_THRESHOLD = SMALL_FRONT_THRESHOLD
        transmit(packetize('w0:50'))
        current_time = time.time()
        time_difference = current_time - previous_time
        print(f"Time between iterations: {time_difference:.6f} seconds")
        [responses, time_rx] = receive()
    else:
        if (distance_t2 < 1 or (distance_t3 < ANGLED_THRESHOLD and distance_t1 < ANGLED_THRESHOLD)) and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD: # TODO change the t2 distance comparison later
            print('backing up')
            print(FRONT_THRESHOLD)
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            transmit(packetize('w0:-0.25'))
            [responses, time_rx] = receive()
            time.sleep(0.25)
        elif (distance_t4 <= SIDE_MIN_THRESHOLD or (distance_t3 <= ANGLED_THRESHOLD)) and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD:
            print('moving left')
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            transmit(packetize('d0:0.1'))
            [responses, time_rx] = receive()
            time.sleep(0.25)
            transmit(packetize('r0:0.8'))
            [responses, time_rx] = receive()
            time.sleep(0.25)
        elif (distance_t0 <= SIDE_MIN_THRESHOLD or (distance_t1 <= ANGLED_THRESHOLD)) and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD:
            print('moving right')
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            transmit(packetize('d0:-0.1'))
            [responses, time_rx] = receive()
            time.sleep(0.25)
            transmit(packetize('r0:-0.8'))
            [responses, time_rx] = receive()
            time.sleep(0.25)
        elif distance_t2 < FRONT_THRESHOLD and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD:
            print('rotating')
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            if distance_t4 > distance_t0:
                transmit(packetize('r0:-80')) # MIGHT NEED TO CHANGE
                [responses, time_rx] = receive()
            else:
                transmit(packetize('r0:80')) # MIGHT NEED TO CHANGE
                [responses, time_rx] = receive()      
            FRONT_THRESHOLD = BIG_FRONT_THRESHOLD
            print('FRONT_THRESHOLD = ' + f'{FRONT_THRESHOLD}')
        else:
            transmit(packetize('r0:170')) # MIGHT NEED TO CHANGE
            [responses, time_rx] = receive()  
    # Calculate the time difference
    time_difference = current_time - previous_time
    print(f"Time between iterations: {time_difference:.6f} seconds")
    previous_time = current_time  # Update the previous time