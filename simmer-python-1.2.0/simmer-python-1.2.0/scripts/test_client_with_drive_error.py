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
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
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
LOOP_PAUSE_TIME = 0.1 # seconds

# Main loop
RUN_DEAD_RECKONING = True # If true, run this. If false, skip it

# Define the collision avoidance threshold in inches
COLLISION_THRESHOLD = 3  # 3 inches

while RUN_DEAD_RECKONING:
    # Pause for a little while so as to not spam commands insanely fast
    time.sleep(LOOP_PAUSE_TIME)

    # Check an ultrasonic sensor 'u0'
    packet_tx = packetize('u0')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        print(f"Ultrasonic 0 reading: {response_string('u0',responses)}")
        
        # Extract the distance reading from the response
        distance_u0 = float(responses[0][1])  # Assuming the sensor sends distance as the second value

    # Check an ultrasonic sensor 'u1'
    packet_tx = packetize('u1')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
        
        # Extract the distance reading from the response
        distance_u1 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
        
    # Check an ultrasonic sensor 'u2'
    packet_tx = packetize('u2')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        print(f"Ultrasonic 2 reading: {response_string('u2',responses)}")
        
        # Extract the distance reading from the response
        distance_u2 = float(responses[0][1])  # Assuming the sensor sends distance as the second value

    # Check an ultrasonic sensor 'u3'
    packet_tx = packetize('u3')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        print(f"Ultrasonic 3 reading: {response_string('u3',responses)}")
        
        # Extract the distance reading from the response
        distance_u3 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
        
    # Check an ultrasonic sensor 'u4'
    packet_tx = packetize('u4')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        print(f"Ultrasonic 4 reading: {response_string('u4',responses)}")
        
        # Extract the distance reading from the response
        distance_u4 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
        
    
    
    FRONT_THRESHOLD = 4 # distance_u0 should be 3.5
    SIDE_THRESHOLD = 3
    
    
    if distance_u0 > FRONT_THRESHOLD and distance_u1 > SIDE_THRESHOLD and distance_u3 > SIDE_THRESHOLD: 
        if approximately_equal(distance_u1, distance_u2, 0.35) or approximately_equal(distance_u3, distance_u4, 0.35) or abs(distance_u1 - distance_u2) > 6 or abs(distance_u3 - distance_u4) > 6:
            transmit(packetize('w0:50'))
            [responses, time_rx] = receive()
        else: # not parallel to walls
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
            
            print('realignment time')
            
            if distance_u1 + distance_u2 > distance_u3 + distance_u4:
                rotation_direction = distance_u1 - distance_u2 # positive means we need to correct orientation by rotating CW
            else:
                rotation_direction = distance_u4 - distance_u3 # positive means we need to correct orientation by rotating CW
            
            
            rotation_str = str(rotation_direction * 20)            
            transmit(packetize('r0:' + rotation_str))
            [responses, time_rx] = receive()
            while (not approximately_equal(distance_u1, distance_u2, 0.05) and not approximately_equal(distance_u3, distance_u4, 0.05)):
                # Check an ultrasonic sensor 'u1'
                packet_tx = packetize('u1')
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
                    
                    # Extract the distance reading from the response
                    distance_u1 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
                    
                # Check an ultrasonic sensor 'u2'
                packet_tx = packetize('u2')
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 2 reading: {response_string('u2',responses)}")
                    
                    # Extract the distance reading from the response
                    distance_u2 = float(responses[0][1])  # Assuming the sensor sends distance as the second value

                # Check an ultrasonic sensor 'u3'
                packet_tx = packetize('u3')
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 3 reading: {response_string('u3',responses)}")
                    
                    # Extract the distance reading from the response
                    distance_u3 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
                    
                # Check an ultrasonic sensor 'u4'
                packet_tx = packetize('u4')
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Ultrasonic 4 reading: {response_string('u4',responses)}")
                    
                    # Extract the distance reading from the response
                    distance_u4 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
            transmit(packetize('xx'))
            [responses, time_rx] = receive()
    elif distance_u0 <= FRONT_THRESHOLD and (distance_u1 <= SIDE_THRESHOLD or distance_u3 <= SIDE_THRESHOLD):
        transmit(packetize('xx'))
        [responses, time_rx] = receive()
        transmit(packetize('w0:-2'))
        [responses, time_rx] = receive()
    elif distance_u0 <= FRONT_THRESHOLD:
        transmit(packetize('xx'))
        [responses, time_rx] = receive()
        
        print('rotate to find new path')
        
        rotation_dir = (distance_u1 + distance_u2) - (distance_u3 + distance_u4) # if more room on the right, then rotate clockwise
        if distance_u0 < 2.5:
            transmit(packetize('w0:-1.5'))
        str(rotation_dir * 180)
        transmit(packetize('r0:' + str(rotation_dir * 180)))
        [responses, time_rx] = receive()
        while (distance_u0 <= 8 or (not approximately_equal(distance_u1, distance_u2, 0.15) and not approximately_equal(distance_u3, distance_u4, 0.15))):
            # Check an ultrasonic sensor 'u0'
            packet_tx = packetize('u0')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                print(f"Ultrasonic 0 reading: {response_string('u0',responses)}")
                
                # Extract the distance reading from the response
                distance_u0 = float(responses[0][1])
            
            # Check an ultrasonic sensor 'u1'
            packet_tx = packetize('u1')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
                
                # Extract the distance reading from the response
                distance_u1 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
                
            # Check an ultrasonic sensor 'u2'
            packet_tx = packetize('u2')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                print(f"Ultrasonic 2 reading: {response_string('u2',responses)}")
                
                # Extract the distance reading from the response
                distance_u2 = float(responses[0][1])  # Assuming the sensor sends distance as the second value

            # Check an ultrasonic sensor 'u3'
            packet_tx = packetize('u3')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                print(f"Ultrasonic 3 reading: {response_string('u3',responses)}")
                
                # Extract the distance reading from the response
                distance_u3 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
                
            # Check an ultrasonic sensor 'u4'
            packet_tx = packetize('u4')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                print(f"Ultrasonic 4 reading: {response_string('u4',responses)}")
                
                # Extract the distance reading from the response
                distance_u4 = float(responses[0][1])  # Assuming the sensor sends distance as the second value
        transmit(packetize('xx'))
        [responses, time_rx] = receive()        
    elif distance_u1 < SIDE_THRESHOLD:
        transmit(packetize('xx'))
        [responses, time_rx] = receive()
        transmit(packetize('d0:-0.25'))
        [responses, time_rx] = receive()
    elif distance_u3 < SIDE_THRESHOLD:
        transmit(packetize('xx'))
        [responses, time_rx] = receive()
        transmit(packetize('d0:0.25'))
        [responses, time_rx] = receive()   
    else:
        RUN_DEAD_RECKONING = False
        print("don't know what to do")
        break
        
# while
#     if all sensors < their specified distance (need to set these so that you can still fully rotate):
#         if left sensors are parallel or right sensors are parallel or difference between left 2 sensors is greater than 11 inches or difference between right 2 sensors is greater than 11 inches:
#             drive forward
#         else: # not parallel to walls 
#             rotate CW or CCW based on left or right sensor values until rover is aligned then stop the drive command
#     elif front sensor and (left or right):
#         back up
#     elif front sensor is too close:
#         if left sensor readings > right sensor readings:
#             rotate CCW until right sensors are parallel
#         else:
#             rotate CW until left sensors are parallel
#     elif left front sensor is too close:
#         rotate and drive forward (need to determine how much to drive forward and how much to rotate)
#     elif left front sensor is too close:
#         rotate and drive forward (need to determine how much to drive forward and how much to rotate)
    
        
            
    
    