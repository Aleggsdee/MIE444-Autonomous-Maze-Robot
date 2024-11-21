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
PORT_SERIAL = 'COM6'    # COM port identification
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




def find_and_move_to_block():
    """
    Function to scan for a block using sensors t3 (top) and t7 (bottom),
    rotate back to the position with the maximum difference, and move towards the block
    in two stages: first to 4 inches away, then re-scan and adjust, and finally move to less than 1 inch away.
    """
    # Step 1: Turn 90 degrees with the command w0:90
    print("Turning 90 degrees.")
    transmit(packetize('r0:90'))
    [responses, time_rx] = receive()
    time.sleep(2)

    # Step 2: Perform a 180-degree scan in 5-degree increments, storing sensor differences
    print("Starting initial 180-degree scan in 5-degree increments.")
    angle_differences = []  # List to store (angle, difference) tuples
    for i in range(36):  # 180 degrees / 5 degrees per increment
        # Turn 5 degrees
        transmit(packetize('r0:-5'))
        [responses, time_rx] = receive()

        # Read sensors t3 (top) and t7 (bottom)
        sensor_t3_reading = get_sensor_reading('t3')
        sensor_t7_reading = get_sensor_reading('t7')

        if sensor_t3_reading is None or sensor_t7_reading is None:
            print(f"Failed to read sensors at angle {5 * (i + 1)} degrees. Storing difference as 0.")
            difference = 0
        else:
            # Calculate the difference
            difference = abs(sensor_t7_reading - sensor_t3_reading)
            print(f"Sensor difference at angle {5 * (i + 1)} degrees: {difference} inches")

        # Store the angle and the difference
        angle_differences.append((5 * (i + 1), difference))

    # Step 3: Find the angle with the maximum difference
    
    # CHANGE: Find average angle for which the delta is above 1.5"
    valid_angles = [angle for angle, diff in angle_differences if diff > 1.5]

    if not valid_angles:
        print("No valid sensor differences found above 1.5 inches. Block may not be in the vicinity.")
        return
    else:
        # Calculate the average angle
        avg_angle = sum(valid_angles) / len(valid_angles)
        print(f"Average angle with sensor difference above 1.5 inches: {avg_angle} degrees.")
        

    # Step 4: Rotate back to the angle with the maximum difference
    # Calculate the angle to rotate back (since we've already turned 180 degrees)
    rotation_angle = (180 - avg_angle)
    print(f"Rotating back {rotation_angle} degrees to face the block.")
    transmit(packetize(f'r0:{rotation_angle}'))
    [responses, time_rx] = receive()
    time.sleep(2)

    # Step 5: Move forward until the bottom sensor reads less than or equal to 4 inches (Stage 1)
    print("Stage 1: Moving towards the block until 4 inches away.")
    
    transmit(packetize('t7'))
    [responses, time_rx] = receive()    
    distance_t7 = float(responses[0][1])
    print(f"Moving Forward: {distance_t7 - 3} inches")
    
    transmit(packetize(f'w0:{distance_t7 - 3}'))
    [responses, time_rx] = receive()
    
    time.sleep((distance_t7 - 3) / 4) # 4 inches/sec speed

    # Step 6: Re-scan and adjust alignment (repeat Steps 2 to 4)
    print("Starting adjustment scan to refine alignment.")

    # Optional: Turn 30 degrees before starting the adjustment scan if needed
    print("Turning 30 degrees for adjustment scan.")
    transmit(packetize('r0:30'))
    [responses, time_rx] = receive()

    # Repeat the 180-degree scan in 5-degree increments
    angle_differences = []
    for i in range(60):  # 180 degrees / 5 degrees per increment
        # Turn 5 degrees
        transmit(packetize('r0:-1'))
        [responses, time_rx] = receive()

        # Read sensors t3 and t7
        sensor_t3_reading = get_sensor_reading('t3')
        sensor_t7_reading = get_sensor_reading('t7')

        if sensor_t3_reading is None or sensor_t7_reading is None:
            print(f"Failed to read sensors at adjustment angle {1 * (i + 1)} degrees. Storing difference as 0.")
            difference = 0
        else:
            # Calculate the difference
            difference = abs(sensor_t7_reading - sensor_t3_reading)
            print(f"Sensor difference at adjustment angle {1 * (i + 1)} degrees: {difference} inches")

        # Store the angle and the difference
        angle_differences.append((1 * (i + 1), difference))

    # CHANGE: Find average angle for which the delta is above 1.5"
    valid_angles = [angle for angle, diff in angle_differences if diff > 1.5]

    if not valid_angles:
        print("No valid sensor differences found above 1.5 inches. Block may not be in the vicinity.")
        return
    else:
        # Calculate the average angle
        avg_angle = sum(valid_angles) / len(valid_angles)
        print(f"Average angle with sensor difference above 1.5 inches: {avg_angle} degrees.")
        

    # Step 6: Rotate back to the angle with the maximum difference
    # Calculate the angle to rotate back (since we've already turned 180 degrees)
    rotation_angle = (180 - avg_angle)
    print(f"Rotating back {avg_angle} degrees to refine alignment.")
    transmit(packetize(f'r0:{avg_angle}'))
    [responses, time_rx] = receive()

    time.sleep(2)
    
    # Step 7: Move forward until the bottom sensor reads less than 1 inch (Stage 2)
    print("Stage 2: Drive to the block.")
    
    transmit(packetize('t7'))
    [responses, time_rx] = receive()    
    distance_t7 = float(responses[0][1])
    print(f"Moving Forward: {distance_t7 + 0.5} inches")
    
    ################# LOWER THE GRIPPER HERE!!!!! ########################
    
    transmit(packetize(f'w0:{distance_t7 + 0.5}'))
    [responses, time_rx] = receive()
    
    time.sleep((distance_t7 - 3) / 4) # 4 inches/sec speed

def get_sensor_reading(sensor_id):
    """
    Helper function to get a reading from a specific sensor.
    """
    packet_tx = packetize(sensor_id)
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        if responses and responses[0][0] == sensor_id:
            try:
                print(f"{sensor_id} Sensor Readings: {responses[0][1]}")
                return float(responses[0][1])
            except ValueError:
                print(f"Invalid reading from sensor {sensor_id}: {responses[0][1]}")
                return None
        else:
            print(f"Unexpected response for {sensor_id}: {responses}")
            return None
    else:
        print(f"Failed to packetize command for {sensor_id}")
        return None
    
    
    
    
        
            
    
    
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