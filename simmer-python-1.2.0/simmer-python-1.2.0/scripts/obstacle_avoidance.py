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
BAUDRATE = 115200         # Baudrate in bps
PORT_SERIAL = 'COM6'    # COM port identification
TIMEOUT_SERIAL = 0.5      # Serial port timeout, in seconds

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

def obstacle_avoidance(duration):

    LOOP_PAUSE_TIME = 0.05 # seconds

    # Main loop
    RUN_DEAD_RECKONING = True # If true, run this. If false, skip it

    # Function to check all ToF sensors (will be in mm!)

    BIG_FRONT_THRESHOLD = 7.5
    SMALL_FRONT_THRESHOLD = 4
    FRONT_THRESHOLD = SMALL_FRONT_THRESHOLD
    SIDE_THRESHOLD = 2.75 # distance_t0, distance_t6
    ANGLED_FRONT_THRESHOLD = 3 # distance_t2, distance_t4
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
                transmit(packetize('r0:-10'))
                [responses, time_rx] = receive()
                # update particles
            elif (distance_t0 < SIDE_THRESHOLD or distance_t1 < ANGLED_SIDE_THRESHOLD or distance_t2 < ANGLED_FRONT_THRESHOLD) and FRONT_THRESHOLD != BIG_FRONT_THRESHOLD:
                print('moving right')
                transmit(packetize('xx'))
                [responses, time_rx] = receive()
                transmit(packetize('d0:0.5'))
                [responses, time_rx] = receive()
                # update particles
                transmit(packetize('r0:10'))
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
                
                
obstacle_avoidance(25)