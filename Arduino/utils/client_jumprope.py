#!/usr/bin/env python3
"""Client for sending controller commands to a controller emulator."""
import sys
import argparse
import math
import time
import serial

STATE_OUT_OF_SYNC   = 0
STATE_SYNC_START    = 1
STATE_SYNC_1        = 2
STATE_SYNC_2        = 3
STATE_SYNC_OK       = 4

# Actual Switch DPAD Values
A_DPAD_CENTER = 0x08
A_DPAD_U      = 0x00
A_DPAD_U_R    = 0x01
A_DPAD_R      = 0x02
A_DPAD_D_R    = 0x03
A_DPAD_D      = 0x04
A_DPAD_D_L    = 0x05
A_DPAD_L      = 0x06
A_DPAD_U_L    = 0x07

# Enum DIR Values
DIR_CENTER = 0x00
DIR_U      = 0x01
DIR_R      = 0x02
DIR_D      = 0x04
DIR_L      = 0x08
DIR_U_R    = DIR_U + DIR_R
DIR_D_R    = DIR_D + DIR_R
DIR_U_L    = DIR_U + DIR_L
DIR_D_L    = DIR_D + DIR_L

BTN_NONE    = 0x0000000000000000
BTN_Y       = 0x0000000000000001
BTN_B       = 0x0000000000000002
BTN_A       = 0x0000000000000004
BTN_X       = 0x0000000000000008
BTN_L       = 0x0000000000000010
BTN_R       = 0x0000000000000020
BTN_ZL      = 0x0000000000000040
BTN_ZR      = 0x0000000000000080
BTN_MINUS   = 0x0000000000000100
BTN_PLUS    = 0x0000000000000200
BTN_LCLICK  = 0x0000000000000400
BTN_RCLICK  = 0x0000000000000800
BTN_HOME    = 0x0000000000001000
BTN_CAPTURE = 0x0000000000002000

DPAD_CENTER = 0x0000000000000000
DPAD_U      = 0x0000000000010000
DPAD_R      = 0x0000000000020000
DPAD_D      = 0x0000000000040000
DPAD_L      = 0x0000000000080000
DPAD_U_R    = DPAD_U + DPAD_R
DPAD_D_R    = DPAD_D + DPAD_R
DPAD_U_L    = DPAD_U + DPAD_L
DPAD_D_L    = DPAD_D + DPAD_L

LSTICK_CENTER = 0x0000000000000000
LSTICK_R      = 0x00000000FF000000 #   0 (000)
LSTICK_U_R    = 0x0000002DFF000000 #  45 (02D)
LSTICK_U      = 0x0000005AFF000000 #  90 (05A)
LSTICK_U_L    = 0x00000087FF000000 # 135 (087)
LSTICK_L      = 0x000000B4FF000000 # 180 (0B4)
LSTICK_D_L    = 0x000000E1FF000000 # 225 (0E1)
LSTICK_D      = 0x0000010EFF000000 # 270 (10E)
LSTICK_D_R    = 0x0000013BFF000000 # 315 (13B)

RSTICK_CENTER = 0x0000000000000000
RSTICK_R      = 0x000FF00000000000 #   0 (000)
RSTICK_U_R    = 0x02DFF00000000000 #  45 (02D)
RSTICK_U      = 0x05AFF00000000000 #  90 (05A)
RSTICK_U_L    = 0x087FF00000000000 # 135 (087)
RSTICK_L      = 0x0B4FF00000000000 # 180 (0B4)
RSTICK_D_L    = 0x0E1FF00000000000 # 225 (0E1)
RSTICK_D      = 0x10EFF00000000000 # 270 (10E)
RSTICK_D_R    = 0x13BFF00000000000 # 315 (13B)

NO_INPUT      = BTN_NONE + DPAD_CENTER + LSTICK_CENTER + RSTICK_CENTER

# Commands to send to MCU
COMMAND_NOP        = 0x00
COMMAND_SYNC_1     = 0x33
COMMAND_SYNC_2     = 0xCC
COMMAND_SYNC_START = 0xFF

# Responses from MCU
RESP_USB_ACK       = 0x90
RESP_UPDATE_ACK    = 0x91
RESP_UPDATE_NACK   = 0x92
RESP_SYNC_START    = 0xFF
RESP_SYNC_1        = 0xCC
RESP_SYNC_OK       = 0x33

# 'this' is a pointer to the module object instance itself.
this = sys.modules[__name__]

# Global Variable: Serial Session
this.serial_session = None

def angle(stick_angle, intensity):
    """Compute x and y based on angle and intensity."""
    # y is negative because on the Y input, UP = 0 and DOWN = 255
    pos_x =  int((math.cos(math.radians(stick_angle)) * 0x7F) * intensity / 0xFF) + 0x80
    pos_y = -int((math.sin(math.radians(stick_angle)) * 0x7F) * intensity / 0xFF) + 0x80
    return pos_x, pos_y

def lstick_angle(stick_angle, intensity):
    """Compute x and y based on angle and intensity for the left stick."""
    return (intensity + (stick_angle << 8)) << 24

def rstick_angle(stick_angle, intensity):
    """Compute x and y based on angle and intensity for the right stick."""
    return (intensity + (stick_angle << 8)) << 44

def p_wait(wait_time, min_sleep_time=0.002, error_correction=0):
    """Wait x seconds (precise)."""
    t_0 = time.perf_counter()
    corrected_wait_time = wait_time - error_correction
    if corrected_wait_time > min_sleep_time:
        time.sleep(corrected_wait_time - min_sleep_time)
    t_1 = time.perf_counter()
    t_2 = t_1
    while t_2 - t_0 < corrected_wait_time:
        t_2 = time.perf_counter()
    return t_2 - t_0

def p_timer(t_0, wait_time, min_sleep_time=0.002, error_correction=0):
    """Pad time by x seconds (precise)."""
    corrected_wait_time = wait_time - error_correction
    t_1 = time.perf_counter()
    if corrected_wait_time > min_sleep_time:
        time.sleep(corrected_wait_time - min_sleep_time - (t_1 - t_0))
    t_2 = time.perf_counter()
    while t_2 - t_0 < corrected_wait_time:
        t_2 = time.perf_counter()
    return t_2 - t_0

def wait_for_data(timeout=1.0, sleep_time=0.1):
    """Wait for data to be available on the serial port."""
    t_0 = time.perf_counter()
    t_1 = t_0
    bytes_in_waiting = this.serial_session.in_waiting
    while ((t_1 - t_0 < timeout) or (bytes_in_waiting == 0)):
        time.sleep(sleep_time)
        bytes_in_waiting = this.serial_session.in_waiting
        t_1 = time.perf_counter()
    return bytes_in_waiting

def read_bytes(size):
    """Read X bytes from the serial port (returns list)."""
    bytes_in = this.serial_session.read(size)
    return list(bytes_in)

def read_byte():
    """Read 1 byte from the serial port (returns int)."""
    bytes_in = read_bytes(1)
    if bytes_in:
        byte_in = bytes_in[0]
    else:
        byte_in = 0
    return byte_in

def read_byte_latest():
    """Discard all incoming bytes and read the last (latest) (returns int)."""
    bytes_in_waiting = this.serial_session.in_waiting
    if bytes_in_waiting == 0:
        bytes_in_waiting = 1
    bytes_in = read_bytes(bytes_in_waiting)
    if bytes_in:
        byte_in = bytes_in[-1]
    else:
        byte_in = 0
    return byte_in

def write_bytes(bytes_out):
    """Write bytes to the serial port."""
    this.serial_session.write(bytearray(bytes_out))
    this.serial_session.flush()

def write_byte(byte_out):
    """Write byte to the serial port."""
    write_bytes([byte_out])

def crc8_ccitt(old_crc, new_data):
    """Compute CRC8

    See:
    https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1gab27eaaef6d7fd096bd7d57bf3f9ba083.html"""
    data = old_crc ^ new_data

    for dummy in range(8):
        if (data & 0x80) != 0:
            data = data << 1
            data = data ^ 0x07
        else:
            data = data << 1
        data = data & 0xff
    return data

def send_packet(packet=None,
                debug=False):
    """Send a raw packet and wait for a response (CRC will be added automatically)."""
    if not packet:
        packet = [0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00]

    if not debug:
        # t_0 = time.perf_counter()
        bytes_out = []
        bytes_out.extend(packet)

        # Compute CRC
        crc = 0
        for byte in packet:
            crc = crc8_ccitt(crc, byte)
        bytes_out.append(crc)
        # t_1 = time.perf_counter()

        # Purge input buffer, send bytes and wait for USB ACK or UPDATE NACK
        this.serial_session.reset_input_buffer()
        write_bytes(bytes_out)
        # t2 = time.perf_counter()
        byte_in = read_byte()
        # t3 = time.perf_counter()
        success = (byte_in == RESP_USB_ACK)
        # print(bytes_out, byte_in)
        # print("Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}', f'{((t2 - t_1) * 1000):4.6}', f'{((t3 - t2) * 1000):4.6}', f'{((t3 - t_0) * 1000):4.6}')
    else:
        success = True
    return success

def decrypt_dpad(dpad):
    """convert DPAD value to actual DPAD value used by Switch."""
    if dpad == DIR_U:
        dpad_out = A_DPAD_U
    elif dpad == DIR_R:
        dpad_out = A_DPAD_R
    elif dpad == DIR_D:
        dpad_out = A_DPAD_D
    elif dpad == DIR_L:
        dpad_out = A_DPAD_L
    elif dpad == DIR_U_R:
        dpad_out = A_DPAD_U_R
    elif dpad == DIR_U_L:
        dpad_out = A_DPAD_U_L
    elif dpad == DIR_D_R:
        dpad_out = A_DPAD_D_R
    elif dpad == DIR_D_L:
        dpad_out = A_DPAD_D_L
    else:
        dpad_out = A_DPAD_CENTER
    return dpad_out

def cmd_to_packet(command):
    """Convert CMD to a packet."""
    cmd_copy = command

    low              = (cmd_copy & 0xFF)
    cmd_copy         = cmd_copy >>  8

    high             = (cmd_copy & 0xFF)
    cmd_copy         = cmd_copy >>  8

    dpad             = (cmd_copy & 0xFF)
    cmd_copy         = cmd_copy >>  8

    l_intensity      = (cmd_copy & 0xFF)
    cmd_copy         = cmd_copy >>  8

    l_angle          = (cmd_copy & 0xFFF)
    cmd_copy         = cmd_copy >> 12

    r_intensity      = (cmd_copy & 0xFF)
    cmd_copy         = cmd_copy >>  8

    r_angle          = (cmd_copy & 0xFFF)

    dpad = decrypt_dpad(dpad)
    left_x, left_y   = angle(l_angle, l_intensity)
    right_x, right_y = angle(r_angle, r_intensity)

    packet = [high, low, dpad, left_x, left_y, right_x, right_y, 0x00]
    # print(hex(command), packet, lstick_angle, lstick_intensity, rstick_angle, rstick_intensity)
    return packet

def send_cmd(command=NO_INPUT):
    """Send a formatted controller command to the MCU."""
    success = send_packet(cmd_to_packet(command))
    return success

def testbench_btn():
    """Test all buttons except for home and capture."""
    send_cmd(BTN_A)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(BTN_B)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(BTN_X)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(BTN_Y)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(BTN_PLUS)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(BTN_MINUS)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(BTN_LCLICK)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(BTN_RCLICK)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

def testbench_dpad():
    """Test DPAD U / R / D / L."""
    send_cmd(DPAD_U)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(DPAD_R)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(DPAD_D)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(DPAD_L)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

def testbench_dpad_diag():
    """Test DPAD diagonals - does not register on switch due to dpad buttons."""
    send_cmd(DPAD_U_R)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(DPAD_D_R)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(DPAD_D_L)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(DPAD_U_L)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

def testbench_lstick():
    """Test left analog stick."""
    #Test U/R/D/L
    send_cmd(BTN_LCLICK)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(LSTICK_U)
    p_wait(0.5)
    send_cmd(LSTICK_R)
    p_wait(0.5)
    send_cmd(LSTICK_D)
    p_wait(0.5)
    send_cmd(LSTICK_L)
    p_wait(0.5)
    send_cmd(LSTICK_U)
    p_wait(0.5)
    send_cmd(LSTICK_CENTER)
    p_wait(0.5)

    # 360 Circle @ Full Intensity
    for i in range(0, 721):
        cmd = lstick_angle(i + 90, 0xFF)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(LSTICK_CENTER)
    p_wait(0.5)

    # 360 Circle @ Partial Intensity
    for i in range(0, 721):
        cmd = lstick_angle(i + 90, 0x80)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(LSTICK_CENTER)
    p_wait(0.5)

def testbench_rstick():
    """Test right analog stick."""
    #Test U/R/D/L
    send_cmd(BTN_RCLICK)
    p_wait(0.5)
    send_cmd()
    p_wait(0.001)

    send_cmd(RSTICK_U)
    p_wait(0.5)
    send_cmd(RSTICK_R)
    p_wait(0.5)
    send_cmd(RSTICK_D)
    p_wait(0.5)
    send_cmd(RSTICK_L)
    p_wait(0.5)
    send_cmd(RSTICK_U)
    p_wait(0.5)
    send_cmd(RSTICK_CENTER)
    p_wait(0.5)

    # 360 Circle @ Full Intensity
    for i in range(0, 721):
        cmd = rstick_angle(i + 90, 0xFF)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(RSTICK_CENTER)
    p_wait(0.5)

    # 360 Circle @ Partial Intensity
    for i in range(0, 721):
        cmd = rstick_angle(i + 90, 0x80)
        send_cmd(cmd)
        p_wait(0.001)
    send_cmd(RSTICK_CENTER)
    p_wait(0.5)

def testbench_packet_speed(count=100, sleep_time=0.050):
    """Test packet speed."""
    testbench_sum = 0
    testbench_min = 999
    testbench_max = 0
    testbench_avg = 0
    testbench_err = 0

    for i in range(0, count + 1):

        # Send packet and check time
        t_0 = time.perf_counter()

        if i % 2:
            status = send_cmd(BTN_A)
        else:
            status = send_cmd(NO_INPUT)
        t_1 = time.perf_counter()
        p_wait(sleep_time)
        t_2 = time.perf_counter()
        # print(f'{((t_1 - t_0) * 1000):4.6}', f'{((t_2 - t_0) * 1000):4.6}')
        print('%.6f' % ((t_1 - t_0)*1000), '%.6f' % ((t_2 - t_0)*1000))
        # Count errors
        if not status:
            err += 1
            print('Packet Error!')

        # Compute times
        testbench_delta = t_1 - t_0
        if testbench_delta < testbench_min:
            testbench_min = testbench_delta
        if testbench_delta > testbench_max:
            testbench_max = testbench_delta
        testbench_sum = testbench_sum + (t_1 - t_0)

    testbench_avg = testbench_sum / i
    print('Min =', '{:.3f}'.format(testbench_min),
          'Max =', '{:.3}'.format(testbench_max),
          'Avg =', '{:.3f}'.format(testbench_avg),
          'Errors =', testbench_err)

def testbench():
    """Test controller functions."""
    testbench_btn()
    testbench_dpad()
    testbench_lstick()
    testbench_rstick()
    testbench_packet_speed()

def force_sync():
    """Force MCU to sync."""
    # Send 9x 0xFF's to fully flush out buffer on device
    # Device will send back 0xFF (RESP_SYNC_START) when it is ready to sync
    write_bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
    time.sleep(0.100)
    this.serial_session.reset_input_buffer()
    write_byte(0xFF)


    # Wait for serial data and read the last byte sent
    wait_for_data()
    byte_in = read_byte_latest()

    # Begin sync...
    in_sync = False
    if byte_in == RESP_SYNC_START:
        write_byte(COMMAND_SYNC_1)
        byte_in = read_byte()
        if byte_in == RESP_SYNC_1:
            write_byte(COMMAND_SYNC_2)
            byte_in = read_byte()
            if byte_in == RESP_SYNC_OK:
                in_sync = True
    return in_sync

def sync():
    """Start MCU syncing process."""
    in_sync = False

    # Try sending a packet
    in_sync = send_packet()
    if not in_sync:
        # Not in sync: force resync and send a packet
        in_sync = force_sync()
        if in_sync:
            in_sync = send_packet()
    return in_sync


def main(arguments):
    """Run the module."""

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    parser.add_argument('port', help="Serial Port")
    args = parser.parse_args(arguments)

    # this.serial_session = serial.Serial(port=args.port, baudrate=19200, timeout=1)
    this.serial_session = serial.Serial(port=args.port, baudrate=31250, timeout=1)
    # this.serial_session = serial.Serial(port=args.port, baudrate=40000, timeout=1)
    # this.serial_session = serial.Serial(port=args.port, baudrate=62500, timeout=1)

    # Attempt to sync with the MCU
    if not sync():
        print('Could not sync!')

    macro()

    # testbench()
    # testbench_packet_speed(100)

    p_wait(1)
    print(this.serial_session.in_waiting)
    print(this.serial_session.read(this.serial_session.in_waiting))

    this.serial_session.close()

def macro():
    """FFIX Jumprope Macro. Use after a fresh Switch reboot."""

    def send(command, duration=0):
        if not send_cmd(command):
            print('Packet Error on Press!')
        p_wait(duration)
        if not send_cmd(NO_INPUT):
            print('Packet Error on Release!')

    def single_jump(jumps, jump_period, button_press_time, first_jump_time=0):
        for jump in range(1, jumps + 1):
            t_0 = time.perf_counter()
            send(BTN_A, button_press_time)
            t_1 = time.perf_counter()
            if jump == 1 and first_jump_time > 0:
                p_timer(t_0, first_jump_time, error_correction=0.0000019)
                t_2 = time.perf_counter()
                t_error = (jump_period/2) - t_2 - t_0
            else:
                p_timer(t_0, jump_period, error_correction=0.0000019)
                t_2 = time.perf_counter()
                t_error = abs(jump_period - (t_2 - t_0))
            print(f'{jump:03}', "Time elapsed (ms): ",
                  f'{((t_2 - t_0) * 1000):4.6}',
                  f'{((t_1 - t_0) * 1000):4.6}',
                  f'{((t_error) * 1000000):4.6}')

    def double_jump(jumps, jump_period, button_press_time, first_jump_time=0):
        for jump in range(1, jumps + 1):
            t_0 = time.perf_counter()
            send(BTN_A, button_press_time)
            p_wait(0.05)
            send(BTN_A, button_press_time)
            t_1 = time.perf_counter()
            if jump == 1 and first_jump_time > 0:
                p_timer(t_0, first_jump_time, error_correction=0.0000019)
            else:
                p_timer(t_0, jump_period, error_correction=0.0000019)
            t_2 = time.perf_counter()
            print(f'{jump:03}', "Time elapsed (ms): ",
                  f'{((t_2 - t_0) * 1000):4.6}',
                  f'{((t_1 - t_0) * 1000):4.6}')

    button_press_time = 0.05
    button_delay1 = 0.670
    button_delay2 = 0.532
    button_delay3 = 0.466
    button_delay4 = 0.433
    button_delay5 = 0.383
    button_delay6 = 0.400

    t_0 = time.perf_counter()
    print('Come play with us again -> Nothing')
    send(BTN_A, button_press_time)
    t_1 = time.perf_counter()
    p_wait(3)
    print("Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}')

    t_0 = time.perf_counter()
    print('Nothing -> You wanna try?')
    send(BTN_A, button_press_time)
    t_1 = time.perf_counter()
    p_wait(3)
    print("Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}')

    t_0 = time.perf_counter()
    print('You wanna try? -> 0')
    send(BTN_A, button_press_time)
    t_1 = time.perf_counter()
    p_wait(3)
    print("Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}')

    single_jump(20, button_delay1, button_press_time, button_delay1 / 2)
    print('Transitioning - 20-49')
    single_jump(30, button_delay2, button_press_time)
    print('Transitioning - 50 - 99')
    single_jump(50, button_delay3, button_press_time)
    print('Transitioning - 100-199')
    single_jump(100, button_delay4, button_press_time)
    print('Transitioning - 200-299')
    double_jump(100, button_delay5, button_press_time, 0.360)
    # double_jump(100, button_delay5, button_press_time, button_delay4)
    print('Transitioning - 300+')
    single_jump(2000, button_delay6, button_press_time, button_delay6 * 0.66)
    print('Transitioning')

    # print('Transitioning')

    # count2 = 30
    # end2 = end1 + count2
    # for x in range(end1 + 1, end2 + 1):
        # t_0 = time.perf_counter()
        # send(BTN_A, button_press_time)
        # p_timer(t_0, button_delay2)
        # t_1 = time.perf_counter()
        # print(f'{x:03}', "Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}')

    # print('Transitioning')

    # count3 = 50
    # end3 = end2 + count3
    # for x in range(end2 + 1, end3 + 1):
        # t_0 = time.perf_counter()
        # send(BTN_A, button_press_time)
        # p_timer(t_0, button_delay3)
        # t_1 = time.perf_counter()
        # print(f'{x:03}', "Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}')

    # print('Transitioning')

    # count4 = 100
    # end4 = end3 + count4
    # for x in range(end3 + 1, end4 + 1):
        # t_0 = time.perf_counter()
        # send(BTN_A, button_press_time)
        # p_timer(t_0, button_delay4)
        # t_1 = time.perf_counter()
        # print(f'{x:03}', "Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}')

    # print('Transitioning')
    # p_wait(0.385)

    # count5 = 100
    # end5 = end4 + count5
    # for x in range(end4 + 1, end5 + 1):
        # t_0 = time.perf_counter()
        # send(BTN_A, button_press_time)
        # send(BTN_A, button_press_time)
        # if x != end5:
            # p_wait(button_delay5)
        # t_1 = time.perf_counter()
        # print(f'{x:03}', "Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}')

    # print('Transitioning')

    # count6 = 1000
    # end6 = end5 + count6
    # for x in range(end5 + 1, end6 + 1):
        # t_0 = time.perf_counter()
        # send(BTN_A, button_press_time)
        # p_timer(t_0, button_delay6)
        # t_1 = time.perf_counter()
        # print(f'{x:03}', "Time elapsed (ms): ", f'{((t_1 - t_0) * 1000):4.6}')

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
