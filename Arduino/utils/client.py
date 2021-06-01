import time
import queue
import serial
import threading
import serial.tools.list_ports

BUTTON_NONE      =   0x00
BUTTON_Y         =   0x01
BUTTON_B         =   0x02
BUTTON_A         =   0x04
BUTTON_X         =   0x08
BUTTON_L         =   0x10
BUTTON_R         =   0x20
BUTTON_ZL        =   0x40
BUTTON_ZR        =   0x80
BUTTON_MINUS     =  0x100
BUTTON_PLUS      =  0x200
BUTTON_LCLICK    =  0x400
BUTTON_RCLICK    =  0x800
BUTTON_HOME      = 0x1000
BUTTON_CAPTURE   = 0x2000

DPAD_UP          = 0x00
DPAD_UP_RIGHT    = 0x01
DPAD_RIGHT       = 0x02
DPAD_DOWN_RIGHT  = 0x03
DPAD_DOWN        = 0x04
DPAD_DOWN_LEFT   = 0x05
DPAD_LEFT        = 0x06
DPAD_UP_LEFT     = 0x07
DPAD_CENTER      = 0x08

STICK_MIN        = -1.0
STICK_CENTER     =  0.0
STICK_MAX        =  1.0

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

class Packet:
    def __init__(self, buttons=set(), dpad=DPAD_CENTER, lx=STICK_CENTER, ly=STICK_CENTER, rx=STICK_CENTER, ry=STICK_CENTER):
        self.buttons = set(buttons)
        self.dpad = dpad
        self.lx = lx
        self.ly = ly
        self.rx = rx
        self.ry = ry
        self.vendorspec = b'\x00'

    @staticmethod
    def f2b(val):
        return int((val + 1.0) / 2.0 * 255).to_bytes(1, byteorder='big')

    def __bytes__(self):
        return sum(self.buttons).to_bytes(2, byteorder='big') + self.dpad.to_bytes(1, byteorder='big') + self.f2b(self.lx) + self.f2b(self.ly) + self.f2b(self.rx) + self.f2b(self.ry) + self.vendorspec

    def __repr__(self):
        return f"{self.__class__.__name__}(buttons={self.buttons!r}, dpad={self.dpad!r}, lx={self.lx!r}, ly={self.ly!r}, rx={self.rx!r}, ry={self.ry!r})"
    
class Controller:
    def __init__(self, serial_port=None, vid=None, pid=None):
        if serial_port is None:
            serial_port = self.find_arduino(vid, pid)
        self.serial_port = serial_port

    @staticmethod
    def find_arduino(vid, pid):
        arduino_ports = [
            p.device
            for p in serial.tools.list_ports.comports()
            if p.vid == vid and p.pid == pid
        ]
        if not arduino_ports:
            raise OSError("No Arduino found")
        if len(arduino_ports) > 1:
            print("Found multiple Arduinos, using the first")
        return arduino_ports[0]

    @staticmethod
    def wait(duration, min_sleep=0.01):
        start = time.perf_counter()
        if duration > min_sleep:
            time.sleep(duration - min_sleep)
        while time.perf_counter() - start < duration:
            pass

    def send_cmd(self, duration=None, **kwargs):
        if duration is None:
            self.q.put_nowait(Packet(**kwargs))
        else:
            self.q.put_nowait(Packet(**kwargs))
            self.q.join()
            self.wait(duration)
            self.q.put_nowait(Packet())
            self.q.join()
        return self

    def push_buttons(self, *buttons, duration=None):
        self.send_cmd(buttons=buttons, duration=duration)

    def push_dpad(self, dpad, duration=None):
        self.send_cmd(dpad=dpad, duration=duration)

    def move_left_stick(self, x, y, duration=None):
        self.send_cmd(lx=x, ly=y, duration=duration)

    def move_right_stick(self, x, y, duration=None):
        self.send_cmd(rx=x, ry=y, duration=duration)

    def reset(self):
        self.send_cmd()

    def connect(self):
        self.push_buttons(BUTTON_L, BUTTON_R, duration=1)

    def wait_for_data(self, timeout=1.0, sleep_time=0.1):
        """Wait for data to be available on the serial port."""
        t_0 = time.perf_counter()
        while (time.perf_counter() - t_0 < timeout or self.ser.in_waiting == 0):
            time.sleep(sleep_time)

    def read_bytes(self, size):
        """Read X bytes from the serial port (returns list)"""
        bytes_in = self.ser.read(size)
        return list(bytes_in)

    def read_byte(self):
        """Read 1 byte from the serial port (returns int)"""
        bytes_in = self.read_bytes(1)
        byte_in = bytes_in[0] if bytes_in else 0
        return byte_in

    def read_byte_latest(self):
        """Discard all incoming bytes and read the last (latest) (returns int)"""
        bytes_in_waiting = max(self.ser.in_waiting, 1)
        bytes_in = self.read_bytes(bytes_in_waiting)
        byte_in = bytes_in[-1] if bytes_in else 0
        return byte_in

    def write_bytes(self, bytes_out):
        """Write bytes to the serial port"""
        self.ser.write(bytearray(bytes_out))
        while self.ser.out_waiting:
            pass

    def write_byte(self, byte_out):
        """Write byte to the serial port"""
        self.write_bytes([byte_out])

    def crc8_ccitt(self, old_crc, new_data):
        """Compute CRC8

        See:
        https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1gab27eaaef6d7fd096bd7d57bf3f9ba083.html"""
        data = old_crc ^ new_data

        for _ in range(8):
            if (data & 0x80) != 0:
                data = data << 1
                data = data ^ 0x07
            else:
                data = data << 1
            data = data & 0xff
        return data

    def send_packet(self, packet=Packet()):
        """Send a raw packet and wait for a response (CRC will be added automatically)"""
        bytes_out = bytearray(bytes(packet))
    
        # Compute CRC
        crc = 0
        for byte in bytes_out:
            crc = self.crc8_ccitt(crc, byte)
        bytes_out.append(crc)

        # Purge input buffer and send bytes
        self.ser.reset_input_buffer()
        self.write_bytes(bytes_out)
        # Wait for USB ACK or UPDATE NACK
        byte_in = self.read_byte()
        success = (byte_in == RESP_USB_ACK)
        return success

    def run(self):
        while True:
            packet = self.q.get()
            if packet is None:
                break
            self.send_packet(packet)
            self.q.task_done()

    def force_sync(self):
        """Force MCU to sync"""
        # Send 9x 0xFF's to fully flush out buffer on device
        # Device will send back 0xFF (RESP_SYNC_START) when it is ready to sync
        self.write_bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])

        # Wait for serial data and read the last byte sent
        self.wait_for_data()
        byte_in = self.read_byte_latest()

        # Begin sync...
        in_sync = False
        if byte_in == RESP_SYNC_START:
            self.write_byte(COMMAND_SYNC_1)
            byte_in = self.read_byte()
            if byte_in == RESP_SYNC_1:
                self.write_byte(COMMAND_SYNC_2)
                byte_in = self.read_byte()
                if byte_in == RESP_SYNC_OK:
                    in_sync = True
        return in_sync

    def sync(self):
        """Start MCU syncing process"""
        # Try sending a packet
        in_sync = self.send_packet()
        if not in_sync:
            # Not in sync: force resync and send a packet
            in_sync = self.force_sync()
            if in_sync:
                in_sync = self.send_packet()
        return in_sync

    def __enter__(self):
        print(f"Opening port {self.serial_port}")
        self.ser = serial.Serial(self.serial_port, 1000000, timeout=1)
        if not self.sync():
            raise OSError("Failed to sync with MCU")
        self.q = queue.Queue()
        # self.t = threading.Thread(target=self.run)
        # self.t.start()
        return self

    def __exit__(self, *args):
        # self.q.put(None)
        # self.t.join()
        self.ser.close()

if __name__ == '__main__':
    with Controller(vid=1027, pid=24577) as controller:
        controller.send_packet()
        controller.wait(1)
        for _ in range(5):
            controller.send_packet(Packet(lx=STICK_MIN, ly=STICK_MIN))
            controller.wait(3)
            controller.send_packet(Packet(lx=STICK_CENTER, ly=STICK_MAX))
            controller.wait(0.4)
            controller.send_packet()
            controller.wait(1)

        # controller.connect()
        # controller.send_cmd(duration=0.5)
        # # print("Resetting cursor...")
        # controller.move_left_stick(STICK_MIN, STICK_MIN, duration=3)
        # time.sleep(0.5)
        # for i in range(5):
        #     # print(f"Iteration {i}...")
        #     controller.move_left_stick(STICK_CENTER, STICK_MAX, duration=0.4)
        #     time.sleep(0.5)
        #     controller.move_left_stick(STICK_MIN, STICK_MIN, duration=1)