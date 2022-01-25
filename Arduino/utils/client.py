import time
from enum import IntEnum
import queue
import threading
import serial
import serial.tools.list_ports

DELAY_PER_UPDATE = 0.001


class Button(IntEnum):
    """"Controller buttons."""

    NONE = 0x00
    Y = 0x01
    B = 0x02
    A = 0x04
    X = 0x08
    L = 0x10
    R = 0x20
    ZL = 0x40
    ZR = 0x80
    MINUS = 0x100
    PLUS = 0x200
    LCLICK = 0x400
    RCLICK = 0x800
    HOME = 0x1000
    CAPTURE = 0x2000


class Dpad(IntEnum):
    """Controller d-pad directions."""

    UP = 0x00
    UP_RIGHT = 0x01
    RIGHT = 0x02
    DOWN_RIGHT = 0x03
    DOWN = 0x04
    DOWN_LEFT = 0x05
    LEFT = 0x06
    UP_LEFT = 0x07
    CENTER = 0x08


class Stick(IntEnum):
    """Controller analog stick positions."""

    MIN = -1.0
    CENTER = 0.0
    MAX = 1.0


class Command(IntEnum):
    """Commands to send to MCU."""

    NOP = 0x00
    SYNC_1 = 0x33
    SYNC_2 = 0xCC
    SYNC_START = 0xFF


class Resp(IntEnum):
    """Responses from MCU."""

    USB_ACK = 0x90
    UPDATE_ACK = 0x91
    UPDATE_NACK = 0x92
    SYNC_START = 0xFF
    SYNC_1 = 0xCC
    SYNC_OK = 0x33


class Packet:
    def __init__(
        self,
        buttons=set(),
        dpad=Dpad.CENTER,
        lx=Stick.CENTER,
        ly=Stick.CENTER,
        rx=Stick.CENTER,
        ry=Stick.CENTER,
    ):
        self.buttons = set(buttons)
        self.dpad = dpad
        self.lx = lx
        self.ly = ly
        self.rx = rx
        self.ry = ry
        self.vendorspec = b"\x00"

    @staticmethod
    def f2b(val):
        """"Convert float within [-1, 1] to byte."""
        return int((val + 1.0) / 2.0 * 255).to_bytes(1, byteorder="big")

    def __bytes__(self):
        return (
            sum(self.buttons).to_bytes(2, byteorder="big")
            + self.dpad.to_bytes(1, byteorder="big")
            + self.f2b(self.lx)
            + self.f2b(self.ly)
            + self.f2b(self.rx)
            + self.f2b(self.ry)
            + self.vendorspec
        )

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
    def sleep(duration, min_sleep=0.01):
        start = time.perf_counter()
        if duration > min_sleep:
            time.sleep(duration - min_sleep)
        while time.perf_counter() - start < duration:
            pass

    def wait_for_data(self, timeout=1.0, sleep_time=0.1):
        """Wait for data to be available on the serial port."""
        start = time.perf_counter()
        while time.perf_counter() - start < timeout or self.ser.in_waiting == 0:
            time.sleep(sleep_time)

    def read_bytes(self, size):
        """Read X bytes from the serial port (returns list)."""
        bytes_in = self.ser.read(size)
        return list(bytes_in)

    def read_byte(self):
        """Read 1 byte from the serial port (returns int)."""
        bytes_in = self.read_bytes(1)
        byte_in = bytes_in[0] if bytes_in else 0
        return byte_in

    def read_byte_latest(self):
        """Discard all incoming bytes and read the last (latest) (returns int)."""
        bytes_in_waiting = max(self.ser.in_waiting, 1)
        bytes_in = self.read_bytes(bytes_in_waiting)
        byte_in = bytes_in[-1] if bytes_in else 0
        return byte_in

    def write_bytes(self, bytes_out):
        """Write bytes to the serial port."""
        self.ser.write(bytearray(bytes_out))
        while self.ser.out_waiting:
            pass

    def write_byte(self, byte_out):
        """Write byte to the serial port."""
        self.write_bytes([byte_out])

    def crc8_ccitt(self, old_crc, new_data):
        """Compute CRC8.

        See:
        https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__util__crc_1gab27eaaef6d7fd096bd7d57bf3f9ba083.html"""
        data = old_crc ^ new_data

        for _ in range(8):
            if (data & 0x80) != 0:
                data = data << 1
                data = data ^ 0x07
            else:
                data = data << 1
            data = data & 0xFF
        return data

    def send_packet(self, packet=Packet()):
        """Send a raw packet and wait for a response (CRC will be added automatically)."""
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
        return byte_in == Resp.USB_ACK

    def force_sync(self):
        """Force MCU to sync."""
        # Send 9x 0xFF's to fully flush out buffer on device
        # Device will send back 0xFF (Resp.SYNC_START) when it is ready to sync
        self.write_bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])

        # Wait for serial data and read the last byte sent
        self.wait_for_data()
        byte_in = self.read_byte_latest()

        # Begin sync...
        in_sync = False
        if byte_in == Resp.SYNC_START:
            self.write_byte(Command.SYNC_1)
            byte_in = self.read_byte()
            if byte_in == Resp.SYNC_1:
                self.write_byte(Command.SYNC_2)
                byte_in = self.read_byte()
                if byte_in == Resp.SYNC_OK:
                    in_sync = True
        return in_sync

    def sync(self):
        """Start MCU syncing process."""
        # Try sending a packet
        in_sync = self.send_packet()
        if not in_sync:
            # Not in sync: force resync and send a packet
            in_sync = self.force_sync()
            if in_sync:
                in_sync = self.send_packet()
        return in_sync

    def send_command(self, duration=None, **kwargs):
        self.q.put_nowait((Packet(**kwargs), duration))
        return self

    def push_buttons(self, *buttons, duration=None):
        return self.send_command(buttons=buttons, duration=duration)

    def push_dpad(self, dpad, duration=None):
        return self.send_command(dpad=dpad, duration=duration)

    def move_left_stick(self, x, y, duration=None):
        return self.send_command(lx=x, ly=y, duration=duration)

    def move_right_stick(self, x, y, duration=None):
        return self.send_command(rx=x, ry=y, duration=duration)

    def reset(self, duration=None):
        return self.send_command(duration=duration)

    def connect(self):
        return self.push_buttons(Button.L, Button.R, duration=0.1)

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            packet, duration = item

            while time.perf_counter() - self.last_update < DELAY_PER_UPDATE:
                pass
            self.send_packet(packet)
            self.last_update = time.perf_counter()

            if duration is not None:
                self.sleep(duration)
            self.q.task_done()

    def join(self):
        self.q.join()

    def __enter__(self):
        print(f"Opening port {self.serial_port}")
        self.ser = serial.Serial(self.serial_port, 1_000_000, timeout=1)
        if not self.sync():
            raise OSError("Failed to sync with MCU")

        self.q = queue.Queue()
        self.last_update = time.perf_counter()
        self.t = threading.Thread(target=self.run)
        self.t.start()
        return self

    def __exit__(self, *args):
        self.q.put(None)
        self.t.join()
        self.ser.close()


if __name__ == "__main__":
    with Controller(vid=1027, pid=24577) as controller:
        print("Connecting...")
        controller.connect().reset(0.1).join()
        print("Moving to top left corner...")
        controller.move_left_stick(Stick.MIN, Stick.MIN, duration=3).reset(0.1).join()
        print("Testing...")
        for i in range(5):
            print(f"Iteration {i}")
            controller.move_left_stick(0, Stick.MAX, duration=0.4).reset(1).move_left_stick(Stick.MIN, Stick.MIN, duration=1).reset(0.1).join()
        print("Done")
