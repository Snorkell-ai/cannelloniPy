import struct
import socket
import threading
import time

# ---------------------------- Constants ----------------------------
CANNELLONI_FRAME_VERSION = 2
OPCODE = 1
CANFD_FRAME = 0x80
CANNELLONI_DATA_PACKET_BASE_SIZE = 4
CANNELLONI_FRAME_BASE_SIZE = 5
CAN_RTR_FLAG = 0x40000000

# ---------------------------- Utils ----------------------------
class CanfdFrame:
    def __init__(self):
        self.can_id = 0
        self.len = 0
        self.flags = 0
        self.data = bytearray(8)  # Assuming maximum payload size of 8 bytes

class FramesQueue:
    def __init__(self, count):
        self.head = 0
        self.tail = 0
        self.count = count
        self.frames = [CanfdFrame() for _ in range(count)]

    def put(self, frame):
        """Put a frame into the circular buffer.

        This method puts a frame into the circular buffer. If the buffer is
        full, it returns None.

        Args:
            frame: The frame to be inserted into the buffer.

        Returns:
            The inserted frame if successful, None if the buffer is full.
        """
 
        if (self.tail + 1) % self.count == self.head:
            return None
        self.frames[self.tail] = frame
        self.tail = (self.tail + 1) % self.count
        return frame

    def take(self):
        """Takes and returns the next frame from the frames list.

        If the head index is equal to the tail index, it means there are no
        frames to take and None is returned. Otherwise, it retrieves the frame
        at the head index, updates the head index to point to the next frame,
        and returns the retrieved frame.

        Returns:
            Any: The next frame from the frames list, or None if there are no frames left
                to take.
        """

        if self.head == self.tail:
            return None
        frame = self.frames[self.head]
        self.head = (self.head + 1) % self.count
        return frame

    def peek(self):
        """Return the element at the front of the queue without removing it.

        This method returns the element at the front of the queue without
        removing it.

        Returns:
            Any: The element at the front of the queue.
        """

        if self.head == self.tail:
            return None
        return self.frames[self.head]

class CannelloniHandle:
    def __init__(self, can_tx_fn=None, can_rx_fn=None, can_buf_size=64, remote_addr=None, remote_port=None):
        self.sequence_number = 0
        self.udp_rx_count = 0
        self.Init = {
            "remote_addr": remote_addr,
            "remote_port": remote_port,
            "can_buf_size": can_buf_size,
            "can_tx_buf": [CanfdFrame() for _ in range(can_buf_size)],
            "can_rx_buf": [CanfdFrame() for _ in range(can_buf_size)],
            "can_tx_fn": can_tx_fn,
            "can_rx_fn": can_rx_fn
        }
        self.tx_queue = FramesQueue(can_buf_size)
        self.rx_queue = FramesQueue(can_buf_size)
        self.udp_pcb = None
        self.can_pcb = None

    # Handle the received Cannelloni frame
    def handle_cannelloni_frame(handle, data, addr):
        """Handle the received Cannelloni frame.

        This function processes the received Cannelloni frame data. It first
        checks if the data packet is complete, then unpacks the data and
        validates the version and operation code. It then processes each CAN
        frame in the packet and puts it into the receive queue.

        Args:
            handle (object): The handle object for managing Cannelloni frames.
            data (bytes): The raw data of the received Cannelloni frame.
            addr: The address information associated with the received frame.
        """

        try:
            if len(data) < CANNELLONI_DATA_PACKET_BASE_SIZE:
                print("Cannellonipy lib: Received incomplete packet")
                return

            try:
                version, op_code, seq_no, count = struct.unpack('!BBBB', data[:CANNELLONI_DATA_PACKET_BASE_SIZE])
            except struct.error:
                print("Cannellonipy lib: Failed to unpack data")
                return
                
            if version != CANNELLONI_FRAME_VERSION or op_code != OPCODE:
                print("Cannellonipy lib: Invalid version or operation code")
                return

            pos = CANNELLONI_DATA_PACKET_BASE_SIZE
            handle.udp_rx_count += 1

            for _ in range(count):
                if pos + CANNELLONI_FRAME_BASE_SIZE > len(data):
                    print("Cannellonipy lib: Received incomplete packet 2")
                    break

                # Unpack the CAN frame
                can_frame = CanfdFrame()
                can_frame.can_id, can_frame.len = struct.unpack('!IB', data[pos:pos+5])
                pos += 5
                length = can_frame.len & ~CANFD_FRAME
                can_frame.flags = can_frame.len & CANFD_FRAME
                can_frame.len = length
                if (can_frame.can_id & CAN_RTR_FLAG) == 0:
                    can_frame.data[:length] = data[pos + 5:pos + 5 + length]

                handle.rx_queue.put(can_frame)

        except Exception as e:
            print("Cannellonipy lib: Error while handling Cannelloni packet: ", e)
            return
    
    def get_received_can_frames(self):
        """Retrieve all received CAN frames from the receive queue.

        This method continuously retrieves CAN frames from the receive queue
        until it encounters a None value, indicating the end of available
        frames. It then clears the received CAN frames from the internal buffer.

        Returns:
            list: A list of received CAN frames.
        """

        frames = []
        while True:
            frame = self.rx_queue.take()
            if frame is None:
                break
            frames.append(frame)
        self.clear_received_can_frames()
        return frames

    def clear_received_can_frames(self):
        """Clear all received CAN frames from the receive queue.

        This function continuously takes frames from the receive queue until it
        encounters a None value, indicating the end of the queue.

        Args:
            self: The instance of the CAN bus controller.
        """

        while True:
            frame = self.rx_queue.take()
            if frame is None:
                break

# ---------------------------- Execution ----------------------------
# Run the Cannellonipy library
def run_cannellonipy(handle, remote_addr, remote_port):
    """Run the Cannellonipy library.

    This function initializes the Cannellonipy library with the provided
    remote address and port. It opens UDP socket and mocks the opening of
    the CAN socket. If either UDP or CAN socket opening fails, it prints a
    message and returns without starting service threads.

    Args:
        handle: The handle object for Cannellonipy.
        remote_addr (str): The remote address to connect to.
        remote_port (int): The remote port to connect to.
    """

    print("Running Cannellonipy...")
    handle.Init["remote_addr"] = remote_addr
    handle.Init["remote_port"] = int(remote_port)

    open_udp_socket(handle)
    # open_can_socket(handle) TODO
    handle.can_pcb = True # Mocking the opening of the CAN socket
    if not handle.udp_pcb or not handle.can_pcb:
        print("Cannellonipy lib: Failed to open sockets")
        return

    # Start all the service threads 
    receive_can_frames_thread = threading.Thread(target=receive_can_frames, args=(handle,), daemon=True) 
    receive_can_frames_thread.start()
    transmit_can_frames_thread = threading.Thread(target=transmit_can_frames, args=(handle,), daemon=True) 
    transmit_can_frames_thread.start()
    receive_udp_packets_thread = threading.Thread(target=receive_udp_packets, args=(handle,), daemon=True)
    receive_udp_packets_thread.start()
    transmit_udp_packets_thread = threading.Thread(target=transmit_udp_packets, args=(handle,), daemon=True)
    transmit_udp_packets_thread.start()

# Create a UDP socket (send/receive)
def open_udp_socket(handle):
    """Create a UDP socket for sending and receiving data.

    This function creates a UDP socket using the provided handle's remote
    address and port. If the socket creation is successful, it prints a
    success message with the port and address.

    Args:
        handle (dict): A dictionary containing the remote address and port.
    """

    try:
        handle.udp_pcb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Check with cmd:  sudo tcpdump -i any udp port 1234 -X
        handle.udp_pcb.bind((handle.Init["remote_addr"], handle.Init["remote_port"]))
        if not handle.udp_pcb:
            print("Cannellonipy lib: Failed to create UDP socket")
            return
        else:
            print("Cannellonipy lib: UDP socket created successfully on port: ", handle.Init["remote_port"], " and address: ", handle.Init["remote_addr"])
    except Exception as e:
        print("Cannellonipy lib: Failed to create UDP socket: ", e)
        return

# Create a CAN socket (send/receive)
def open_can_socket(handle):
    """Create a CAN socket for sending and receiving data.

    This function creates a CAN socket for sending and receiving data. It
    checks if the handle has a CAN PCB attribute and prints a success
    message if the socket is created successfully on interface can0.

    Args:
        handle: The handle object containing CAN PCB information.
    """

    try:
        # TODO
        if not handle.can_pcb:
            print("Cannellonipy lib: Failed to create CAN socket")
            return
        else:
            print("Cannellonipy lib: CAN socket created successfully on interface can0")
    except Exception as e:
        print("Cannellonipy lib: Failed to create CAN socket: ", e)
        return

# Transmit CAN frames over UDP
def transmit_udp_packets(handle):
    """Transmit CAN frames over UDP.

    This function continuously transmits CAN frames over UDP using the
    provided handle.

    Args:
        handle (object): An object containing necessary information and configurations for
            transmitting CAN frames.
    """

    try:
        while True:
            frame = handle.tx_queue.take()
            if frame is not None:
                data = bytearray()
                data.extend(struct.pack('!BBBB', CANNELLONI_FRAME_VERSION, OPCODE, handle.sequence_number, 1))
                data.extend(struct.pack('!IB', frame.can_id, frame.len | frame.flags))
                data.extend(frame.data[:frame.len])
                handle.udp_pcb.sendto(data, (handle.Init["remote_addr"], handle.Init["remote_port"]))
                handle.sequence_number = (handle.sequence_number + 1) % 256
    except Exception as e:
        print("Cannellonipy lib: Error while transmitting UDP packets: ", e)
        return

# Receive UDP packets
def receive_udp_packets(handle):
    """Receive UDP packets and handle them using the provided handle.

    This function continuously receives UDP packets using the provided
    handle's UDP PCB. If data is received, it is passed to the
    handle_cannelloni_frame method of the handle.

    Args:
        handle: An object that provides access to the UDP PCB and the
            handle_cannelloni_frame method.
    """

    try:
        while True:
            data, addr = handle.udp_pcb.recvfrom(1024)
            if data:
                handle.handle_cannelloni_frame(data, addr)
    except OSError as e:
        if e.errno == 9:  # Check if the error is "Bad file descriptor"
            pass  # Ignore the error silently
        else:
            print("Cannellonipy lib: Error while receiving UDP packets:", e)
    except Exception as e:
        print("Cannellonipy lib: Error while receiving UDP packets: ", e)
        return

def receive_can_frames(handle):
    # TODO: Implement this function
    # This function should receive CAN frames and put them in the tx_queue
    pass

def transmit_can_frames(handle):
    # TODO: Implement this function
    # This function should transmit CAN frames from the rx_queue
    pass


# ---------------------------- Cannelloni message composition ----------------------------

# UDP packet format:
# 1 byte - Version
# 1 byte - Operation code
# 1 byte - Sequence number
# 1 byte - Number of CAN frames
# - CAN frame format:
# - 4 bytes - CAN ID
# - 1 byte - Length of hexadecimal data
# - N bytes - Data

# -----------------------------------------------------------------------------------------
# | Version | Operation code | Sequence number | Number of CAN frames | CAN frame 1 | ... |
# -----------------------------------------------------------------------------------------

# EXAMPLE of a UDP packet:
# 020100010000007b0d48656c6c6f2c20576f726c6421
# 02 - Version
# 01 - Operation code
# 00 - Sequence number
# 01 - Number of CAN frames
# 0000007b - CAN ID
# 0d - Length of data
# 48656c6c6f2c20576f726c6421 - CAN DATA
# CAN DATA:
# 48 65 6c 6c 6f 2c 20 57 6f 72 6c 64 21 -> Hello, World!
