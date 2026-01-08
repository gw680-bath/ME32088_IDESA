#!/usr/bin/env python3
"""
Basic UDP sender for sending an array of 5 numbers to Simulink UDP receive block.
"""

import socket
import struct

# UDP target IP and port (adjust as needed for your Raspberry Pi setup)
UDP_IP = "127.0.0.1"  # localhost if Simulink is on the same machine
UDP_PORT = 50001

# Array of 5 numbers (floats)
data = [1.0, 2.0, 3.0, 4.0, 5.0]

# Pack the array into bytes (5 floats, each 4 bytes)
message = struct.pack('5f', *data)

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)
print("Sending data:", data)

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Send the packed data
sock.sendto(message, (UDP_IP, UDP_PORT))

print("Data sent successfully")