#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Receive_test.py

UDP receiver to receive 5 float32 little-endian values from vision_test_1.py
and print them.

Based on UDP_Receive.py
"""

import socket
import struct

# Set the IP and PORT to listen on
UDP_IP = "127.0.0.1"
UDP_PORT = 50001

# Create the socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind to the socket
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening on IP: {UDP_IP}, Port: {UDP_PORT}")
print("Expecting 5 float32 little-endian values: robot_x, robot_y, robot_yaw, target_x, target_y")

# Listen indefinitely
while True:
    # Receive data (5 floats * 4 bytes = 20 bytes)
    data, addr = sock.recvfrom(1024)
    if len(data) == 20:
        # Unpack 5 float32 little-endian
        try:
            values = struct.unpack('<fffff', data)
            robot_x, robot_y, robot_yaw, target_x, target_y = values
            print(f"Received from {addr}: Robot(x={robot_x:.2f}, y={robot_y:.2f}, yaw={robot_yaw:.1f}) Target(x={target_x:.2f}, y={target_y:.2f})")
        except struct.error as e:
            print(f"Error unpacking data: {e}, raw bytes: {list(data)}")
    else:
        print(f"Received unexpected data length: {len(data)}, from {addr}, bytes: {list(data)}")