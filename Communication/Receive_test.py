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
print("Expecting a space-separated string of 5 float32 values: robot_x, robot_y, robot_yaw, target_x, target_y")

# Listen indefinitely
while True:
    # Receive data
    data, addr = sock.recvfrom(1024)
    try:
        # Decode as UTF-8 string
        data_str = data.decode('utf-8').strip()
        # Split into values
        values = [float(x) for x in data_str.split()]
        if len(values) == 5:
            robot_x, robot_y, robot_yaw, target_x, target_y = values
            print(f"Received from {addr}: Robot(x={robot_x:.2f}, y={robot_y:.2f}, yaw={robot_yaw:.1f}) Target(x={target_x:.2f}, y={target_y:.2f})")
        else:
            print(f"Received unexpected number of values: {len(values)}, string: '{data_str}'")
    except (UnicodeDecodeError, ValueError) as e:
        print(f"Error parsing data: {e}, raw bytes: {list(data)}")