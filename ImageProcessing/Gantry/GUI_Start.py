import tkinter as tk
from tkinter import ttk
import cv2
import cv2.aruco as aruco
import numpy as np
import threading
import time
import os
from PIL import Image, ImageTk
import paho.mqtt.client as mqtt

# Load the camera calibration values
script_dir = os.path.dirname(os.path.abspath(__file__))
camera_calibration = np.load(os.path.join(script_dir, 'ME32088_IDESA', 'ImageProcessing', 'CalibrationGantry.npz'))
CM = camera_calibration['CM']  # camera matrix
dist_coef = camera_calibration['dist_coef']  # distortion coefficients from the camera

# Define the ArUco dictionary and parameters
marker_size = 40
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Assume robot marker ID = 0, targets 1-8
robot_id = 0
target_ids = list(range(1, 9))

class GUI_Start:
    def __init__(self, root):
        self.root = root
        self.root.title("GUI_Start - Manual Control and Tracking")
        self.running = False
        self.cap = None
        self.thread = None

        # MQTT setup
        self.client = mqtt.Client()
        self.client.username_pw_set("student", password="HousekeepingGlintsStreetwise")
        self.client.connect("fesv-mqtt.bath.ac.uk", 31415, 60)
        self.client.loop_start()
        self.main_topic = "TeamTopic"

        # Video display label
        self.video_label = tk.Label(root)
        self.video_label.pack(pady=10)

        # Buttons frame
        button_frame = tk.Frame(root)
        button_frame.pack(pady=10)

        self.start_btn = tk.Button(button_frame, text="Start", command=self.start, width=10)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = tk.Button(button_frame, text="Stop", command=self.stop, width=10)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        # Key bindings for manual control
        self.root.bind('<Up>', self.move_up)
        self.root.bind('<Down>', self.move_down)
        self.root.bind('<Left>', self.move_left)
        self.root.bind('<Right>', self.move_right)

        # Label for instructions
        instructions = tk.Label(root, text="Use arrow keys for manual control: Up, Down, Left, Right")
        instructions.pack(pady=10)

    def start(self):
        if not self.running:
            self.running = True
            self.cap = cv2.VideoCapture(0)
            self.thread = threading.Thread(target=self.update_frame)
            self.thread.start()
            self.client.publish(self.main_topic, "start")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        if self.cap:
            self.cap.release()
        self.client.publish(self.main_topic, "stop")

    def update_frame(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if ids is not None:
                    # Draw detected markers
                    frame = aruco.drawDetectedMarkers(frame, corners, ids)

                    # Estimate pose
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, CM, dist_coef)

                    for i, id_val in enumerate(ids.flatten()):
                        rvec = rvecs[i]
                        tvec = tvecs[i]
                        # Draw axis
                        frame = cv2.drawFrameAxes(frame, CM, dist_coef, rvec, tvec, 100)

                        # Label robot and targets
                        corner = corners[i][0][0]  # top-left corner
                        if id_val == robot_id:
                            cv2.putText(frame, "Robot", (int(corner[0]), int(corner[1]) - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        elif id_val in target_ids:
                            cv2.putText(frame, f"Target_{id_val}", (int(corner[0]), int(corner[1]) - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                # Convert to PIL Image for Tkinter
                img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(img)
                imgtk = ImageTk.PhotoImage(image=img)
                self.video_label.imgtk = imgtk
                self.video_label.configure(image=imgtk)

            time.sleep(0.1)  # Adjust for frame rate

    def move_up(self, event):
        print("Move Up")
        self.client.publish(self.main_topic, "up")

    def move_down(self, event):
        print("Move Down")
        self.client.publish(self.main_topic, "down")

    def move_left(self, event):
        print("Move Left")
        self.client.publish(self.main_topic, "left")

    def move_right(self, event):
        print("Move Right")
        self.client.publish(self.main_topic, "right")

if __name__ == "__main__":
    root = tk.Tk()
    app = GUI_Start(root)
    root.mainloop()