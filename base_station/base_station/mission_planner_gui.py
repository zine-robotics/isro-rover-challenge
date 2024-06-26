from pygame_map2 import setwaypoints, get_map, setnav
import tkinter as tk
import socket
import numpy as np
import json
import pygame  
import os

# Establish socket connection
HOST = 'localhost'  # Replace with the server IP address
PORT = 8000  # Replace with the desired port number

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
sock.connect((HOST, PORT))

map = get_map()
map = map.tolist()

chunk_size = 1024

saved_coords = []
saved_angle = []
saved_coord = []
mission = []

# Function to handle the "CALIBRATE" button click
def calibrate():
    mission.append({"name": "CALIBRATE", "goal": None})
    update_mission_list()

# Function to handle the "PICKUP_OBJECT" button click
def pickup_object():
    mission.append({"name": "PICKUP_OBJECT", "goal": None})
    update_mission_list()

# Function to handle the "AUTONOMOUS_NAVIGATE" button click
def autonomous_navigate():
    mission.append({"name": "AUTONOMOUS_NAVIGATE", "goal": [9.0, -3.0]})
    update_mission_list()

# Function to handle the "DROP_OBJECT" button click
def drop_object():
    mission.append({"name": "DROP_OBJECT", "goal": None})
    update_mission_list()

# Function to handle the "SET_WAYPOINTS" button click
def set_waypoints():
    saved_coords = open_pygame_window(setwaypoints)
    mission.append({"name": "SET_WAYPOINTS", "goal": saved_coords})
    update_mission_list()

def nav_to_pose():
    saved_coord, saved_angle = open_pygame_window(setnav)
    mission.append({"name": "NAV_TO_POSE", "goal": {"coord": saved_coord, "angle": saved_angle}})
    update_mission_list()

def send_mission():
    # Convert mission to bytes
    mission_data = {
        "mission": mission,
        "map": map
    }
    mission_data = json.dumps(mission_data)
    mission_bytes = mission_data.encode('utf-8')

    for i in range(0, len(mission_bytes), chunk_size):
        chunk = mission_bytes[i:i+chunk_size]
        sock.sendall(chunk)
    sock.sendall(b'\n')

    mission.clear()
    update_mission_list()

def open_pygame_window(pygame_function):
    # Get the position of the Tkinter window
    x, y = window.winfo_x(), window.winfo_y()
    width, height = window.winfo_width(), window.winfo_height()
    
    # Initialize Pygame and set the position of the new window
    pygame.init()

    os.environ['SDL_VIDEO_WINDOW_POS'] = f"{x + width+10},{y+31}"

    # Call the Pygame function and get the result
    result = pygame_function()
    
    pygame.quit()
    return result

# Create a tkinter window
window = tk.Tk()
window.title("Mission Control")
window.geometry('150x400')

# Function to update the mission list in the GUI
def update_mission_list():
    mission_list.delete(0, tk.END)  # Clear the current list
    for item in mission:
        mission_list.insert(tk.END, item["name"])

# Create a listbox to display the mission list
mission_list = tk.Listbox(window)
mission_list.pack()

# Create buttons for different actions
calibrate_button = tk.Button(window, text="CALIBRATE", command=calibrate)
set_waypoint_button = tk.Button(window, text="SET WAYPOINTS", command=set_waypoints)
pickup_button = tk.Button(window, text="PICKUP OBJECT", command=pickup_object)
nav_to_pose_button = tk.Button(window, text="NAV TO POSE", command=nav_to_pose)
# autonomous_button = tk.Button(window, text="AUTONOMOUS NAVIGATE", command=autonomous_navigate)
drop_button = tk.Button(window, text="DROP OBJECT", command=drop_object)
send_mission_button = tk.Button(window, text="SEND MISSION", command=send_mission)

# Pack the buttons into the window
calibrate_button.pack()
set_waypoint_button.pack()
pickup_button.pack()
nav_to_pose_button.pack()
# autonomous_button.pack()
drop_button.pack()
send_mission_button.pack()

# Run the tkinter event loop
window.mainloop()
