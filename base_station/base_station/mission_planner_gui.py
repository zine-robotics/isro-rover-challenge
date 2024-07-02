import tkinter as tk
from tkinter import ttk
import socket
import json
import pygame
import os
from pygame_map2 import setwaypoints, get_map, setnav

# Establish socket connection
HOST = 'localhost'  # Replace with the server IP address
PORT = 8000  # Replace with the desired port number

# Create a socket object
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the server
sock.connect((HOST, PORT))

# Placeholder for map fetching
map = get_map()
map = map.tolist()

chunk_size = 1024

mission = []  # List to store mission stages
expanded_stages = {}  # Dictionary to keep track of expanded stages


def add_stage(stage_type):
    if stage_type == "CALIBRATE":
        mission.append({"name": "CALIBRATE", "goal": None})
    elif stage_type == "PICKUP_OBJECT":
        mission.append({"name": "PICKUP_OBJECT", "goal": None})
    elif stage_type == "DROP_OBJECT":
        mission.append({"name": "DROP_OBJECT", "goal": None})
    elif stage_type == "WAYPOINT_FOLLOWER":
        saved_coords = open_pygame_window(setwaypoints)
        mission.append({"name": "WAYPOINT_FOLLOWER", "goal": saved_coords})
    elif stage_type == "NAV_TO_POSE":
        saved_coord, saved_angle = open_pygame_window(setnav)
        mission.append(
            {
                "name": "NAV_TO_POSE",
                "goal": {"coord": saved_coord, "angle": saved_angle},
            }
        )
    expanded_stages[len(mission) - 1] = True  # Expand the new stage by default
    update_mission_list()


def send_mission():
    # Convert mission to bytes
    mission_data = {"mission": mission, "map": map}
    mission_data = json.dumps(mission_data)
    mission_bytes = mission_data.encode("utf-8")

    for i in range(0, len(mission_bytes), chunk_size):
        chunk = mission_bytes[i : i + chunk_size]
        sock.sendall(chunk)
    sock.sendall(b"\n")

    mission.clear()
    update_mission_list()

    window.destroy()  # Close the Tkinter window after sending the mission


def open_pygame_window(pygame_function):
    # Get the position of the Tkinter window
    x, y = window.winfo_x(), window.winfo_y()
    width, height = window.winfo_width(), window.winfo_height()

    # Initialize Pygame and set the position of the new window
    pygame.init()
    os.environ["SDL_VIDEO_WINDOW_POS"] = f"{x + width + 10},{y + 31}"

    # Call the Pygame function and get the result
    result = pygame_function()

    pygame.quit()
    return result


def toggle_stage(index):
    """Toggle the expansion state of the given stage."""
    expanded_stages[index] = not expanded_stages.get(index, False)
    update_mission_list()


def delete_stage(index):
    """Delete the stage at the given index."""
    del mission[index]
    expanded_stages.pop(index, None)
    update_mission_list()


# Create a tkinter window
window = tk.Tk()
window.title("Rover Mission Planner")
window.geometry("300x500")

# Create a frame to hold the mission list
mission_frame = tk.Frame(window)
mission_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10, side=tk.TOP)


# Function to update the mission list in the GUI
def update_mission_list():
    for widget in mission_frame.winfo_children():
        widget.destroy()  # Clear the current list

    for idx, item in enumerate(mission):
        frame = tk.Frame(mission_frame)
        frame.pack(fill=tk.X, pady=2)

        toggle_button = tk.Button(
            frame,
            text="-" if expanded_stages.get(idx, False) else "+",
            command=lambda idx=idx: toggle_stage(idx),
            width=2,
        )
        toggle_button.pack(side=tk.LEFT)

        stage_label = tk.Label(frame, text=item["name"])
        stage_label.pack(side=tk.LEFT, padx=(5, 0))

        delete_button = tk.Button(
            frame,
            text="❌",
            fg="red",
            command=lambda idx=idx: delete_stage(idx),
            width=2,
        )
        delete_button.pack(side=tk.RIGHT)

        # Check if the stage is expanded and if it has sublist items
        if expanded_stages.get(idx, False):
            if item["name"] == "WAYPOINT_FOLLOWER":
                for waypoint in item["goal"]:
                    waypoint_label = tk.Label(
                        mission_frame, text=f"  Point: {waypoint}"
                    )
                    waypoint_label.pack(anchor="w", padx=(20, 0))

            elif item["name"] == "NAV_TO_POSE":
                coord = item["goal"]["coord"]
                angle = item["goal"]["angle"]
                coord_label = tk.Label(mission_frame, text=f"  Coord: {coord}")
                coord_label.pack(anchor="w", padx=(20, 0))
                angle_label = tk.Label(
                    mission_frame, text=f"  Angle: {angle:.2f} radians"
                )
                angle_label.pack(anchor="w", padx=(20, 0))


# Create a label for stage selection
stage_label = tk.Label(window, text="Select Stage:")
stage_label.pack(pady=(10, 0))

# Create a combobox for selecting stages
stage_options = [
    "CALIBRATE",
    "WAYPOINT_FOLLOWER",
    "PICKUP_OBJECT",
    "NAV_TO_POSE",
    "DROP_OBJECT",
]
stage_combobox = ttk.Combobox(window, values=stage_options)
stage_combobox.pack(pady=(0, 10))

# Create a button to add the selected stage to the mission
add_stage_button = tk.Button(
    window, text="Add Stage", command=lambda: add_stage(stage_combobox.get())
)
add_stage_button.pack(pady=5)

# Create a button to send the mission to the rover
send_mission_button = tk.Button(window, text="Send Mission", command=send_mission)
send_mission_button.pack(pady=5)

# Run the tkinter event loop
window.mainloop()
