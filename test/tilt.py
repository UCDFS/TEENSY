import serial
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import re

# === SETTINGS ===
PORT = "/dev/tty.usbmodem180360401"  # or "COM3" on Windows
BAUD = 115200

# === SERIAL ===
ser = serial.Serial(PORT, BAUD, timeout=0.001)  # Much lower timeout for faster response

# === PYGAME / OPENGL SETUP ===
pygame.init()
display = (800, 600)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
glTranslatef(0.0, 0.0, -5)
# Improve visibility
glEnable(GL_DEPTH_TEST)
glLineWidth(2.0)

# === CUBOID VERTICES ===
vertices = [
    [1, -1, -1],
    [1, 1, -1],
    [-1, 1, -1],
    [-1, -1, -1],
    [1, -1, 1],
    [1, 1, 1],
    [-1, -1, 1],
    [-1, 1, 1]
]

edges = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,7),(7,6),(6,4),
    (0,4),(1,5),(2,7),(3,6)
]

def draw_cube():
    glColor3f(1.0, 1.0, 1.0)
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()

# === MAIN LOOP ===
pitch_angle = 0
roll_angle = 0
# No need for artificial rotation scale - use actual angles
ROTATION_SCALE = 1.0
# Print smoothed angles occasionally for debugging
frame_counter = 0
# Print raw serial lines for debugging when True
DEBUG_SERIAL = False  # Disable to reduce overhead

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            ser.close()
            pygame.quit()
            quit()

    # Read multiple lines per frame to clear any backlog and get latest data
    latest_pitch = None
    latest_roll = None
    lines_read = 0
    
    # Read up to 10 lines per frame to catch up with buffer
    while lines_read < 10:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            break
        lines_read += 1
        
        if DEBUG_SERIAL:
            print("SERIAL:", line)

        # Extract pitch and roll from "TILT pitch=X.XX roll=Y.YY" format
        pitch_match = re.search(r'pitch\s*=\s*([-+]?[0-9]*\.?[0-9]+)', line, re.IGNORECASE)
        roll_match = re.search(r'roll\s*=\s*([-+]?[0-9]*\.?[0-9]+)', line, re.IGNORECASE)
        
        if pitch_match:
            try:
                latest_pitch = float(pitch_match.group(1))
            except ValueError:
                pass
        
        if roll_match:
            try:
                latest_roll = float(roll_match.group(1))
            except ValueError:
                pass
    
    # Only update if we got new data
    if latest_pitch is not None or latest_roll is not None:
        # Much higher alpha for more responsive movement
        alpha = 0.5  # 0..1 where higher is more responsive
        
        if latest_pitch is not None:
            pitch_angle = (1.0 - alpha) * pitch_angle + alpha * latest_pitch
        
        if latest_roll is not None:
            roll_angle = (1.0 - alpha) * roll_angle + alpha * latest_roll
        
        # occasional debug print of smoothed values
        frame_counter += 1
        if DEBUG_SERIAL and (frame_counter % 50 == 0):
            print(f"SMOOTHED - Pitch: {pitch_angle:.2f}° Roll: {roll_angle:.2f}°")

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glPushMatrix()
    # Apply pitch rotation around X axis (forward/backward tilt)
    glRotatef(pitch_angle * ROTATION_SCALE, 1, 0, 0)
    # Apply roll rotation around Z axis (left/right tilt)
    glRotatef(roll_angle * ROTATION_SCALE, 0, 0, 1)
    draw_cube()
    glPopMatrix()

    pygame.display.flip()
    pygame.time.wait(10)  # Faster refresh rate - 100 FPS max
