import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import os
import argparse

# Constants
LINKLENGTH_CELLS = 100.0
PI = np.pi

def load_map_data(filepath):
    """ Loads map data from the C++ compatible text file format. """
    # (Function remains the same as provided in the previous context)
    try:
        with open(filepath, 'r') as f:
            # Read height
            line1 = f.readline()
            if not line1.startswith("height"):
                print(f"Error: Expected 'height' line in {filepath}")
                return None, None, None
            height = int(line1.split()[1])

            # Read width
            line2 = f.readline()
            if not line2.startswith("width"):
                 print(f"Error: Expected 'width' line in {filepath}")
                 return None, None, None
            width = int(line2.split()[1])

            # Read map data
            map_data = []
            for row_idx in range(height): # Iterate using index for better error reporting
                line = f.readline()
                if not line.strip(): # Handle empty lines
                     print(f"Warning: Encountered unexpected empty line at row {row_idx+1} in {filepath}")
                     map_data.append([1.0] * width) # Treat as obstacle
                     continue
                try:
                    row = [float(val) for val in line.split()]
                except ValueError as e:
                    print(f"Error converting value to float in row {row_idx+1} of {filepath}: {e}")
                    return None, None, None
                if len(row) != width: # Ensure row has correct width
                     print(f"Error: Row {row_idx+1} length mismatch in {filepath}. Expected {width}, got {len(row)}")
                     return None, None, None
                map_data.append(row)

            if len(map_data) != height: # Ensure correct number of rows read
                 print(f"Error: Row count mismatch in {filepath}. Expected {height}, got {len(map_data)}")
                 return None, None, None

            return np.array(map_data), width, height
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return None, None, None
    except Exception as e:
        print(f"Error loading map data from {filepath}: {e}")
        return None, None, None


def compute_forward_kinematics(angles_rad, map_width):
    """ Computes end-effector position. Assumes Y=0 at bottom, increasing upwards. """
    # (Function remains the same)
    x1 = float(map_width) / 2.0
    y1 = 0.0 # Base Y coordinate (bottom edge)
    for angle in angles_rad:
        x0 = x1
        y0 = y1
        x1 = x0 + LINKLENGTH_CELLS * np.cos(2 * PI - angle)
        y1 = y0 - LINKLENGTH_CELLS * np.sin(2 * PI - angle) # Y increases upwards
    return x1, y1

def parse_angles(angles_str):
    """ Parses comma-separated angle string into a list of floats. """
    # (Function remains the same)
    try:
        # Filter out empty strings that might result from trailing commas
        return [float(a.strip()) for a in angles_str.split(',') if a.strip()]
    except ValueError:
        print(f"Error: Could not parse angles string: '{angles_str}'. Ensure it's comma-separated numbers.")
        return None

# --- Argument Parsing ---
parser = argparse.ArgumentParser(description="Visualize a fine map with a low-cost map overlay and start/goal points.")
parser.add_argument("fine_map_path", help="Path to the fine map file (.txt)")
parser.add_argument("low_cost_map_path", help="Path to the low-cost map file (.txt)")
parser.add_argument("start_angles", help="Comma-separated start joint angles (radians)")
parser.add_argument("goal_angles", help="Comma-separated goal joint angles (radians)")
args = parser.parse_args()

fine_map_path = args.fine_map_path
low_cost_map_path = args.low_cost_map_path
start_angles_rad = parse_angles(args.start_angles)
goal_angles_rad = parse_angles(args.goal_angles)

if start_angles_rad is None or goal_angles_rad is None: exit(1)

# --- Load Maps ---
print(f"Loading fine map: {fine_map_path}")
fine_map, fine_width, fine_height = load_map_data(fine_map_path)
print(f"Loading low-cost map: {low_cost_map_path}")
low_cost_map, lc_width, lc_height = load_map_data(low_cost_map_path)

# --- Validation ---
if fine_map is None or low_cost_map is None:
    print("Failed to load one or both maps. Exiting.")
    exit(1)
if fine_width != lc_width or fine_height != lc_height:
    print(f"Error: Map dimensions do not match!")
    print(f"Fine map: {fine_width}x{fine_height}")
    print(f"Low-cost map: {lc_width}x{lc_height}")
    exit(1)

# --- Compute FK for Start/Goal ---
start_ee_x, start_ee_y = compute_forward_kinematics(start_angles_rad, fine_width)
goal_ee_x, goal_ee_y = compute_forward_kinematics(goal_angles_rad, fine_width)
print(f"Calculated Start EE: ({start_ee_x:.2f}, {start_ee_y:.2f})")
print(f"Calculated Goal EE: ({goal_ee_x:.2f}, {goal_ee_y:.2f})")

# --- Visualization ---
fig, ax = plt.subplots(figsize=(10, 10 * fine_height / fine_width))

# **Coordinate System Correction:**
# Use origin='lower' for imshow to match the FK coordinate system (Y=0 at bottom).
# Adjust extent accordingly. Remove set_ylim flip.

# 1. Display the fine map (Obstacles=Black, Free=White)
# origin='lower' places (0,0) index of array at bottom-left of plot axes
ax.imshow(fine_map, cmap='gray_r', origin='upper', interpolation='nearest',
          extent=[0, fine_width, 0, fine_height]) # extent y from 0 (bottom) to height (top)

# 2. Create the low-cost overlay
overlay_rgba = np.zeros((fine_height, fine_width, 4))
# Ensure low_cost_map is treated as float/int for comparison
low_cost_indices = np.where(low_cost_map == 0) # Find low-cost cells (value 0)
orange_color = mcolors.to_rgba('orange')
alpha = 0.3
overlay_rgba[low_cost_indices] = list(orange_color[:3]) + [alpha]

# 3. Display the overlay
# Use same extent and origin as the fine map for alignment
ax.imshow(overlay_rgba, origin='lower', interpolation='nearest',
          extent=[0, fine_width, 0, fine_height]) # origin='lower', extent y from 0 to height

# 4. Plot Start and Goal End-Effector Positions
# These coordinates (where Y=0 is bottom) should now align correctly with the axes
ax.plot(start_ee_x, start_ee_y, 'go', markersize=8, label='Start EE', markeredgecolor='black')
ax.plot(goal_ee_x, goal_ee_y, 'ro', markersize=8, label='Goal EE', markeredgecolor='black')

# --- Final Touches ---
fine_map_filename = os.path.basename(fine_map_path)
low_cost_map_filename = os.path.basename(low_cost_map_path)
ax.set_title(f"Fine Map ('{fine_map_filename}') with Low-Cost Corridor ('{low_cost_map_filename}')")
ax.set_xlabel("Width (pixels)")
ax.set_ylabel("Height (pixels)")

# Set plot limits (Y-axis now runs 0 at bottom to height at top)
ax.set_xlim(0, fine_width)
ax.set_ylim(0, fine_height) # Correct ylim for origin='lower'

ax.legend()
ax.set_aspect('equal', adjustable='box')
plt.tight_layout()
plt.show() # Display the plot

print("Visualization complete.")

# Optional: Save the figure
output_filename = "map_visualization.png"
plt.savefig(output_filename)
print(f"Visualization saved to {output_filename}")
