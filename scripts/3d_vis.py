import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import re
import sys

# ---------- DH Forward Kinematics ----------
# (No changes needed in this section)
def get_transformation_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,     d],
        [0,        0,        0,     1]
    ])

def forward_kinematics_ur5(angles, link_length=250):
    d = [1.0, 0.0, 0.0, 1.0, 0.0, 1.0]
    alpha = [math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2, 0.0]
    a = [0.0, link_length, link_length, 0.0, 0.0, 0.0]
    T = np.identity(4)
    positions = [T[:3, 3]]
    if len(angles) != 6:
         print(f"Warning: Expected 6 angles, got {len(angles)}. Using first 6 if available.")
    for i in range(min(len(angles), 6)):
        current_d = d[i]
        T_i = get_transformation_matrix(angles[i], current_d, a[i], alpha[i])
        T = T @ T_i
        positions.append(T[:3, 3])
    return np.array(positions)

# ---------- Map and Planner Parsing ----------
# (No changes needed in this section)
def load_map(filename):
    try:
        with open(filename, 'r') as f:
            try:
                height_line = f.readline()
                width_line = f.readline()
                height = int(height_line.split()[1])
                width = int(width_line.split()[1])
                map_data = np.array([[int(x) for x in line.split()] for line in f if line.strip()])
                if map_data.size == 0:
                     print(f"Warning: Map data is empty in {filename}")
                elif map_data.shape[0] != height or map_data.shape[1] != width:
                     print(f"Warning: Map dimensions mismatch. Header: {width}x{height}, Data: {map_data.shape[1]}x{map_data.shape[0]}")
            except (IndexError, ValueError) as e:
                print(f"Error parsing map header or data in {filename}: {e}")
                sys.exit(1)
            except Exception as e:
                print(f"An unexpected error occurred reading map {filename}: {e}")
                sys.exit(1)
    except FileNotFoundError:
        print(f"Error: Map file not found at {filename}")
        sys.exit(1)
    return map_data, width, height

def load_planner_data(filename):
    nodes = {}
    path_indices = []
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
        path_line_found_at = -1
        for line_num, line in enumerate(lines):
            line = line.strip()
            if line.startswith("Node ID:"):
                match = re.match(r"Node ID: (\d+), Angles: ([\d\.\-eE,]+)", line)
                if match:
                    try:
                        node_id = int(match.group(1))
                        angles_str = match.group(2).split(",Neighbors")[0]
                        angles = [float(a.strip()) for a in angles_str.split(",") if a.strip()]
                        if len(angles) == 6:
                            nodes[node_id] = angles
                        else:
                            print(f"Warning line {line_num + 1}: Node {node_id} has {len(angles)} angles, expected 6. Skipping node.")
                    except ValueError as e:
                        print(f"Warning line {line_num + 1}: Could not parse angles for Node ID {match.group(1)}: {match.group(2)} - {e}")
                    except Exception as e:
                         print(f"Warning line {line_num + 1}: Error processing Node ID {match.group(1)}: {e}")
            elif line.startswith("path:"):
                path_line_found_at = line_num
                path_str_same_line = line.split("path:")[1].strip()
                if path_str_same_line:
                     print(f"Info line {line_num + 1}: Found path data on the same line as 'path:'.")
                     path_str = path_str_same_line
                     try:
                         path_indices = [int(x.strip()) for x in path_str.split(",") if x.strip()]
                         break
                     except ValueError as e:
                         print(f"Warning line {line_num + 1}: Could not parse path indices from same line: {path_str} - {e}")
                         path_indices = []
                else:
                    print(f"Info line {line_num + 1}: Found 'path:' keyword. Checking subsequent line(s) for indices.")
        if path_line_found_at != -1 and not path_indices:
             indices_line_index = path_line_found_at + 1
             if indices_line_index < len(lines):
                 path_str = lines[indices_line_index].strip()
                 print(f"Info line {indices_line_index + 1}: Reading path indices from line: '{path_str}'")
                 if path_str:
                     try:
                         path_indices = [int(x.strip()) for x in path_str.split(",") if x.strip()]
                     except ValueError as e:
                         print(f"Warning line {indices_line_index + 1}: Could not parse path indices from next line: {path_str} - {e}")
                         path_indices = []
                 else:
                      print(f"Warning line {indices_line_index + 1}: Line after 'path:' is also empty.")
             else:
                 print(f"Warning: Found 'path:' on last line ({path_line_found_at + 1}), but no subsequent line exists.")
        if path_line_found_at == -1:
             print("Warning: 'path:' line keyword not found anywhere in planner file.")
    except FileNotFoundError:
        print(f"Error: Planner file not found at {filename}")
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred reading planner file {filename}: {e}")
        sys.exit(1)
    if not path_indices and path_line_found_at != -1:
        print("Error: Found 'path:' keyword but failed to parse indices from subsequent line(s).")
    return nodes, path_indices


# ---------- Main (Map Y-Flip, Link Outline/Transparency) ----------

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python 3d_vis.py <mapfile.txt> <planner_output.txt>")
        sys.exit(1)

    map_filename = sys.argv[1]
    planner_filename = sys.argv[2]

    map_data, map_width, map_height = load_map(map_filename)
    nodes_data, path_indices = load_planner_data(planner_filename)

    print(f"Map loaded: {map_width}x{map_height}")
    print(f"Nodes loaded: {len(nodes_data)}")
    print(f"Path indices found: {path_indices}")

    base_offset = np.array([map_width / 2.0, 0.0, 1.0])
    print(f"Applying base offset: {base_offset}")

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot Map using plot_surface
    if map_data is not None and map_data.size > 0:
        cmap = ListedColormap(['blue', 'red']) # 0: blue, 1: red
        norm = BoundaryNorm([0, 0.5, 1], cmap.N)
        x_coords = np.arange(map_width)
        y_coords = np.arange(map_height)
        X, Y = np.meshgrid(x_coords, y_coords)
        Z = np.zeros_like(X)

        # --- *** FLIP map_data vertically before generating colors *** ---
        flipped_map_data = np.flipud(map_data)
        facecolors = cmap(norm(flipped_map_data))
        # --- *** End Map Flip *** ---

        print("Plotting map surface...")
        surf = ax.plot_surface(X, Y, Z, facecolors=facecolors, shade=False,
                               rstride=5, cstride=5) # Adjust stride as needed
        print("Map surface plotted.")
    else:
         print("Map data invalid or empty, skipping map plot.")

    # Variables for Start/End EE positions
    start_ee_pos = None
    last_ee_pos = None
    plotted_something = False
    valid_poses_count = 0

    # Plotting Loop
    for i, node_index in enumerate(path_indices):
        if node_index in nodes_data:
            angles = nodes_data[node_index]
            positions_relative = forward_kinematics_ur5(angles, link_length=250)
            positions_world = positions_relative + base_offset

            if np.any(np.isnan(positions_world)) or np.any(np.isinf(positions_world)):
                print(f"Warning: Invalid coordinates calculated for node {node_index}. Skipping plot.")
                continue

            # --- *** Plot links with outline and transparency *** ---
            # Plot black outline (thicker)
            ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2],
                    linewidth=5, color='black', zorder=9)
            # Plot cyan link (slightly thinner, transparent)
            ax.plot(positions_world[:, 0], positions_world[:, 1], positions_world[:, 2],
                    linewidth=4, color='cyan', alpha=0.6, zorder=10) # << ADJUSTED
            # --- *** End link plotting change *** ---

            # Plot End-Effector Trail
            ee_pos = positions_world[-1]
            ax.scatter(ee_pos[0], ee_pos[1], ee_pos[2],
                       color='yellow', marker='.', s=30, zorder=20)

            # Store start and last positions
            if start_ee_pos is None:
                start_ee_pos = ee_pos
            last_ee_pos = ee_pos

            plotted_something = True
            valid_poses_count += 1
        else:
            print(f"Warning: Node ID {node_index} from path not found in nodes_data.")

    # Plot Start/End Markers After Loop
    if start_ee_pos is not None:
        ax.scatter(start_ee_pos[0], start_ee_pos[1], start_ee_pos[2],
                   color='lime', marker='X', s=150, depthshade=False,
                   zorder=25, label='Start EE')
        print(f"Start EE Position: {start_ee_pos}")

    if last_ee_pos is not None:
        ax.scatter(last_ee_pos[0], last_ee_pos[1], last_ee_pos[2],
                   color='black', marker='*', s=200, depthshade=False,
                   zorder=25, label='End EE')
        print(f"End EE Position: {last_ee_pos}")

    # Add Legend
    if start_ee_pos is not None or last_ee_pos is not None:
        ax.legend()

    # Status messages
    if not plotted_something and path_indices:
        print("Warning: Path indices found, but no valid poses were plotted.")
    elif not path_indices:
        print("Warning: No path indices found to plot.")
    else:
         print(f"Plotted {valid_poses_count} poses.")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("UR5-style Arm Path Visualization")

    # Axis limits
    buffer = 50
    ax.set_xlim(0 - buffer, map_width + buffer)
    ax.set_ylim(0 - buffer, map_height + buffer)
    max_reach_z = 4 * 100
    ax.set_zlim(-1, base_offset[2] + max_reach_z + buffer)
    print(f"Axis limits set to: X={ax.get_xlim()}, Y={ax.get_ylim()}, Z={ax.get_zlim()}")

    ax.view_init(elev=30, azim=45)
    plt.tight_layout()
    plt.show()