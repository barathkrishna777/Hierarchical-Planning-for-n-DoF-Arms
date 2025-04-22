import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import re
import sys
import os # Added for file existence check

# ---------- DH Forward Kinematics ----------
# (Copied from previous 3d_vis.py - calculates relative to base origin)
def get_transformation_matrix(theta, d, a, alpha):
    """Calculates the homogenous transformation matrix for DH parameters."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,     d],
        [0,        0,        0,     1]
    ])

def forward_kinematics_ur5(angles, link_length=250):
    """
    Calculates the positions of all joints relative to the base frame origin (0,0,0).
    Returns only the end-effector position.
    """
    d_params = [1.0, 0.0, 0.0, 1.0, 0.0, 1.0]
    alpha_params = [math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2, 0.0]
    a_params = [0.0, link_length, link_length, 0.0, 0.0, 0.0]

    T_cumulative = np.identity(4)

    if len(angles) != 6:
         print(f"Warning: FK Expected 6 angles, got {len(angles)}. Using first 6 if available.")

    for i in range(min(len(angles), 6)):
        T_i = get_transformation_matrix(angles[i], d_params[i], a_params[i], alpha_params[i])
        T_cumulative = T_cumulative @ T_i # Post-multiply

    # Return only the position part of the final transformation matrix (end-effector)
    return T_cumulative[:3, 3]

# ---------- Map Loading ----------
# (Copied from previous 3d_vis.py)
def load_map(filename):
    """Loads map data from the specified file."""
    if not os.path.exists(filename):
        print(f"Error: Map file not found at {filename}")
        sys.exit(1)
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
                     return None, 0, 0 # Handle empty map case
                elif map_data.shape[0] != height or map_data.shape[1] != width:
                     print(f"Warning: Map dimensions mismatch. Header: {width}x{height}, Data: {map_data.shape[1]}x{map_data.shape[0]}")
                     # Attempt to use actual shape if possible
                     height, width = map_data.shape
            except (IndexError, ValueError) as e:
                print(f"Error parsing map header or data in {filename}: {e}")
                sys.exit(1)
            except Exception as e:
                print(f"An unexpected error occurred reading map {filename}: {e}")
                sys.exit(1)
    except IOError as e:
        print(f"Error opening or reading map file {filename}: {e}")
        sys.exit(1)
    return map_data, width, height

# ---------- RRT Data Loading ----------
def load_rrt_data(filename):
    """
    Loads node angles and neighbor relationships from the rrt.txt file.

    Args:
        filename (str): Path to the rrt.txt file.

    Returns:
        tuple: (nodes_angles, nodes_neighbors)
               nodes_angles: dict {node_id: [angle1, angle2,...]}
               nodes_neighbors: dict {node_id: [neighbor_id1, neighbor_id2,...]}
    """
    if not os.path.exists(filename):
        print(f"Error: RRT file not found at {filename}")
        sys.exit(1)

    nodes_angles = {}
    nodes_neighbors = {}
    # Regex to capture ID, Angles, and Neighbors part
    # Makes Neighbors part optional and non-greedy
    node_pattern = re.compile(r"Node ID: (\d+), Angles: ([\d\.\-eE,]+)(?:,Neighbors:(.*))?")

    try:
        with open(filename, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if line.startswith("Node ID:"):
                    match = node_pattern.match(line)
                    if match:
                        try:
                            node_id = int(match.group(1))
                            angles_str = match.group(2).strip().rstrip(',') # Clean up angles string
                            neighbors_str = match.group(3) # Might be None if no Neighbors part

                            # Parse angles
                            angles = [float(a.strip()) for a in angles_str.split(",") if a.strip()]
                            if len(angles) == 6: # Expect 6 DoF
                                nodes_angles[node_id] = angles
                            else:
                                print(f"Warning line {line_num}: Node {node_id} has {len(angles)} angles, expected 6. Skipping node angles.")
                                continue # Skip this node if angles are wrong

                            # Parse neighbors
                            neighbors = []
                            if neighbors_str:
                                neighbors_str = neighbors_str.strip()
                                if neighbors_str: # Check if neighbor string is not empty
                                     neighbors = [int(n.strip()) for n in neighbors_str.split() if n.strip()] # Split by space

                            nodes_neighbors[node_id] = neighbors

                        except ValueError as e:
                            print(f"Warning line {line_num}: Could not parse data for Node ID {match.group(1)}: {line} - {e}")
                        except Exception as e:
                             print(f"Warning line {line_num}: Error processing Node ID {match.group(1)}: {e}")
                    # else:
                    #      print(f"Debug line {line_num}: Did not match Node ID regex: {line}") # Optional debug

                elif line.startswith("path:"):
                    # We don't need the path for this visualization, but good to know it exists
                    pass # Ignore path line for this script

    except IOError as e:
        print(f"Error opening or reading RRT file {filename}: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"An unexpected error occurred reading RRT file {filename}: {e}")
        sys.exit(1)

    print(f"Loaded data for {len(nodes_angles)} nodes from {filename}.")
    return nodes_angles, nodes_neighbors

# ---------- Main Visualization Logic ----------
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python visualize_rrt_workspace.py <mapfile.txt> <rrt_output.txt>")
        sys.exit(1)

    map_filename = sys.argv[1]
    rrt_filename = sys.argv[2]

    # --- Load Data ---
    map_data, map_width, map_height = load_map(map_filename)
    nodes_angles, nodes_neighbors = load_rrt_data(rrt_filename)

    if map_data is None or len(nodes_angles) == 0:
        print("Error: Could not load map or node data. Exiting.")
        sys.exit(1)

    # --- Calculate End-Effector Positions ---
    print("Calculating end-effector positions for all nodes...")
    base_offset = np.array([map_width / 2.0, 0.0, 1.0]) # World offset of base
    ee_positions_world = {} # Dictionary to store {node_id: [x, y, z]}

    valid_nodes_count = 0
    for node_id, angles in nodes_angles.items():
        try:
            ee_pos_relative = forward_kinematics_ur5(angles, link_length=250)
            ee_pos_world = ee_pos_relative + base_offset
            if not (np.any(np.isnan(ee_pos_world)) or np.any(np.isinf(ee_pos_world))):
                ee_positions_world[node_id] = ee_pos_world
                valid_nodes_count += 1
            else:
                 print(f"Warning: Invalid coordinates calculated for node {node_id}. Skipping.")
        except Exception as e:
            print(f"Error calculating FK for node {node_id}: {e}")

    print(f"Calculated valid world EE positions for {valid_nodes_count} nodes.")
    if valid_nodes_count == 0:
        print("No valid node positions to plot. Exiting.")
        sys.exit(1)

    # --- Setup 3D Plot ---
    fig = plt.figure(figsize=(12, 10)) # Slightly larger figure
    ax = fig.add_subplot(111, projection='3d')

    # --- Plot Map Surface ---
    print("Plotting map surface...")
    cmap = ListedColormap(['blue', 'red']) # 0: blue, 1: red
    norm = BoundaryNorm([0, 0.5, 1], cmap.N)
    x_coords = np.arange(map_width)
    y_coords = np.arange(map_height)
    X, Y = np.meshgrid(x_coords, y_coords)
    Z = np.zeros_like(X)
    # Flip map data vertically for correct orientation
    flipped_map_data = np.flipud(map_data)
    facecolors = cmap(norm(flipped_map_data))
    # Use smaller stride for potentially better detail, adjust if too slow
    surf = ax.plot_surface(X, Y, Z, facecolors=facecolors, shade=False,
                           rstride=10, cstride=10, alpha=0.7) # Added some transparency
    print("Map surface plotted.")

    # --- Plot RRT Nodes (End-Effector Positions) ---
    print("Plotting RRT nodes...")
    node_ids = list(ee_positions_world.keys())
    node_coords = np.array(list(ee_positions_world.values()))
    if node_coords.size > 0:
         # Plot all nodes as small gray dots
        ax.scatter(node_coords[:, 0], node_coords[:, 1], node_coords[:, 2],
                   c='gray', marker='.', s=10, alpha=0.6, label='RRT Nodes (EE Pos)')
    print(f"Plotted {len(node_ids)} nodes.")

    # --- Plot RRT Edges ---
    print("Plotting RRT edges...")
    edge_count = 0
    plotted_edges = set() # To avoid plotting edges twice (e.g., A->B and B->A if symmetric)
    for node_id, neighbors in nodes_neighbors.items():
        if node_id not in ee_positions_world:
            continue # Skip if parent node position is invalid

        start_pos = ee_positions_world[node_id]

        for neighbor_id in neighbors:
            if neighbor_id not in ee_positions_world:
                # print(f"Warning: Neighbor {neighbor_id} of node {node_id} not found or invalid. Skipping edge.")
                continue # Skip if neighbor node position is invalid

            # Avoid duplicate edges (optional, assumes undirected graph visualization)
            edge_tuple = tuple(sorted((node_id, neighbor_id)))
            if edge_tuple in plotted_edges:
                continue
            plotted_edges.add(edge_tuple)

            end_pos = ee_positions_world[neighbor_id]

            # Plot line segment for the edge
            ax.plot([start_pos[0], end_pos[0]],
                    [start_pos[1], end_pos[1]],
                    [start_pos[2], end_pos[2]],
                    color='green', linewidth=0.5, alpha=0.5) # Thin, semi-transparent green lines
            edge_count += 1

    print(f"Plotted {edge_count} unique edges.")

    # --- Final Plot Setup ---
    ax.set_xlabel("X World")
    ax.set_ylabel("Y World")
    ax.set_zlabel("Z World")
    ax.set_title("RRT Workspace Visualization (Nodes and Edges)")

    # --- *** Set Fixed Axis Limits *** ---
    print("Setting fixed axis limits...")
    ax.set_xlim(0, 500)
    ax.set_ylim(0, 500)
    ax.set_zlim(0, 350)
    # --- *** End Fixed Axis Limits *** ---


    print(f"Final axis limits set to: X={ax.get_xlim()}, Y={ax.get_ylim()}, Z={ax.get_zlim()}")

    ax.legend()
    # Set view angle
    ax.view_init(elev=45, azim=-45) # Adjust elevation and azimuth as desired
    plt.tight_layout()
    plt.show()

