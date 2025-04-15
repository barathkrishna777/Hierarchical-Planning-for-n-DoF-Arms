# Use 'Agg' for non-interactive backend if running without a display
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os # To check file existence
import re # For parsing the new format

# --- Constants ---
LINK_LENGTH = 100 # Must match LINKLENGTH_CELLS in C++ code

# --- Helper Functions ---

def read_map(mapfile):
    """ Reads the map file.
    Input: mapfile path (string)
    Output: 2D numpy array (int) representing the map, width (int), height (int)
    """
    if not os.path.exists(mapfile):
        raise FileNotFoundError(f"Map file not found: {mapfile}")
    with open(mapfile) as f:
        line = f.readline() # e.g. "height 50"
        if not line.startswith("height"): raise ValueError("Invalid map format: Missing height")
        height = int(line.split(' ')[1])
        line = f.readline() # e.g. "width 50"
        if not line.startswith("width"): raise ValueError("Invalid map format: Missing width")
        width = int(line.split(' ')[1])

        mapdata = np.array([line.rstrip().split(" ") for line in f])

    # Basic validation
    if mapdata.shape[0] != height or mapdata.shape[1] != width:
         print(f"Warning: Map dimensions ({mapdata.shape[0]}x{mapdata.shape[1]}) "
               f"don't match header ({height}x{width}). Check map file format.")
         # Attempt to reshape based on header, might fail if data is wrong
         try:
            mapdata = mapdata.flatten()[:height*width].reshape((height, width))
         except ValueError:
            raise ValueError("Map data size mismatch with header dimensions.")

    mapdata = mapdata.astype(int)
    return mapdata, width, height

def parse_planner_output(output_file):
    """ Reads and parses the planner output file with nodes, neighbors, and optional path.
        Handles specific format for RRT-Connect with tree_A/tree_B markers.
    Input: output_file path (string)
           Expected line format: "Node ID: {id}, Angles: {a1},{a2},...,Neighbors: {n1} {n2} ..."
           For RRT-Connect, expects "tree_A" and "tree_B" lines to delineate nodes.
           Expects optional "path:" marker followed by comma-separated node IDs for the path.
    Output: nodes (dict {node_id: {'angles': [angles], 'tree': 'A'/'B'/None}}),
            edges (list [(node1_id, node2_id)]),
            path (list [node_id]) or None if not found
    """
    nodes = {}
    edges = []
    path = None # Initialize path as None
    parsing_path = False # Flag to indicate if we are reading the path line

    if not os.path.exists(output_file):
        print(f"Warning: Planner output file not found: {output_file}. Cannot visualize graph.")
        return nodes, edges, path

    # Determine planner type from filename for specific handling
    filename_base = os.path.basename(output_file).lower()
    is_rrt_connect = 'rrt_connect' in filename_base

    current_tree_marker = None # To track 'A' or 'B' for RRT-Connect

    # Regex to capture the node data parts
    line_regex = re.compile(r"Node ID:\s*(\d+),\s*Angles:\s*([^,]+(?:,[^,]+)*?),\s*(?:Neighbors:\s*(.*))?$")

    with open(output_file) as f:
        for line_num, line in enumerate(f):
            line = line.strip()
            if not line: continue

            # Check for path marker
            if line.lower() == 'path:':
                parsing_path = True
                print("Found path marker, reading path nodes...")
                continue # Skip marker line

            # If parsing path, read the next line as the path
            if parsing_path:
                try:
                    path = [int(p) for p in line.split(',') if p]
                    print(f"Parsed path with {len(path)} nodes.")
                except ValueError:
                    print(f"Warning: Could not parse path line: {line}")
                    path = None # Reset path if parsing fails
                parsing_path = False # Stop parsing path after this line
                continue # Move to next line after processing path

            # Check for RRT-Connect tree markers (only if not parsing path)
            if is_rrt_connect:
                if line.lower() == 'tree_a':
                    current_tree_marker = 'A'
                    print("Parsing Tree A nodes...")
                    continue # Skip marker line
                elif line.lower() == 'tree_b':
                    current_tree_marker = 'B'
                    print("Parsing Tree B nodes...")
                    continue # Skip marker line

            # Parse node data (only if not parsing path)
            match = line_regex.match(line)
            if not match:
                # Don't warn if it's a tree marker we already handled
                if not (is_rrt_connect and (line.lower() == 'tree_a' or line.lower() == 'tree_b')):
                    print(f"Warning: Skipping malformed line {line_num+1} in {output_file}: {line}")
                continue

            try:
                node_id = int(match.group(1))
                # Split angles, remove trailing comma if present from C++ code
                angles_str = match.group(2).strip()
                if angles_str.endswith(','):
                    angles_str = angles_str[:-1]
                angles = [float(a) for a in angles_str.split(',') if a] # Filter empty strings

                # Store node data including tree marker if applicable
                nodes[node_id] = {'angles': np.array(angles), 'tree': current_tree_marker if is_rrt_connect else None}

                # Process neighbors if they exist
                neighbors_str = match.group(3)
                if neighbors_str:
                    neighbor_ids = [int(n) for n in neighbors_str.split() if n] # Split by space, filter empty
                    for neighbor_id in neighbor_ids:
                        # Add edge.
                        edges.append((node_id, neighbor_id))

            except (ValueError, IndexError) as e:
                print(f"Warning: Error parsing node data on line {line_num+1} in {output_file}: {line} ({e})")
                continue # Skip to next line

    print(f"Parsed {len(nodes)} nodes and {len(edges)} edge relationships from {output_file}")
    if path is None:
        print("Path information not found or failed to parse in the output file.")

    return nodes, edges, path


def calculate_end_effector(angles, base_pos):
    """ Calculates the end effector position for a given configuration.
    Input: angles (numpy array), base_pos (tuple/list [x, y])
    Output: End effector position [x, y] (numpy array)
    """
    x, y = base_pos[0], base_pos[1]
    # Assuming angles[i] is the absolute angle CCW from +X axis for link i
    current_x, current_y = base_pos[0], base_pos[1]
    link_end_x, link_end_y = base_pos[0], base_pos[1] # Track the end of the current link

    for i, angle in enumerate(angles):
        # Calculate end of this specific link based on the angle
        link_end_x = current_x + LINK_LENGTH * np.cos(angle)
        link_end_y = current_y + LINK_LENGTH * np.sin(angle)
        # The start of the next link is the end of this one
        current_x = link_end_x
        current_y = link_end_y

    # The final link_end_x, link_end_y is the end effector position
    return np.array([link_end_x, link_end_y])

# --- Main Visualization Function ---

def visualize_graph(map_file, planner_output_file, output_image="planner_graph.png"):
    """ Creates a static plot of the map, nodes, edges, and highlighted path.
        Colors RRT-Connect trees differently.
    """
    try:
        map_data, map_width, map_height = read_map(map_file)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error reading map file: {e}")
        return

    # Parse the combined planner output file
    nodes, edges, path = parse_planner_output(planner_output_file)

    if not nodes:
        print("No node data found in planner output file to visualize.")
        return

    # Determine if it's RRT-Connect for coloring
    filename_base = os.path.basename(planner_output_file).lower()
    is_rrt_connect = 'rrt_connect' in filename_base

    # --- Plotting ---
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot Map (Obstacles)
    ax.imshow(1 - map_data.T, cmap='gray', origin='lower', extent=[0, map_width, 0, map_height], interpolation='none')

    # Calculate Base Position
    base_pos = [map_width / 2.0, 0]

    # Calculate and Store End Effector Positions
    end_effectors = {}
    print("Calculating end effector positions...")
    for node_id, node_data in nodes.items():
        # Check if node_data is a dictionary (new format) or just angles (old format fallback)
        if isinstance(node_data, dict):
            angles = node_data.get('angles')
            if angles is not None:
                 end_effectors[node_id] = calculate_end_effector(angles, base_pos)
            else:
                 print(f"Warning: Node {node_id} missing 'angles' data.")
        elif isinstance(node_data, (np.ndarray, list)): # Fallback for old format if needed
             print(f"Warning: Node {node_id} using fallback angle format.")
             end_effectors[node_id] = calculate_end_effector(np.array(node_data), base_pos)


    # Define Colors and Styles
    default_node_color = 'blue'
    default_edge_color = 'cyan'
    path_edge_color = 'lime' # Bright green for path
    path_line_width = 1.5
    rrt_connect_colors = {
        'A_node': 'blue', 'B_node': 'red',
        'A_edge': 'blue', 'B_edge': 'red',
        'connect_edge': 'magenta' # Edge connecting the two trees
    }

    # Create a set of edges in the path for quick lookup
    path_edges = set()
    if path and len(path) > 1:
        for i in range(len(path) - 1):
            # Add edge in a consistent order (e.g., sorted tuple)
            path_edges.add(tuple(sorted((path[i], path[i+1]))))
        print(f"Path edges identified: {len(path_edges)}")


    # Plot Edges (Plot non-path edges first, then path edges on top)
    plotted_edges = set() # Track plotted edges to avoid duplicates and overplotting

    if edges:
        print(f"Plotting {len(edges)} edges...")
        # Plot non-path edges first
        for u, v in edges:
            edge_tuple_sorted = tuple(sorted((u, v)))
            # Skip if this edge is part of the path (will be plotted later)
            if edge_tuple_sorted in path_edges:
                continue
            # Skip if edge already plotted (handles duplicates/reverse)
            if edge_tuple_sorted in plotted_edges:
                continue

            if u in end_effectors and v in end_effectors:
                pos_u = end_effectors[u]
                pos_v = end_effectors[v]

                edge_color = default_edge_color
                line_width = 0.5
                alpha = 0.6 # Make non-path edges slightly more transparent

                # Determine edge color for RRT-Connect (only if not on path)
                if is_rrt_connect and u in nodes and v in nodes:
                    # Ensure node data is dictionary before accessing 'tree'
                    tree_u = nodes[u]['tree'] if isinstance(nodes[u], dict) else None
                    tree_v = nodes[v]['tree'] if isinstance(nodes[v], dict) else None

                    if tree_u == 'A' and tree_v == 'A':
                        edge_color = rrt_connect_colors['A_edge']
                    elif tree_u == 'B' and tree_v == 'B':
                        edge_color = rrt_connect_colors['B_edge']
                    elif tree_u != tree_v and tree_u is not None and tree_v is not None:
                        edge_color = rrt_connect_colors['connect_edge']
                        line_width = 0.8
                        alpha = 0.8
                    # else: use default color

                ax.plot([pos_u[0], pos_v[0]], [pos_u[1], pos_v[1]], color=edge_color, linestyle='-', linewidth=line_width, alpha=alpha)
                plotted_edges.add(edge_tuple_sorted)
            else:
                 print(f"Warning: Skipping non-path edge ({u}, {v}) due to missing node data for plotting.")

        # Plot path edges on top
        if path_edges:
            print("Plotting path edges...")
            path_plotted = False
            for u_path, v_path in path_edges:
                 if u_path in end_effectors and v_path in end_effectors:
                     pos_u = end_effectors[u_path]
                     pos_v = end_effectors[v_path]
                     # Use path style, add label only once
                     ax.plot([pos_u[0], pos_v[0]], [pos_u[1], pos_v[1]],
                             color=path_edge_color, linestyle='-', linewidth=path_line_width, alpha=1.0,
                             label='Final Path' if not path_plotted else "")
                     path_plotted = True
                 else:
                     print(f"Warning: Skipping path edge ({u_path}, {v_path}) due to missing node data for plotting.")


    # Plot Nodes (as end-effector positions)
    print(f"Plotting {len(end_effectors)} nodes...")
    if end_effectors:
        # Separate nodes by tree for RRT-Connect legend
        nodes_a_x, nodes_a_y = [], []
        nodes_b_x, nodes_b_y = [], []
        nodes_other_x, nodes_other_y = [], []

        for node_id, pos in end_effectors.items():
             # Ensure node data is dictionary before accessing 'tree'
            tree_type = nodes[node_id]['tree'] if node_id in nodes and isinstance(nodes[node_id], dict) else None

            if is_rrt_connect and tree_type == 'A':
                nodes_a_x.append(pos[0])
                nodes_a_y.append(pos[1])
            elif is_rrt_connect and tree_type == 'B':
                nodes_b_x.append(pos[0])
                nodes_b_y.append(pos[1])
            else:
                nodes_other_x.append(pos[0])
                nodes_other_y.append(pos[1])

        node_marker_size = 2
        node_alpha = 0.8
        # Plot non-RRT-Connect or unclassified nodes
        if nodes_other_x:
             ax.plot(nodes_other_x, nodes_other_y, '.', color=default_node_color, markersize=node_marker_size, alpha=node_alpha, label='Nodes (Other/Default)')
        # Plot RRT-Connect Tree A nodes
        if nodes_a_x:
             ax.plot(nodes_a_x, nodes_a_y, '.', color=rrt_connect_colors['A_node'], markersize=node_marker_size, alpha=node_alpha, label='Nodes (Tree A)')
        # Plot RRT-Connect Tree B nodes
        if nodes_b_x:
             ax.plot(nodes_b_x, nodes_b_y, '.', color=rrt_connect_colors['B_node'], markersize=node_marker_size, alpha=node_alpha, label='Nodes (Tree B)')

    else:
        print("No valid end effectors calculated.")


    # Highlight Start/Goal Nodes from Path if available
    start_node_id_path = path[0] if path else None
    goal_node_id_path = path[-1] if path else None

    # Use path start/goal if available, otherwise fall back to defaults
    start_node_id = start_node_id_path if start_node_id_path is not None else 0
    goal_node_id = goal_node_id_path if goal_node_id_path is not None else (0 if is_rrt_connect else None) # Default goal for RRT-Connect is 0 (in tree B)

    start_node_label = 'Start Node'
    goal_node_label = 'Goal Node'

    # Highlight Start Node
    if start_node_id in end_effectors:
        start_color = 'lime' # Brighter green
        # Add tree info to label if RRT-Connect
        if is_rrt_connect and start_node_id in nodes and isinstance(nodes[start_node_id], dict) and nodes[start_node_id]['tree'] == 'A':
            start_node_label += ' (Tree A)'

        ax.plot(end_effectors[start_node_id][0], end_effectors[start_node_id][1],
                'o', color=start_color, markersize=8, markeredgecolor='black', label=start_node_label)

    # Highlight Goal Node
    if goal_node_id is not None and goal_node_id in end_effectors:
        goal_color = 'deeppink'
         # Add tree info to label if RRT-Connect
        if is_rrt_connect and goal_node_id in nodes and isinstance(nodes[goal_node_id], dict) and nodes[goal_node_id]['tree'] == 'B':
             goal_node_label += ' (Tree B)'

        ax.plot(end_effectors[goal_node_id][0], end_effectors[goal_node_id][1],
                'o', color=goal_color, markersize=8, markeredgecolor='black', label=goal_node_label)


    # --- Final Plot Settings ---
    ax.set_title(f"Planner Graph/Tree Visualization ({os.path.basename(map_file)})")
    ax.set_xlabel("X-coordinate")
    ax.set_ylabel("Y-coordinate")
    ax.set_xlim(0, map_width)
    ax.set_ylim(0, map_height)
    ax.set_aspect('equal', adjustable='box') # Ensure correct aspect ratio
    # Collect labels and handles for the legend, avoid duplicates
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles)) # Dictionary to remove duplicate labels
    ax.legend(by_label.values(), by_label.keys(), loc='best', fontsize='small') # Use 'best' or adjust position
    ax.grid(True)

    plt.tight_layout()
    plt.savefig(output_image)
    print(f"Graph visualization saved to {output_image}")
    # plt.show() # Uncomment to display interactively if not using 'Agg'

# --- Main Execution ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize planner graph/tree and path from C++ output file.")
    parser.add_argument("map_file", help="Path to the workspace map file (e.g., map1.txt)")
    parser.add_argument("planner_output_file", help="Path to the planner output file (e.g., rrt_connect.txt, prm.txt)")
    parser.add_argument("--output", help="Output image file name", default="planner_graph.png")
    args = parser.parse_args()

    visualize_graph(args.map_file, args.planner_output_file, args.output)
