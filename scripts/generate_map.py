import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
# import scipy.ndimage as ndi # No longer needed as low-cost map generation was removed
import os

# --- Parameters ---
MAP_WIDTH = 500
MAP_HEIGHT = 500
ROBOT_BASE_POS = (MAP_WIDTH // 2, MAP_HEIGHT)  # Center bottom
ROBOT_REACH = 400  # Max reach of the end-effector from the base
OBSTACLE_COST = 1.0 # Value representing obstacles and inaccessible areas
FREE_SPACE_COST = 0.0 # Value representing free, accessible space
# Characters used in the output map file
OBSTACLE_CHAR = '1'
FREE_SPACE_CHAR = '0'


# --- Output Files ---
FINE_MAP_FILE = 'map_accessible_fine.txt'
VIS_FINE_MAP_FILE = 'map_accessible_fine.png'

# --- Functions ---

def create_accessible_fine_map(width, height, base_pos, reach):
    """
    Generates a fine map considering robot accessibility.

    Args:
        width (int): Map width.
        height (int): Map height.
        base_pos (tuple): (x, y) coordinates of the robot base.
        reach (float): Maximum reach of the robot end-effector.

    Returns:
        tuple: (fine_map, is_accessible_mask)
               fine_map: 2D numpy array (0=free, 1=obstacle/inaccessible).
               is_accessible_mask: Boolean mask of accessible cells.
    """
    print("Generating fine map...")
    # Initialize map with free space cost
    fine_map = np.full((height, width), FREE_SPACE_COST, dtype=float)
    y_coords, x_coords = np.indices((height, width))

    # 1. Calculate Workspace Mask
    base_x, base_y = base_pos
    dist_sq = (x_coords - base_x)**2 + (y_coords - base_y)**2
    is_accessible_mask = dist_sq <= reach**2
    print(f"Total accessible cells: {np.sum(is_accessible_mask)}")

    # 2. Add Obstacles (within the potential workspace)
    # Obstacle 1: Large central block
    fine_map[250:350, 150:350] = OBSTACLE_COST
    # Obstacle 2: Smaller block forcing detour
    fine_map[150:200, 300:400] = OBSTACLE_COST

    # 3. Mark Inaccessible Areas on Fine Map
    # Areas outside the robot's reach are marked as obstacles/inaccessible
    fine_map[~is_accessible_mask] = OBSTACLE_COST
    print(f"Obstacle/Inaccessible cells in fine_map: {np.sum(fine_map == OBSTACLE_COST)}")

    print("Fine map generation complete.")
    return fine_map, is_accessible_mask

def visualize_map(map_data, title, filename, base_pos, reach, is_accessible_mask, start=None, goal=None):
    """Visualizes the fine map, workspace, and optional start/goal points."""
    print(f"Visualizing: {title}")
    fig, ax = plt.subplots(figsize=(8, 8))

    # Use a grayscale colormap for the binary fine map
    # 0 (FREE_SPACE_COST) = white, 1 (OBSTACLE_COST) = black
    cmap = plt.cm.Greys
    norm = plt.Normalize(vmin=FREE_SPACE_COST, vmax=OBSTACLE_COST)
    im = ax.imshow(map_data, cmap=cmap, origin='upper', norm=norm,
                   extent=[0, map_data.shape[1], map_data.shape[0], 0]) # Set extent for correct axes

    # Draw workspace boundary
    workspace_circle = patches.Circle(base_pos, reach, linewidth=1.5, edgecolor='cyan', facecolor='none', ls='--')
    ax.add_patch(workspace_circle)
    ax.plot(base_pos[0], base_pos[1], 'c^', markersize=10, label='Robot Base')

    # Plot Start and Goal if provided, checking against the fine map
    if start:
        is_start_valid = (0 <= start[0] < map_data.shape[1] and
                          0 <= start[1] < map_data.shape[0] and
                          is_accessible_mask[start[1], start[0]] and
                          map_data[start[1], start[0]] == FREE_SPACE_COST)
        if is_start_valid:
            ax.plot(start[0], start[1], 'go', markersize=10, label='Start EE')
        else:
            ax.plot(start[0], start[1], 'gx', markersize=10, label='Start EE (Invalid/Obstacle)')

    if goal:
        is_goal_valid = (0 <= goal[0] < map_data.shape[1] and
                         0 <= goal[1] < map_data.shape[0] and
                         is_accessible_mask[goal[1], goal[0]] and
                         map_data[goal[1], goal[0]] == FREE_SPACE_COST)
        if is_goal_valid:
            ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal EE')
        else:
            ax.plot(goal[0], goal[1], 'rx', markersize=10, label='Goal EE (Invalid/Obstacle)')

    ax.set_title(title)
    ax.set_xlabel("Width (pixels)")
    ax.set_ylabel("Height (pixels)")
    ax.set_xlim(0, map_data.shape[1])
    ax.set_ylim(map_data.shape[0], 0) # Invert y-axis to match image coordinates (origin top-left)
    ax.set_aspect('equal', adjustable='box')
    ax.legend()
    plt.grid(False)
    plt.savefig(filename)
    print(f"Saved visualization to {filename}")
    # plt.show() # Comment out if running in a non-interactive environment

def save_map_to_file(map_data, filename):
    """
    Saves the map data to a text file in the format expected by the C++ loadMap function.
    Format:
    height <H>
    width <W>
    <data> (H rows, W columns, space-separated '0' or '1')
    """
    print(f"Saving map to {filename} in C++ compatible format...")
    height, width = map_data.shape

    try:
        with open(filename, 'w') as f:
            # Write header
            f.write(f"height {height}\n")
            f.write(f"width {width}\n")

            # Write map data row by row
            for y in range(height):
                row_chars = []
                for x in range(width):
                    value = map_data[y, x]
                    char_to_write = FREE_SPACE_CHAR if value == FREE_SPACE_COST else OBSTACLE_CHAR
                    row_chars.append(char_to_write)
                # Join characters with spaces and add newline for each row
                f.write(" ".join(row_chars) + "\n")

        print("Save complete.")
    except IOError as e:
        print(f"Error saving map file {filename}: {e}")
        raise # Re-raise the exception

# --- Main Execution ---
if __name__ == "__main__":
    # Create output directory if it doesn't exist
    output_dir = "robot_maps"
    os.makedirs(output_dir, exist_ok=True)

    fine_map_path = os.path.join(output_dir, FINE_MAP_FILE)
    vis_fine_path = os.path.join(output_dir, VIS_FINE_MAP_FILE)

    # Generate the fine map
    fine_map, is_accessible_mask = create_accessible_fine_map(
        MAP_WIDTH, MAP_HEIGHT, ROBOT_BASE_POS, ROBOT_REACH
    )

    # Define Start and Goal points
    start_ee = (100, 300) # (x, y)
    goal_ee = (400, 300)  # (x, y)

    # Check if start/goal are valid based on the fine map
    print(f"Checking Start point {start_ee}:")
    if not (0 <= start_ee[0] < MAP_WIDTH and 0 <= start_ee[1] < MAP_HEIGHT):
         print("  Start is outside map boundaries.")
    elif not is_accessible_mask[start_ee[1], start_ee[0]]:
         print("  Start is outside robot workspace.")
    elif fine_map[start_ee[1], start_ee[0]] == OBSTACLE_COST:
         print("  Start is inside an obstacle or inaccessible area.")
    else:
         print("  Start is valid.")

    print(f"Checking Goal point {goal_ee}:")
    if not (0 <= goal_ee[0] < MAP_WIDTH and 0 <= goal_ee[1] < MAP_HEIGHT):
         print("  Goal is outside map boundaries.")
    elif not is_accessible_mask[goal_ee[1], goal_ee[0]]:
         print("  Goal is outside robot workspace.")
    elif fine_map[goal_ee[1], goal_ee[0]] == OBSTACLE_COST:
         print("  Goal is inside an obstacle or inaccessible area.")
    else:
         print("  Goal is valid.")

    # Visualize the fine map
    visualize_map(fine_map, f'Fine Map ({FINE_MAP_FILE}) with Workspace',
                  vis_fine_path, ROBOT_BASE_POS, ROBOT_REACH, is_accessible_mask, start_ee, goal_ee)

    # Save the fine map to a text file in the specific C++ format
    save_map_to_file(fine_map, fine_map_path)

    print(f"\nScript finished. Check the '{output_dir}' directory for {FINE_MAP_FILE} and {VIS_FINE_MAP_FILE}.")
