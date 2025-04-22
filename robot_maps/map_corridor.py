import numpy as np
import io
import os

def generate_corridor_map(filename="map_corridor.txt", height=500, width=500):
    """
    Generates a map file with a challenging narrow corridor between two
    large obstacle regions.

    Args:
        filename (str): The name of the output map file.
        height (int): The height of the map grid.
        width (int): The width of the map grid.
    """
    print(f"Generating corridor map ({width}x{height})...")

    # --- Create the map data ---
    # Start with all free space (0)
    new_map = np.zeros((height, width), dtype=int)

    # Add an outer border obstacle (thickness = 10)
    border_thickness = 10
    new_map[0:border_thickness, :] = 1  # Top
    new_map[height - border_thickness:height, :] = 1  # Bottom
    new_map[:, 0:border_thickness] = 1  # Left
    new_map[:, width - border_thickness:width] = 1  # Right
    print("Added border obstacles.")

    # Define corridor parameters
    corridor_center_y = height // 2
    corridor_width = 20  # Width of the horizontal corridor (adjust for difficulty)
    corridor_half_width = corridor_width // 2

    # Create top obstacle block (above the corridor)
    top_block_start_y = border_thickness
    top_block_end_y = corridor_center_y - corridor_half_width
    new_map[top_block_start_y : top_block_end_y, border_thickness : width - border_thickness] = 1
    print(f"Added top obstacle block (Y: {top_block_start_y} to {top_block_end_y}).")


    # Create bottom obstacle block (below the corridor)
    bottom_block_start_y = corridor_center_y + corridor_half_width
    bottom_block_end_y = height - border_thickness
    new_map[bottom_block_start_y : bottom_block_end_y, border_thickness : width - border_thickness] = 1
    print(f"Added bottom obstacle block (Y: {bottom_block_start_y} to {bottom_block_end_y}).")

    # Optional: Add some small pillar obstacles within the corridor
    pillar_size = 5
    num_pillars = 3
    pillar_spacing_x = (width - 2 * border_thickness) // (num_pillars + 1)

    for i in range(num_pillars):
        pillar_center_x = border_thickness + (i + 1) * pillar_spacing_x
        pillar_start_x = pillar_center_x - pillar_size // 2
        pillar_end_x = pillar_center_x + pillar_size // 2
        # Place pillar slightly offset vertically within corridor
        pillar_y = corridor_center_y + (i % 2 - 0.5) * (corridor_half_width // 2) # Alternate offset
        pillar_start_y = int(pillar_y - pillar_size // 2)
        pillar_end_y = int(pillar_y + pillar_size // 2)

        # Ensure pillars are within corridor bounds
        pillar_start_y = max(top_block_end_y, pillar_start_y)
        pillar_end_y = min(bottom_block_start_y, pillar_end_y)

        if pillar_start_y < pillar_end_y and pillar_start_x < pillar_end_x:
             new_map[pillar_start_y:pillar_end_y, pillar_start_x:pillar_end_x] = 1
             print(f"Added pillar obstacle near ({pillar_center_x}, {int(pillar_y)}).")


    print(f"Created corridor (Y: {top_block_end_y} to {bottom_block_start_y}, Width: {corridor_width}).")

    # --- Format the map data into the required string format ---
    print("Formatting map data for file...")
    output_string = io.StringIO()
    output_string.write(f"height {height}\n")
    output_string.write(f"width {width}\n")

    for y in range(height):
        # Convert row integers to strings and join with spaces
        row_string = " ".join(map(str, new_map[y, :]))
        output_string.write(row_string + "\n")

    # Get the full string value
    map_file_content = output_string.getvalue()

    # Close the StringIO object
    output_string.close()

    # --- Write the formatted string to the specified file ---
    try:
        with open(filename, 'w') as f:
            f.write(map_file_content)
        print(f"Successfully wrote corridor map to '{filename}'")
        # Optional: Display a preview using matplotlib if available
        try:
            import matplotlib.pyplot as plt
            plt.figure(figsize=(6, 6))
            # Use flipud to match the visual orientation discussed previously
            plt.imshow(np.flipud(new_map), cmap='gray_r', origin='lower')
            plt.title(f'Preview of {filename}')
            plt.xlabel('X-coordinate')
            plt.ylabel('Y-coordinate')
            plt.show()
        except ImportError:
            print("Matplotlib not found, skipping preview.")

    except IOError as e:
        print(f"Error writing map file '{filename}': {e}")

# --- Main execution block ---
if __name__ == "__main__":
    # Define the output filename
    output_filename = "map_corridor.txt"
    generate_corridor_map(filename=output_filename)
