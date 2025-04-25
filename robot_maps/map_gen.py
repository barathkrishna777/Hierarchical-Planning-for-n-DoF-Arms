import numpy as np
import io
import os

def generate_challenging_map(filename="map_challenging.txt", height=500, width=500):
    """
    Generates a map file with obstacles designed to be challenging for planners.

    Args:
        filename (str): The name of the output map file.
        height (int): The height of the map grid.
        width (int): The width of the map grid.
    """
    print(f"Generating challenging map ({width}x{height})...")

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

    # Add a large central block obstacle
    block_start_x = 100
    block_end_x = 400
    block_start_y = 100
    block_end_y = 400
    new_map[block_start_y:block_end_y, block_start_x:block_end_x] = 1
    print("Added central block obstacle.")

    # Create narrow gaps (tunnels) through the central block
    gap_width = 15  # How wide the gap is (adjust for difficulty)
    gap_center_x = width // 2
    gap_center_y = height // 2

    # Gap 1: Top wall opening (horizontal tunnel segment)
    new_map[block_start_y : block_start_y + 5,  # Cut slightly into block
            gap_center_x - gap_width // 2 : gap_center_x + gap_width // 2] = 0

    # Gap 2: Bottom wall opening (horizontal tunnel segment)
    new_map[block_end_y - 5 : block_end_y,      # Cut slightly into block
            gap_center_x - gap_width // 2 : gap_center_x + gap_width // 2] = 0

    # Gap 3: Left wall opening (vertical tunnel segment)
    new_map[gap_center_y - gap_width // 2 : gap_center_y + gap_width // 2,
            block_start_x : block_start_x + 5] = 0 # Cut slightly into block

    # Gap 4: Right wall opening (vertical tunnel segment)
    new_map[gap_center_y - gap_width // 2 : gap_center_y + gap_width // 2,
            block_end_x - 5 : block_end_x] = 0     # Cut slightly into block
    print(f"Carved narrow gaps (width ~{gap_width}).")

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
        print(f"Successfully wrote challenging map to '{filename}'")
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
    # You can change the output filename here if needed
    output_filename = "map_challenging.txt"
    generate_challenging_map(filename=output_filename)

