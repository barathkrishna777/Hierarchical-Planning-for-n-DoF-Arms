import numpy as np
import io

def generate_vertical_corridor_map(filename="map_vertical_corridor_noborder.txt", height=500, width=500):
    print(f"Generating vertical corridor map ({width}x{height})...")

    new_map = np.zeros((height, width), dtype=int)

    # Define corridor properties
    corridor_center_x = width // 2
    corridor_width = 100
    corridor_half_width = corridor_width // 2

    free_space_margin = 100
    corridor_start_y = free_space_margin
    corridor_end_y = height - free_space_margin

    # Fill left and right blocks (obstacles)
    left_block_end_x = corridor_center_x - corridor_half_width
    right_block_start_x = corridor_center_x + corridor_half_width

    new_map[corridor_start_y:corridor_end_y, 0:left_block_end_x] = 1
    new_map[corridor_start_y:corridor_end_y, right_block_start_x:width] = 1

    print(f"Vertical corridor created from Y={corridor_start_y} to Y={corridor_end_y}.")

    # Add alternating bars across the corridor
    bar_size = 25
    num_bars = 4
    bar_spacing_y = (corridor_end_y - corridor_start_y) // (num_bars + 1)

    for i in range(num_bars):
        bar_center_y = corridor_start_y + (i + 1) * bar_spacing_y
        bar_start_y = bar_center_y - bar_size // 2
        bar_end_y = bar_center_y + bar_size // 2
        bar_x = corridor_center_x + (i % 2 - 0.5) * (corridor_half_width // 2)
        bar_start_x = int(bar_x - bar_size // 2)
        bar_end_x = int(bar_x + bar_size // 2)

        bar_start_x = max(left_block_end_x, bar_start_x)
        bar_end_x = min(right_block_start_x, bar_end_x)

        if bar_start_y < bar_end_y and bar_start_x < bar_end_x:
            new_map[bar_start_y:bar_end_y, bar_start_x:bar_end_x] = 1
            print(f"Added obstacle bar at ({int(bar_x)}, {bar_center_y}).")

    # Format output
    output_string = io.StringIO()
    output_string.write(f"height {height}\n")
    output_string.write(f"width {width}\n")

    for y in range(height):
        row_string = " ".join(map(str, new_map[y, :]))
        output_string.write(row_string + "\n")

    with open(filename, 'w') as f:
        f.write(output_string.getvalue())
    print(f"✅ Map written to '{filename}'")

    # Optional visual preview
    try:
        import matplotlib.pyplot as plt
        plt.figure(figsize=(6, 6))
        plt.imshow(np.flipud(new_map), cmap='gray_r', origin='lower')
        plt.title(f'Preview of {filename}')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()
    except ImportError:
        print("Matplotlib not installed; skipping preview.")

if __name__ == "__main__":
    generate_vertical_corridor_map()
