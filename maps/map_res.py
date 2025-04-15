import os

def refine_map(input_filename, output_filename, factor=10):
    """
    Reads an occupancy grid map from input_filename, where the first two lines are a header:
       height <N>
       width <M>
    followed by rows of space‐separated occupancy values.
    It outputs a new file with each cell replicated into a factor×factor block,
    and updates the header to reflect the new dimensions.
    """
    with open(input_filename, 'r') as f:
        lines = f.read().splitlines()
    
    # Assume the first two nonempty lines are the header lines for height and width.
    header_lines = []
    occupancy_start = 0
    for i, line in enumerate(lines):
        if line.strip() == "":
            continue
        # We assume header lines start with "height" or "width"
        if line.startswith("height") or line.startswith("width"):
            header_lines.append(line)
        else:
            occupancy_start = i
            break

    # Parse original dimensions from the header
    orig_height, orig_width = None, None
    for line in header_lines:
        parts = line.split()
        if parts[0].lower() == "height":
            orig_height = int(parts[1])
        elif parts[0].lower() == "width":
            orig_width = int(parts[1])
    
    if orig_height is None or orig_width is None:
        raise ValueError(f"Could not find valid header dimensions in {input_filename}")

    # Calculate new dimensions
    new_height = orig_height * factor
    new_width = orig_width * factor

    # Process occupancy rows
    occupancy_lines = lines[occupancy_start:]
    refined_rows = []
    for row in occupancy_lines:
        row = row.strip()
        if row == "":
            continue  # skip any empty lines
        tokens = row.split()
        # For each token, replicate it 'factor' times horizontally
        new_row_tokens = []
        for token in tokens:
            new_row_tokens.extend([token] * factor)
        new_row_str = " ".join(new_row_tokens)
        # Replicate the full new row 'factor' times vertically
        for _ in range(factor):
            refined_rows.append(new_row_str)

    # Write the refined map to the output file with updated header.
    with open(output_filename, 'w') as out:
        out.write(f"height {new_height}\n")
        out.write(f"width {new_width}\n")
        for r in refined_rows:
            out.write(r + "\n")
    print(f"Refined map saved to {output_filename}")

# List of map files to refine
input_files = ["map1.txt", "map2.txt", "map3.txt", "map4.txt"]

# Process each file and output a new file (e.g., map1_fine.txt)
for infile in input_files:
    # Create output file name by adding _fine before the .txt extension
    if infile.endswith(".txt"):
        outfile = infile[:-4] + "_fine.txt"
    else:
        outfile = infile + "_fine.txt"
    refine_map(infile, outfile)
