import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.animation import FuncAnimation, PillowWriter
import argparse

LINKLENGTH_CELLS = 100.0
PI = np.pi

def read_map(mapfile):
    # Reads “height H” then “width W” then H lines of W ints
    with open(mapfile) as f:
        H = int(f.readline().split()[1])
        W = int(f.readline().split()[1])
        data = [list(map(int, f.readline().split())) for _ in range(H)]
    return np.array(data), W, H

def parse_path_file(pathfile):
    # First line is header, ignore
    poses = []
    with open(pathfile) as f:
        f.readline()
        for line in f:
            line = line.strip()
            if not line: continue
            vals = line.split(",")
            # drop empty after trailing comma
            angles = [float(x) for x in vals if x]
            poses.append(angles)
    return np.array(poses)

def fk_links(angles, W_orig):
    # Forward kinematics in original map frame: base at (W_orig/2, 0), y up
    x = W_orig / 2.0
    y = 0.0
    xs = [x]
    ys = [y]
    for theta in angles:
        x += LINKLENGTH_CELLS * np.cos(2 * PI - theta)
        y -= LINKLENGTH_CELLS * np.sin(2 * PI - theta)
        xs.append(x)
        ys.append(y)
    return np.array(xs), np.array(ys)

def create_frame(frame_idx, fine, lowc, poses, W_orig):
    plt.clf()
    H, W = fine.shape
    
    # Draw fine map (rotated)
    plt.imshow(fine, cmap='gray_r', origin='lower', extent=[0, W, 0, H])
    
    # Overlay low-cost region
    overlay = np.zeros((H, W, 4))
    ys, xs = np.where(lowc == 0)
    orange = mcolors.to_rgba('orange', alpha=0.3)
    overlay[ys, xs] = orange
    plt.imshow(overlay, origin='upper', extent=[0, W, 0, H])
    
    # Robot links for this frame, using original width
    angles = poses[frame_idx]
    xs_link, ys_link = fk_links(angles, W_orig)
    # Transform to rotated map coords: x stays, y flips
    ys_plot = H - ys_link
    xs_plot = xs_link
    plt.plot(xs_plot, ys_plot, '-o', color='g', linewidth=2, markersize=4)
    
    # Static start and goal markers
    # compute once outside, but here we mark at first and last
    start_xs, start_ys = fk_links(poses[0], W_orig)
    goal_xs, goal_ys   = fk_links(poses[-1], W_orig)
    sx_plot = start_xs[-1]
    sy_plot = H - start_ys[-1]
    gx_plot = goal_xs[-1]
    gy_plot = H - goal_ys[-1]
    plt.plot(sx_plot, sy_plot, 's', color='blue', markersize=8, label='Start EE')
    plt.plot(gx_plot, gy_plot, 's', color='magenta', markersize=8, label='Goal EE')
    
    plt.xlim(0, W)
    plt.ylim(0, H)
    plt.axis('off')
    return []

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("fine_map", help="Fine map file")
    parser.add_argument("low_cost_map", help="Low-cost map file")
    parser.add_argument("path_file", help="Path TXT file")
    parser.add_argument("--gif", default="out.gif", help="Output GIF")
    parser.add_argument("--fps", type=int, default=4, help="FPS")
    args = parser.parse_args()
    
    # Load maps
    fine, W_orig, H_orig = read_map(args.fine_map)
    lowc, W2, H2 = read_map(args.low_cost_map)
    if (W_orig, H_orig) != (W2, H2):
        raise ValueError("Map size mismatch")
    
    # Rotate fine map 90° CCW; lowc stays unrotated
    fine = np.rot90(fine, k=1)
    lowc = np.rot90(lowc, k=0)
    # After rotation, new dims:
    W_rot, H_rot = H_orig, W_orig

    # Parse path and poses
    poses = parse_path_file(args.path_file)
    
    # Setup animation
    fig = plt.figure()
    ani = FuncAnimation(
        fig,
        create_frame,
        frames=len(poses),
        fargs=(fine, lowc, poses, W_orig),
        repeat=False
    )
    # Save and show
    ani.save(args.gif, dpi=200, writer=PillowWriter(fps=args.fps))
    print(f"Animation saved: {args.gif}")
    plt.show()

if __name__ == "__main__":
    main()