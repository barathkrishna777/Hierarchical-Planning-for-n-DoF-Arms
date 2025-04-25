#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import argparse

LINKLENGTH_CELLS = 100.0
PI = np.pi

def read_map(mapfile):
    """Reads:
       height H
       width  W
       then H lines of W ints.
    Returns (map_array, W, H)."""
    with open(mapfile) as f:
        H = int(f.readline().split()[1])
        W = int(f.readline().split()[1])
        data = [list(map(int, f.readline().split())) for _ in range(H)]
    return np.array(data), W, H

def fk_end_effector(angles, W):
    """Compute EE: base at (W/2, 0), Y up."""
    x, y = W/2.0, 0.0
    for θ in angles:
        x += LINKLENGTH_CELLS * np.cos(2*PI - θ)
        y -= LINKLENGTH_CELLS * np.sin(2*PI - θ)
    return x, y

def parse_angles(s):
    return [float(p) for p in s.split(',') if p.strip()]

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("fine_map")
    p.add_argument("low_cost_map")
    p.add_argument("start_angles")
    p.add_argument("goal_angles")
    args = p.parse_args()

    # ─── load your two maps ───────────────────────────────────────────────────
    fine, W, H   = read_map(args.fine_map)
    lowc, W2, H2 = read_map(args.low_cost_map)
    if (W, H) != (W2, H2):
        raise ValueError("Map dimensions mismatch")

    # ─── compute EE in the *un-rotated* frame ────────────────────────────────
    start = parse_angles(args.start_angles)
    goal  = parse_angles(args.goal_angles)
    sx, sy = fk_end_effector(start, W)
    gx, gy = fk_end_effector(goal,  W)

    # ─── rotate the *fine* map CCW 90° so your base is on the top edge: ─────
    fine = np.rot90(fine, k=1)
    # the low-cost corridor stays in its native orientation:
    lowc = np.rot90(lowc, k=0)

    # swap W/H now that fine has been turned
    W, H = H, W

    # ─── **only** flip the EE y-coords to match an “upper” origin ───────────
    sx_r, sy_r = sx, (H - sy)
    gx_r, gy_r = gx, (H - gy)

    # ─── draw ────────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 8*H/W))

    # fine map with origin at lower-left
    ax.imshow(
        fine,
        cmap='gray_r',
        origin='lower',
        extent=[0, W, 0, H],
        interpolation='nearest'
    )

    # low-cost overlay with its origin at upper-left
    overlay = np.zeros((H, W, 4))
    ys, xs = np.where(lowc == 0)
    orange = mcolors.to_rgba('orange', alpha=0.3)
    overlay[ys, xs] = orange
    ax.imshow(
        overlay,
        origin='upper',
        extent=[0, W, 0, H],
        interpolation='nearest'
    )

    # plot the EE’s (no rotation, just flipped y)
    ax.plot(sx_r, sy_r, 'go', ms=8, mec='k', label='Start EE')
    ax.plot(gx_r, gy_r, 'ro', ms=8, mec='k', label='Goal EE')

    # move X axis to top, Y on left
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.xaxis.set_ticks_position('top')
    ax.xaxis.set_label_position('top')
    ax.spines['bottom'].set_visible(False)
    ax.yaxis.set_ticks_position('left')
    ax.spines['right'].set_visible(False)

    ax.set_aspect('equal', 'box')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend(loc='upper right')
    ax.set_title("Rotated CCW: Fine Map + Low-Cost Corridor")

    plt.tight_layout()
    plt.show()
