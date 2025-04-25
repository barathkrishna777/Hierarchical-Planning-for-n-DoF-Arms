#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import argparse, re

LINK_LENGTH = 100.0
PI = np.pi

def read_map(mapfile):
    """Reads height H, width W, then H×W ints -> (array, W, H)."""
    with open(mapfile) as f:
        H = int(f.readline().split()[1])
        W = int(f.readline().split()[1])
        data = [list(map(int, f.readline().split())) for _ in range(H)]
    return np.array(data), W, H

def parse_planner_output(fname):
    nodes, edges, path = {}, [], None
    pat = re.compile(r"Node ID:\s*(\d+),.*Angles:\s*([^,]+).*Neighbors:\s*(.*)")
    with open(fname) as f:
        for line in f:
            line=line.strip()
            if line.lower()=="path:":
                raw = next(f).strip().split(',')
                path = [int(x) for x in raw if x]
                continue
            m = pat.match(line)
            if not m: continue
            nid = int(m.group(1))
            angs = list(map(float, m.group(2).split(',')))
            nodes[nid] = np.array(angs)
            for nbr in m.group(3).split():
                if nbr.isdigit():
                    edges.append((nid,int(nbr)))
    return nodes, edges, path

def fk_end_effector(angles, W):
    """EE in original coords: base at (W/2,0), Y up."""
    x, y = W/2.0, 0.0
    for θ in angles:
        x += LINK_LENGTH * np.cos(2*PI - θ)
        y -= LINK_LENGTH * np.sin(2*PI - θ)
    return np.array([x, y])

if __name__=="__main__":
    p = argparse.ArgumentParser()
    p.add_argument("map_file")
    p.add_argument("planner_output")
    p.add_argument("--output", "-o", help="save image", default=None)
    args = p.parse_args()

    # Load and parse
    M, W, H      = read_map(args.map_file)
    nodes, edges, path = parse_planner_output(args.planner_output)

    # Compute EE in original frame
    ee_orig = {nid: fk_end_effector(angs, W) for nid, angs in nodes.items()}

    # Rotate M CCW
    M_rot = np.rot90(M, k=1)
    W, H = H, W

    # Rotate EE coords CCW: (x,y) -> (H - y, x)
    ee = {nid: np.array([H - y, x]) for nid, (x, y) in ee_orig.items()}

    fig, ax = plt.subplots(figsize=(8, 8*H/W))
    ax.imshow(M_rot, cmap='gray_r', origin='lower',
              extent=[0, W, 0, H], interpolation='nearest')

    # Plot edges
    for u, v in edges:
        if u in ee and v in ee:
            x1, y1 = ee[u]; x2, y2 = ee[v]
            ax.plot([x1, x2], [y1, y2], color='cyan', alpha=0.5, lw=0.5)

    # Highlight path
    if path:
        for a, b in zip(path, path[1:]):
            if a in ee and b in ee:
                x1, y1 = ee[a]; x2, y2 = ee[b]
                ax.plot([x1, x2], [y1, y2], color='lime', lw=2)
        # start/goal
        s, g = path[0], path[-1]
        ax.plot(*ee[s], 'go', ms=8, mec='k', label='Start')
        ax.plot(*ee[g], 'ro', ms=8, mec='k', label='Goal')

    # Axes on top/left
    ax.set_xlim(0, W); ax.set_ylim(0, H)
    ax.xaxis.set_ticks_position('top')
    ax.xaxis.set_label_position('top')
    ax.spines['bottom'].set_visible(False)
    ax.yaxis.set_ticks_position('left')
    ax.spines['right'].set_visible(False)

    ax.set_aspect('equal','box')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend(loc='best')
    ax.set_title("Rotated CCW: RRT* Tree & Path")

    plt.tight_layout()
    if args.output:
        plt.savefig(args.output, dpi=200)
    plt.show()
