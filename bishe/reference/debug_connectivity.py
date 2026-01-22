#!/usr/bin/env python3
import numpy as np
import sys
from pathlib import Path

def parse_benchmark(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    grid_lines = []
    for line in lines:
        if line.strip() == "":
            break
        grid_lines.append(line)
    
    grid = []
    for line in grid_lines:
        row = list(map(int, line.strip().split('\t')))
        grid.append(row)
    
    return np.array(grid, dtype=int), [list(map(int, line.strip().split())) 
                                        for line in lines[len(grid_lines)+1:] 
                                        if line.strip()]

def load_routed(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    wires = []
    for line in lines:
        if line.strip():
            row = list(map(int, line.strip().split('\t')))
            wires.append(row)
    return np.array(wires, dtype=int)

def verify_net(nid, net, grid, wires):
    # Get all terminal cells
    term_cells = set()
    for vid in net:
        vid_norm = abs(vid)
        if vid_norm > 0:
            coords = list(zip(*np.where(grid == vid_norm)))
            term_cells.update(coords)
    
    print(f"\nNet {nid}: {len(term_cells)} terminals")
    print(f"  Terminals: {sorted(term_cells)}")
    
    # Get wire cells
    wire_cells = set(zip(*np.where(wires == nid)))
    print(f"  Wire cells: {len(wire_cells)}")
    
    # BFS
    if len(term_cells) < 2:
        return True
    
    traversable = wire_cells | term_cells
    start = next(iter(term_cells))
    visited = {start}
    queue = [start]
    H, W = grid.shape
    
    while queue:
        r, c = queue.pop(0)
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if (nr, nc) in visited:
                    continue
                if not (0 <= nr < H and 0 <= nc < W):
                    continue
                if (nr, nc) in traversable:
                    visited.add((nr, nc))
                    queue.append((nr, nc))
    
    missing = term_cells - visited
    if missing:
        print(f"  FAILED: Missing {len(missing)} terminals: {sorted(missing)}")
        print(f"  Visited: {len(visited)} cells")
        return False
    else:
        print(f"  SUCCESS")
        return True

if __name__ == "__main__":
    grid, nets = parse_benchmark("../benchmarks/CPA.txt")
    wires = load_routed("../benchmarks/CPA_geo_routed.txt")
    
    failed = []
    for nid in range(len(nets)):
        if not verify_net(nid, nets[nid], grid, wires):
            failed.append(nid)
    
    print(f"\nTotal: {len(nets) - len(failed)}/{len(nets)} verified")
    if failed:
        print(f"Failed: {failed}")
