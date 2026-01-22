#!/usr/bin/env python3
"""
Simplified Geometric Router:
- Θ* for direct visibility check between valve points
- Greedy geometric pathfinding when direct line blocked
- Much simpler and faster than full ACO
"""

import numpy as np
from collections import defaultdict
from typing import List, Tuple, Dict, Set
from pathlib import Path
import matplotlib.pyplot as plt
import sys

Coord = Tuple[int, int]
Point = Tuple[float, float]

class SimpleGeometricRouter:
    def __init__(self, grid: np.ndarray, nets: List[List[int]]):
        self.grid = grid.copy()
        self.nets = nets
        self.H, self.W = grid.shape
        self.wires = np.full((self.H, self.W), -1, dtype=int)
        self.wire_mult: Dict[Coord, Set[int]] = defaultdict(set)  # track multi-net occupancy
        self.net_status = {i: "pending" for i in range(len(nets))}
        # Map valve id -> nets that use it (for traversal permissions)
        self.valve_to_nets: Dict[int, Set[int]] = defaultdict(set)
        for nid, net in enumerate(nets):
            for vid in net:
                vid_norm = abs(vid)
                if vid_norm > 0:
                    self.valve_to_nets[vid_norm].add(nid)
    
    def route_all(self):
        """Two-stage routing: force-connect then optimize."""
        groups = self._build_net_groups()
        print(f"Found {len(groups)} net groups\n")
        
        # Stage 1: Force connectivity (allow high-cost/shared paths)
        print("=== Stage 1: Force connectivity ===")
        for gid, nids in enumerate(groups):
            print(f"Group {gid}: nets {nids}")
            success = self._route_group_forced(nids)
            print("  OK" if success else "  FAIL")
        
        # Check connectivity after stage 1
        connected = []
        disconnected = []
        for nid in range(len(self.nets)):
            comps = self._net_components(nid, self._net_terminals(nid))
            if len(comps) == 1:
                connected.append(nid)
                self.net_status[nid] = "success"
            else:
                disconnected.append((nid, len(comps)))
                self.net_status[nid] = "failed"
        print(f"After stage 1: {len(connected)}/{len(self.nets)} connected")
        print(f"  Connected: {connected}")
        print(f"  Disconnected: {disconnected}\n")
        
        # Stage 2: Bridge disconnected components via dynamic Kruskal + multi-source Dijkstra
        print("=== Stage 2: Bridge trees with Dijkstra-Kruskal ===")
        failed_nets = [nid for nid in range(len(self.nets)) if self.net_status[nid] == "failed"]
        if failed_nets:
            print(f"Bridging: {failed_nets}")
            for nid in failed_nets:
                terms = self._net_terminals(nid)
                comps = self._net_components(nid, terms)
                print(f"  Net {nid}: {len(comps)} trees")
                if self._connect_forest_kruskal(nid, terms):
                    comps = self._net_components(nid, terms)
                    if len(comps) == 1:
                        self.net_status[nid] = "success"
                        print("    -> Connected")
                    else:
                        print(f"    -> Still {len(comps)} trees")
                else:
                    print("    -> No bridge found")
        
        # Stage 3: Ensure every terminal has at least one wire cell
        print("\n=== Stage 3: Connect isolated terminals ===")
        for nid in range(len(self.nets)):
            if self._connect_isolated_terminals(nid):
                if self.net_status[nid] != "success":
                    comps = self._net_components(nid, self._net_terminals(nid))
                    if len(comps) == 1:
                        self.net_status[nid] = "success"
                        print(f"  Net {nid}: isolated terminals connected")
        
        final_failed = [nid for nid in range(len(self.nets)) if self.net_status[nid] == "failed"]
        if final_failed:
            print(f"Still failed: {final_failed}")
        
        # Mark all valve cells with their net IDs to ensure connectivity
        print("\n=== Marking valve cells ===")
        for nid in range(len(self.nets)):
            for vid in self.nets[nid]:
                vid_norm = abs(vid)
                if vid_norm > 0:
                    coords = self._find_valve(vid_norm)
                    for r, c in coords:
                        if self.wires[r, c] in (-1, nid):
                            self.wires[r, c] = nid
                        self.wire_mult[(r, c)].add(nid)
        
        if not final_failed:
            # Post-pass: prune redundant leaves to compact routes
            # DISABLED for now to preserve connectivity
            # for nid in range(len(self.nets)):
            #     self._prune_leaves(nid)
            # print("Post-prune leaves: done")
            pass
        
        # Final BFS verification
        print("\n=== BFS Connectivity Verification ===")
        bfs_verified = []
        bfs_failed = []
        for nid in range(len(self.nets)):
            if self._verify_connectivity_bfs(nid):
                bfs_verified.append(nid)
            else:
                bfs_failed.append(nid)
        print(f"BFS verified: {len(bfs_verified)}/{len(self.nets)}")
        if bfs_failed:
            print(f"BFS failed: {bfs_failed}")
        
        return sum(1 for s in self.net_status.values() if s == "success")
    
    def _build_net_groups(self) -> List[List[int]]:
        """Group nets by shared terminals."""
        adj = defaultdict(set)
        for i in range(len(self.nets)):
            phys_i = set(abs(v) for v in self.nets[i] if v != 0)
            for j in range(i + 1, len(self.nets)):
                phys_j = set(abs(v) for v in self.nets[j] if v != 0)
                if phys_i & phys_j:
                    adj[i].add(j)
                    adj[j].add(i)
        
        visited = set()
        groups = []
        for nid in range(len(self.nets)):
            if nid in visited:
                continue
            group = []
            queue = [nid]
            while queue:
                cur = queue.pop(0)
                if cur in visited:
                    continue
                visited.add(cur)
                group.append(cur)
                for nbr in adj[cur]:
                    if nbr not in visited:
                        queue.append(nbr)
            groups.append(sorted(group))
        return groups
    
    def _get_group_shared_valves(self, nids: List[int]) -> List[int]:
        """Get valves shared by ALL nets in group."""
        if not nids:
            return []
        sets = [set(abs(v) for v in self.nets[nid] if v != 0) for nid in nids]
        shared = sets[0]
        for s in sets[1:]:
            shared &= s
        return sorted(list(shared))
    
    def _route_group_forced(self, nids: List[int]) -> bool:
        """Route each net in group independently with forced connectivity."""
        for nid in nids:
            terms = self._net_terminals(nid)
            if len(terms) < 2:
                continue
            ok = self._connect_components_forced(terms, {nid})
            # Even if failed, continue (allow partial)
        return True

    def _route_group(self, nids: List[int]) -> bool:
        """Route a group by repeatedly bridging its disconnected components."""
        # Collect all terminal coordinates (centers of valve cells)
        all_terminals = []
        for nid in nids:
            phys = [abs(v) for v in self.nets[nid] if v != 0]
            for vid in phys:
                coords = self._find_valve(vid)
                for r, c in coords:
                    pt = (float(r) + 0.5, float(c) + 0.5)
                    if pt not in all_terminals:
                        all_terminals.append(pt)
        
        if len(all_terminals) < 2:
            return True
        
        print(f"    Bridging {len(all_terminals)} terminals via component MST...")
        return self._connect_components(all_terminals, set(nids))

    def _connect_components_forced(self, terminals: List[Point], allowed_nets: Set[int]) -> bool:
        """Connect components allowing high-cost or shared cell occupation for forced connectivity."""
        components = [{i} for i in range(len(terminals))]
        
        while len(components) > 1:
            best = None  # (cost, i, j, path, cells)
            
            for i in range(len(components)):
                for j in range(i + 1, len(components)):
                    comp_a = components[i]
                    comp_b = components[j]
                    for a_idx in comp_a:
                        for b_idx in comp_b:
                            p_a = terminals[a_idx]
                            p_b = terminals[b_idx]
                            # Try forced pathfinding (allow occupying other nets' cells with cost penalty)
                            path = self._find_geometric_path_forced(p_a, p_b, allowed_nets)
                            if path is None:
                                continue
                            cost = self._path_length(path)
                            if best is None or cost < best[0]:
                                cells = self._rasterize_path(path)
                                best = (cost, i, j, path, cells)
            
            if best is None:
                return False
            
            _, i, j, path, cells = best
            # Commit for all nets
            for r, c in cells:
                if 0 <= r < self.H and 0 <= c < self.W:
                    for nid in allowed_nets:
                        if self.wires[r, c] in (-1, nid):
                            self.wires[r, c] = nid
                        self.wire_mult[(r, c)].add(nid)
            
            merged = components[i] | components[j]
            components = [components[k] for k in range(len(components)) if k not in (i, j)]
            components.append(merged)
        
        return True

    def _connect_components(self, terminals: List[Point], allowed_nets: Set[int]) -> bool:
        """Connect all terminal components by shortest feasible bridge; limit candidate pairs per component."""
        # Each terminal starts as its own component
        components = [{i} for i in range(len(terminals))]
        k_candidates = 3  # limit per component to nearest boundary points

        while len(components) > 1:
            best = None  # (cost, comp_a_idx, comp_b_idx, path, cells)

            # Precompute candidate points per component (nearest to others)
            comp_candidates: List[List[int]] = []
            for idx, comp in enumerate(components):
                # Distance from each terminal in comp to nearest terminal outside comp
                scores = []
                for t_idx in comp:
                    p = terminals[t_idx]
                    nearest = min(self._distance(p, terminals[o]) for o in range(len(terminals)) if o not in comp)
                    scores.append((nearest, t_idx))
                scores.sort()
                comp_candidates.append([t for _, t in scores[:k_candidates]])

            # Evaluate component pairs over candidate terminals
            for i in range(len(components)):
                for j in range(i + 1, len(components)):
                    comp_a_candidates = comp_candidates[i]
                    comp_b_candidates = comp_candidates[j]
                    for a_idx in comp_a_candidates:
                        for b_idx in comp_b_candidates:
                            p_a = terminals[a_idx]
                            p_b = terminals[b_idx]
                            path = self._find_geometric_path(p_a, p_b, allowed_nets)
                            if path is None:
                                continue
                            cost = self._path_length(path)
                            if best is None or cost < best[0]:
                                cells = self._rasterize_path(path)
                                best = (cost, i, j, path, cells)

            if best is None:
                return False  # no feasible bridge

            _, i, j, path, cells = best
            # Commit bridge for all nets in this group
            for r, c in cells:
                if 0 <= r < self.H and 0 <= c < self.W:
                    for nid in allowed_nets:
                        if self.wires[r, c] == -1 or self.wires[r, c] == nid:
                            self.wires[r, c] = nid
                        self.wire_mult[(r, c)].add(nid)

            # Merge components
            merged = components[i] | components[j]
            components = [components[k] for k in range(len(components)) if k not in (i, j)]
            components.append(merged)

        return True
    
    def _find_valve(self, valve_id: int) -> List[Coord]:
        """Find all grid cells with this valve."""
        coords = np.argwhere(self.grid == valve_id)
        return [(int(r), int(c)) for r, c in coords]
    
    def _find_geometric_path_forced(self, start: Point, goal: Point, allowed_nets: Set[int]) -> List[Point] | None:
        """Pathfinding allowing high-cost crossing of other nets' cells."""
        # Try direct LOS first
        if self._has_los(start, goal, allowed_nets):
            return [start, goal]
        
        # Try one-bend paths with forced crossing allowed
        candidates: List[Point] = []
        ortho1 = (start[0], goal[1])
        ortho2 = (goal[0], start[1])
        candidates.extend([ortho1, ortho2])
        offsets = [0.5, -0.5]
        for off in offsets:
            candidates.append((start[0] + off, goal[1]))
            candidates.append((goal[0], start[1] + off))

        best_path = None
        best_cost = 1e9
        for mid in candidates:
            if not self._point_traversable_forced(mid, allowed_nets):
                continue
            if not self._has_los_forced(start, mid, allowed_nets):
                continue
            if not self._has_los_forced(mid, goal, allowed_nets):
                continue
            path = [start, mid, goal]
            cost = self._path_length(path)
            if cost < best_cost:
                best_cost = cost
                best_path = path
        if best_path:
            return best_path
        
        # Greedy with forced crossing
        path = [start]
        current = start
        max_steps = 200
        step_size = 0.5
        stuck_count = 0
        
        for _ in range(max_steps):
            if self._distance(current, goal) < 1.0:
                path.append(goal)
                return path
            
            dy = goal[0] - current[0]
            dx = goal[1] - current[1]
            dist = (dy**2 + dx**2)**0.5
            if dist < 0.1:
                path.append(goal)
                return path
            dy /= dist
            dx /= dist
            
            next_pt = (current[0] + step_size * dy, current[1] + step_size * dx)
            if self._point_traversable_forced(next_pt, allowed_nets):
                current = next_pt
                path.append(current)
                stuck_count = 0
            else:
                # Try alternatives or push through with higher cost
                stuck_count += 1
                if stuck_count > 3:
                    # Force through obstacle with high cost
                    current = next_pt
                    path.append(current)
                    stuck_count = 0
                else:
                    alternatives = [
                        (current[0] + step_size, current[1]),
                        (current[0] - step_size, current[1]),
                        (current[0], current[1] + step_size),
                        (current[0], current[1] - step_size),
                        (current[0] + step_size, current[1] + step_size),
                        (current[0] + step_size, current[1] - step_size),
                        (current[0] - step_size, current[1] + step_size),
                        (current[0] - step_size, current[1] - step_size),
                    ]
                    best_alt = None
                    best_dist = 1e9
                    for alt in alternatives:
                        if self._point_traversable_forced(alt, allowed_nets):
                            alt_dist = self._distance(alt, goal)
                            if alt_dist < best_dist:
                                best_dist = alt_dist
                                best_alt = alt
                    if best_alt:
                        current = best_alt
                        path.append(current)
                    else:
                        return None
        return None

    def _point_traversable_forced(self, pt: Point, allowed_nets: Set[int]) -> bool:
        """Point is traversable if no hard obstacle; valves usable when they belong to allowed nets."""
        r, c = int(pt[0]), int(pt[1])
        if not (0 <= r < self.H and 0 <= c < self.W):
            return False
        cell_val = self.grid[r, c]
        if cell_val > 0:
            return bool(self.valve_to_nets[cell_val] & allowed_nets)
        return True
    
    def _has_los_forced(self, p1: Point, p2: Point, allowed_nets: Set[int]) -> bool:
        """LOS check allowing traversal through allowed valves and other nets."""
        cells = self._bresenham_line(p1, p2)
        for r, c in cells:
            if not (0 <= r < self.H and 0 <= c < self.W):
                return False
            cell_val = self.grid[r, c]
            if cell_val > 0 and not (self.valve_to_nets[cell_val] & allowed_nets):
                return False
        return True

    def _find_geometric_path(self, start: Point, goal: Point, allowed_nets: Set[int]) -> List[Point] | None:
        """
        Find path in geometric space respecting existing wires/obstacles.
        1) Direct LOS
        2) One-bend (L/V) paths
        3) Greedy stepping toward goal
        """
        # 1) Direct LOS
        if self._has_los(start, goal, allowed_nets):
            return [start, goal]

        # 2) One-bend candidates: try orthogonal and slight offsets
        candidates: List[Point] = []
        ortho1 = (start[0], goal[1])
        ortho2 = (goal[0], start[1])
        candidates.extend([ortho1, ortho2])
        # small offsets (+/-0.5) to escape blocking obstacles
        offsets = [0.5, -0.5]
        for off in offsets:
            candidates.append((start[0] + off, goal[1]))
            candidates.append((goal[0], start[1] + off))

        best_path = None
        best_cost = 1e9
        for mid in candidates:
            if not self._point_traversable(mid, allowed_nets):
                continue
            if not self._has_los(start, mid, allowed_nets):
                continue
            if not self._has_los(mid, goal, allowed_nets):
                continue
            path = [start, mid, goal]
            cost = self._path_length(path)
            if cost < best_cost:
                best_cost = cost
                best_path = path
        if best_path:
            return best_path

        # 3) Greedy search: move toward goal with detours when blocked
        path = [start]
        current = start
        max_steps = 200
        step_size = 0.5

        for _ in range(max_steps):
            if self._distance(current, goal) < 1.0:
                path.append(goal)
                return path

            dy = goal[0] - current[0]
            dx = goal[1] - current[1]
            dist = (dy**2 + dx**2)**0.5
            if dist < 0.1:
                path.append(goal)
                return path
            dy /= dist
            dx /= dist

            next_pt = (current[0] + step_size * dy, current[1] + step_size * dx)
            if self._point_traversable(next_pt, allowed_nets):
                current = next_pt
                path.append(current)
            else:
                alternatives = [
                    (current[0] + step_size, current[1]),
                    (current[0] - step_size, current[1]),
                    (current[0], current[1] + step_size),
                    (current[0], current[1] - step_size),
                    (current[0] + step_size, current[1] + step_size),
                    (current[0] + step_size, current[1] - step_size),
                    (current[0] - step_size, current[1] + step_size),
                    (current[0] - step_size, current[1] - step_size),
                ]
                best_alt = None
                best_dist = 1e9
                for alt in alternatives:
                    if self._point_traversable(alt, allowed_nets):
                        alt_dist = self._distance(alt, goal)
                        if alt_dist < best_dist:
                            best_dist = alt_dist
                            best_alt = alt
                if best_alt:
                    current = best_alt
                    path.append(current)
                else:
                    return None
        return None
    
    def _has_los(self, p1: Point, p2: Point, allowed_nets: Set[int]) -> bool:
        """Check line-of-sight using Bresenham rasterization respecting existing wires."""
        cells = self._bresenham_line(p1, p2)
        for r, c in cells:
            if not self._cell_free(r, c, allowed_nets):
                return False
        return True
    
    def _bresenham_line(self, p1: Point, p2: Point) -> List[Coord]:
        """Get grid cells along line from p1 to p2."""
        x0, y0 = int(p1[1]), int(p1[0])
        x1, y1 = int(p2[1]), int(p2[0])
        cells = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        
        while True:
            cells.append((y, x))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def _neighbors8(self, pos: Coord) -> List[Coord]:
        r, c = pos
        res = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.H and 0 <= nc < self.W:
                    res.append((nr, nc))
        return res
    
    def _point_traversable(self, pt: Point, allowed_nets: Set[int] | None) -> bool:
        """Check if a point can be occupied (no obstacle and not blocked by other nets)."""
        if allowed_nets is None:
            allowed_nets = set()
        r, c = int(pt[0]), int(pt[1])
        if not (0 <= r < self.H and 0 <= c < self.W):
            return False
        cell_val = self.grid[r, c]
        if cell_val > 0:
            # Allow traversal if this valve belongs to the active net(s)
            return bool(self.valve_to_nets[cell_val] & allowed_nets)
        if self.wires[r, c] == -1:
            return True
        return self.wires[r, c] in allowed_nets
    
    def _cell_free(self, r: int, c: int, allowed_nets: Set[int] | None) -> bool:
        """Cell is free for LOS if obstacle-free and not occupied by other nets."""
        if allowed_nets is None:
            allowed_nets = set()
        if not (0 <= r < self.H and 0 <= c < self.W):
            return False
        cell_val = self.grid[r, c]
        if cell_val > 0 and not (self.valve_to_nets[cell_val] & allowed_nets):
            return False
        if self.wires[r, c] == -1:
            return True
        return self.wires[r, c] in allowed_nets
    
    def _distance(self, p1: Point, p2: Point) -> float:
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

    def _path_length(self, path: List[Point]) -> float:
        """Euclidean length of a polyline path."""
        return sum(self._distance(path[i], path[i+1]) for i in range(len(path) - 1))

    def _component_boundary_points(self, comp: Set[Coord], target_comp: Set[Coord]) -> List[Point]:
        """Get boundary points of a component closest to target component."""
        boundary = []
        for r, c in comp:
            # Find nearest distance to target component
            min_dist = min(((r - tr)**2 + (c - tc)**2)**0.5 for tr, tc in target_comp)
            boundary.append((min_dist, (float(r) + 0.5, float(c) + 0.5)))
        
        # Sort by distance and return top candidates
        boundary.sort()
        return [pt for _, pt in boundary[:5]]

    def _connect_forest_kruskal(self, nid: int, terminals: List[Point]) -> bool:
        """Connect all components of a net using multi-source Dijkstra (dynamic Kruskal)."""
        comps = self._net_components(nid, terminals)
        if len(comps) <= 1:
            return True

        # While more than one component, find shortest bridge between any two components
        for _ in range(100):  # safety cap
            comps = self._net_components(nid, terminals)
            if len(comps) <= 1:
                return True

            bridge = self._shortest_bridge_dijkstra(nid, comps)
            if bridge is None:
                return False

            path_cells = bridge
            for r, c in path_cells:
                if 0 <= r < self.H and 0 <= c < self.W:
                    self.wires[r, c] = nid
                    self.wire_mult[(r, c)].add(nid)

        return len(self._net_components(nid, terminals)) == 1

    def _shortest_bridge_dijkstra(self, nid: int, comps: List[Set[Coord]]) -> List[Coord] | None:
        """Multi-source Dijkstra; returns grid cells of shortest bridge between any two comps."""
        # Label each source cell with its component index
        import heapq
        dist = {}
        label = {}
        parent = {}
        pq = []

        for idx, comp in enumerate(comps):
            for cell in comp:
                dist[cell] = 0.0
                label[cell] = idx
                parent[cell] = None
                heapq.heappush(pq, (0.0, cell[0], cell[1], idx))

        best = None  # (cost, cell_a, cell_b)
        target_pair = None

        while pq:
            d, r, c, src = heapq.heappop(pq)
            if d != dist[(r, c)]:
                continue
            # If this cell already claimed by another component, we found a meeting point
            if label[(r, c)] != src:
                total_cost = dist[(r, c)] + dist[(r, c)]  # approximate; we'll reconstruct
                best = (total_cost, (r, c), (r, c))
                target_pair = (label[(r, c)], src, (r, c))
                break

            for nr, nc in self._neighbors8((r, c)):
                w = self._edge_cost(nid, (r, c), (nr, nc))
                if w >= 1e9:
                    continue
                nd = d + w
                if nd < dist.get((nr, nc), 1e18):
                    dist[(nr, nc)] = nd
                    label[(nr, nc)] = src
                    parent[(nr, nc)] = (r, c)
                    heapq.heappush(pq, (nd, nr, nc, src))
                elif label.get((nr, nc)) is not None and label[(nr, nc)] != src:
                    # meeting of two components via existing node
                    meet_cost = dist[(nr, nc)] + d + w
                    if best is None or meet_cost < best[0]:
                        best = (meet_cost, (r, c), (nr, nc))
                        target_pair = (label[(nr, nc)], src, (nr, nc), (r, c))

        if best is None or target_pair is None:
            return None

        # Reconstruct path between the two meeting nodes
        def backtrace(cell):
            path = []
            cur = cell
            while cur is not None:
                path.append(cur)
                cur = parent.get(cur)
            return path

        # target_pair could be 3-tuple or 4-tuple depending on where we found meeting
        if len(target_pair) == 3:
            _, src, meet = target_pair
            path_a = backtrace(meet)
            path_b = backtrace(meet)
        else:
            comp_a, comp_b, cell_a, cell_b = target_pair
            path_a = backtrace(cell_a)
            path_b = backtrace(cell_b)

        # Merge two paths (reverse second excluding overlap)
        bridge = list(path_a)
        bridge_rev = path_b[::-1]
        if bridge and bridge_rev and bridge[-1] == bridge_rev[0]:
            bridge_rev = bridge_rev[1:]
        bridge.extend(bridge_rev)
        return bridge

    def _edge_cost(self, nid: int, a: Coord, b: Coord) -> float:
        """Weighted edge cost for Dijkstra bridging."""
        r, c = b
        if not (0 <= r < self.H and 0 <= c < self.W):
            return 1e9
        cell_val = self.grid[r, c]
        if cell_val > 0:
            # Only allow entering valve cells that belong to this net
            if nid not in self.valve_to_nets[cell_val]:
                return 1e9
            base = 0.05  # encourage docking into own valve
        else:
            base = 1.0
        if self.wires[r, c] == nid:
            base = min(base, 0.1)  # encourage reuse
        elif self.wires[r, c] >= 0 and self.wires[r, c] != nid:
            base = max(base, 8.0)  # crossing other nets is expensive but allowed
        # diagonal penalty
        if abs(a[0]-b[0]) + abs(a[1]-b[1]) == 2:
            base *= 1.41421356
        return base

    def _net_terminals(self, nid: int) -> List[Point]:
        """Return one geometric center per physical valve for a net."""
        pts: List[Point] = []
        phys = [abs(v) for v in self.nets[nid] if v != 0]
        for vid in phys:
            coords = self._find_valve(vid)
            if not coords:
                continue
            avg_r = float(np.mean([r for r, _ in coords]))
            avg_c = float(np.mean([c for _, c in coords]))
            # Snap to the valve cell closest to centroid to ensure rasterization hits the valve
            best_cell = min(coords, key=lambda rc: (rc[0] - avg_r) ** 2 + (rc[1] - avg_c) ** 2)
            br, bc = best_cell
            pts.append((br + 0.5, bc + 0.5))
        return pts

    def _prune_dangling(self, nid: int, max_iter: int = 3):
        """Remove only true dangling tips (degree 1, far from terminals)."""
        terminals = set(self._net_terminals(nid))
        term_cells = {(int(r), int(c)) for r, c in [(p[0], p[1]) for p in terminals]}
        
        for _ in range(max_iter):
            removed = False
            cells = list(zip(*np.where(self.wires == nid)))
            for r, c in cells:
                if (r, c) in term_cells:
                    continue
                # Count degree
                neighbors = []
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < self.H and 0 <= nc < self.W and self.wires[nr, nc] == nid:
                            neighbors.append((nr, nc))
                
                # Only remove if degree=1 AND it's a short dangling branch
                if len(neighbors) == 1:
                    # Don't remove if it's a junction point or critical bridge
                    nr, nc = neighbors[0]
                    junction_count = 0
                    for dr in [-1, 0, 1]:
                        for dc in [-1, 0, 1]:
                            if dr == 0 and dc == 0:
                                continue
                            nnr, nnc = nr + dr, nc + dc
                            if 0 <= nnr < self.H and 0 <= nnc < self.W and self.wires[nnr, nnc] == nid:
                                junction_count += 1
                    # Only remove if neighbor also has degree 1 or is terminal
                    if junction_count <= 1:
                        self.wires[r, c] = -1
                        removed = True
            
            if not removed:
                break

    def _prune_leaves(self, nid: int):
        """Iteratively strip non-terminal leaves to reduce redundant branches."""
        term_cells: Set[Coord] = set()
        for vid in self.nets[nid]:
            vid_norm = abs(vid)
            if vid_norm > 0:
                term_cells.update(self._find_valve(vid_norm))
        term_cells.update({(int(round(p[0])), int(round(p[1]))) for p in self._net_terminals(nid)})

        changed = True
        while changed:
            changed = False
            cells = list(zip(*np.where(self.wires == nid)))
            for r, c in cells:
                if (r, c) in term_cells:
                    continue
                # degree within this net
                deg = 0
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if 0 <= nr < self.H and 0 <= nc < self.W and self.wires[nr, nc] == nid:
                            deg += 1
                if deg <= 1:
                    self.wires[r, c] = -1
                    changed = True
    
    def _connect_isolated_terminals(self, nid: int) -> bool:
        """Connect any isolated terminal cells (not adjacent to any wire) to the net."""
        term_cells: Set[Coord] = set()
        for vid in self.nets[nid]:
            vid_norm = abs(vid)
            if vid_norm > 0:
                term_cells.update(self._find_valve(vid_norm))
        
        if not term_cells:
            return False
        
        # Find isolated terminals (not adjacent to any wire cell of this net)
        wire_cells = set(zip(*np.where(self.wires == nid)))
        isolated = []
        for r, c in term_cells:
            has_adjacent_wire = False
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < self.H and 0 <= nc < self.W:
                        if self.wires[nr, nc] == nid:
                            has_adjacent_wire = True
                            break
                if has_adjacent_wire:
                    break
            if not has_adjacent_wire:
                isolated.append((r, c))
        
        if not isolated:
            return False
        
        print(f"    Net {nid}: {len(isolated)} isolated terminals")
        
        # For each isolated terminal, use Dijkstra to find shortest path to wire cell
        import heapq
        for iso_r, iso_c in isolated:
            dist = {(iso_r, iso_c): 0.0}
            parent = {(iso_r, iso_c): None}
            pq = [(0.0, iso_r, iso_c)]
            found_path = None
            
            while pq:
                d, r, c = heapq.heappop(pq)
                if d != dist[(r, c)]:
                    continue
                
                # Check if we reached a wire cell
                if self.wires[r, c] == nid:
                    # Reconstruct path
                    path = []
                    cur = (r, c)
                    while cur is not None:
                        path.append(cur)
                        cur = parent.get(cur)
                    path.reverse()
                    found_path = path
                    break
                
                # Expand to neighbors
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if not (0 <= nr < self.H and 0 <= nc < self.W):
                            continue
                        if self.grid[nr, nc] > 0 and (nr, nc) not in term_cells:
                            continue  # Block other valves
                        
                        cost = 1.0 + 0.414 if (abs(dr) + abs(dc) == 2) else 1.0
                        nd = d + cost
                        if nd < dist.get((nr, nc), 1e18):
                            dist[(nr, nc)] = nd
                            parent[(nr, nc)] = (r, c)
                            heapq.heappush(pq, (nd, nr, nc))
            
            if found_path:
                for r, c in found_path:
                    if self.wires[r, c] in (-1, nid):
                        self.wires[r, c] = nid
                    self.wire_mult[(r, c)].add(nid)
        
        return True
    
    def _verify_connectivity_bfs(self, nid: int) -> bool:
        """Use BFS to verify all terminals of a net are reachable from one another."""
        # Get all terminal cells for this net
        term_cells: Set[Coord] = set()
        for vid in self.nets[nid]:
            vid_norm = abs(vid)
            if vid_norm > 0:
                term_cells.update(self._find_valve(vid_norm))
        
        if len(term_cells) < 2:
            return True
        
        # Build traversable cells: wire cells OR valve cells belonging to this net
        traversable: Set[Coord] = set(coord for coord, owners in self.wire_mult.items() if nid in owners)
        traversable.update(term_cells)
        
        # Start BFS from first terminal
        start = next(iter(term_cells))
        visited: Set[Coord] = {start}
        queue = [start]
        
        while queue:
            r, c = queue.pop(0)
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if (nr, nc) in visited:
                        continue
                    if not (0 <= nr < self.H and 0 <= nc < self.W):
                        continue
                    # Can traverse if in traversable set
                    if (nr, nc) in traversable:
                        visited.add((nr, nc))
                        queue.append((nr, nc))
        
        # Check if all terminals are reachable
        return term_cells.issubset(visited)

    def _ensure_net_connected(self, nid: int) -> bool:
        """Ensure all terminals of a net are connected; try bridging if not."""
        terms = self._net_terminals(nid)
        if len(terms) < 2:
            return True
        comps = self._net_components(nid, terms)
        if len(comps) == 1:
            return True
        # Try to connect components with shortest bridges
        comp_points = []
        for comp in comps:
            # pick representative points (grid cells)
            comp_points.append([(r, c) for (r, c) in comp])
        # Use geometric representative (center of first cell)
        reps = [(float(cells[0][0]) + 0.0, float(cells[0][1]) + 0.0) for cells in comp_points]
        success = self._connect_components(reps, {nid})
        if success:
            comps2 = self._net_components(nid, terms)
            return len(comps2) == 1
        return False

    def _net_components(self, nid: int, terminals: List[Point]) -> List[Set[Coord]]:
        """Connected components of net wires+terminals (8-neighbor)."""
        # Collect all terminal cells
        term_cells: Set[Coord] = set()
        for vid in self.nets[nid]:
            vid_norm = abs(vid)
            if vid_norm > 0:
                term_cells.update(self._find_valve(vid_norm))
        
        # Get all wire cells
        wire_cells = set(zip(*np.where(self.wires == nid)))
        
        visited: Set[Coord] = set()
        comps: List[Set[Coord]] = []

        # Start DFS/BFS from each unvisited cell (wire or terminal)
        all_cells = wire_cells | term_cells
        for start_cell in list(all_cells):
            if start_cell in visited:
                continue
            
            stack = [start_cell]
            comp = set()
            visited.add(start_cell)
            
            while stack:
                r, c = stack.pop()
                comp.add((r, c))
                
                for dr in [-1, 0, 1]:
                    for dc in [-1, 0, 1]:
                        if dr == 0 and dc == 0:
                            continue
                        nr, nc = r + dr, c + dc
                        if (nr, nc) in visited:
                            continue
                        if not (0 <= nr < self.H and 0 <= nc < self.W):
                            continue
                        # Can traverse wire cells OR terminal cells (they're connected via adjacency)
                        if (nr, nc) in wire_cells or (nr, nc) in term_cells:
                            visited.add((nr, nc))
                            stack.append((nr, nc))
            
            comps.append(comp)
        
        return comps
    
    def _rasterize_path(self, path: List[Point]) -> List[Coord]:
        """Convert geometric path to grid cells."""
        cells = set()
        for i in range(len(path) - 1):
            seg_cells = self._bresenham_line(path[i], path[i+1])
            cells.update(seg_cells)
        return list(cells)
    
    def visualize(self, output_path: str):
        fig, ax = plt.subplots(figsize=(self.W * 0.3, self.H * 0.3))
        
        img = np.ones((self.H, self.W, 3))
        wire_mask = self.wires >= 0
        img[wire_mask] = [0.2, 0.8, 0.2]
        valve_mask = self.grid > 0
        img[valve_mask] = [0.3, 0.3, 1.0]
        
        ax.imshow(img, origin='upper')
        ax.set_title(f"Geometric: {sum(1 for s in self.net_status.values() if s == 'success')}/{len(self.nets)}")
        ax.axis('off')
        plt.tight_layout()
        plt.savefig(output_path, dpi=150)
        plt.close()
    
    def save_routed_grid(self, output_path: str):
        with open(output_path, 'w') as f:
            for row in self.wires:
                f.write('\t'.join(map(str, row)) + '\n')


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


def main():
    if len(sys.argv) < 2:
        print("Usage: python router_geo_simple.py <benchmark>")
        sys.exit(1)
    
    bench = sys.argv[1]
    bench_path = Path("../benchmarks") / f"{bench}.txt"
    
    if not bench_path.exists():
        print(f"Not found: {bench_path}")
        sys.exit(1)
    
    grid, nets = parse_benchmark(bench_path)
    print(f"Routing {bench} with geometric pathfinding...\n")
    
    router = SimpleGeometricRouter(grid, nets)
    success_count = router.route_all()
    
    wire_count = np.sum(router.wires >= 0)
    
    print("\n" + "="*60)
    print(f"Success: {success_count}/{len(nets)} ({100*success_count//len(nets)}%)")
    print(f"Wires: {wire_count}")
    print("="*60)
    
    output_dir = Path("../benchmarks")
    router.save_routed_grid(output_dir / f"{bench}_geo_routed.txt")
    router.visualize(output_dir / f"{bench}_geo_routed.png")

def _verify_net_connectivity_bfs(self, net_idx: int) -> bool:
    """用 BFS 验证网络的所有阀门终端是否连通"""
    net = self.nets[net_idx]
    # 归一化阀门 ID
    valve_ids = {abs(v) for v in net}
    # 收集所有终端单元格
    terminals = set()
    for vid in valve_ids:
        if vid in self.valves:
            terminals.update(self.valves[vid])
    
    if len(terminals) < 2:
        return True  # 单终端或空网视为连通
    
    # 从第一个终端开始 BFS
    start = next(iter(terminals))
    visited = {start}
    queue = [start]
    
    while queue:
        r, c = queue.pop(0)
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr, nc = r + dr, c + dc
            if (nr, nc) in visited:
                continue
            if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                continue
            # 只能经过本网的线或阀门
            if self.wires[nr, nc] == net_idx or self.grid[nr, nc] in valve_ids:
                visited.add((nr, nc))
                queue.append((nr, nc))
    
    # 检查是否所有终端都被访问
    return terminals.issubset(visited)
if __name__ == "__main__":
    main()
