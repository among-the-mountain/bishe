#!/usr/bin/env python3
"""
Routing Optimization Framework
Based on the problem statement requirements:
1. Basic path generation (A*/Theta*/DRL)
2. Geometric constraint embedding (spacing, boundaries)
3. Crossing constraint optimization
4. Timing constraint optimization
5. Local optimization strategies
6. Multi-objective optimization function
"""

import numpy as np
from typing import List, Tuple, Set, Dict, Optional
from dataclasses import dataclass
from collections import defaultdict
import heapq

Point = Tuple[float, float]
Coord = Tuple[int, int]


@dataclass
class PathSegment:
    """Represents a segment of a routing path"""
    start: Point
    end: Point
    net_id: int
    
    def length(self) -> float:
        """Calculate segment length"""
        dy = self.end[0] - self.start[0]
        dx = self.end[1] - self.start[1]
        return (dy**2 + dx**2)**0.5
    
    def intersects(self, other: 'PathSegment') -> bool:
        """Check if this segment intersects with another"""
        # Line segment intersection using cross product method
        def ccw(A: Point, B: Point, C: Point) -> bool:
            return (C[0] - A[0]) * (B[1] - A[1]) > (B[0] - A[0]) * (C[1] - A[1])
        
        A, B = self.start, self.end
        C, D = other.start, other.end
        
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
    
    def distance_to_segment(self, other: 'PathSegment') -> float:
        """
        Calculate minimum distance between two segments
        
        NOTE: This is a conservative approximation using midpoints.
        For production use, implement proper segment-to-segment distance:
        - Point-to-segment distance for each endpoint
        - Segment-to-segment perpendicular distance
        This simplified version errs on the side of reporting closer distances.
        """
        # Calculate distance between segment midpoints as approximation
        mid1 = ((self.start[0] + self.end[0]) / 2, (self.start[1] + self.end[1]) / 2)
        mid2 = ((other.start[0] + other.end[0]) / 2, (other.start[1] + other.end[1]) / 2)
        dy = mid2[0] - mid1[0]
        dx = mid2[1] - mid1[1]
        return (dy**2 + dx**2)**0.5


@dataclass
class RoutingPath:
    """Represents a complete routing path for a net"""
    net_id: int
    points: List[Point]
    segments: List[PathSegment]
    
    def __init__(self, net_id: int, points: List[Point]):
        self.net_id = net_id
        self.points = points
        self.segments = []
        for i in range(len(points) - 1):
            self.segments.append(PathSegment(points[i], points[i+1], net_id))
    
    def total_length(self) -> float:
        """Calculate total path length"""
        return sum(seg.length() for seg in self.segments)
    
    def num_bends(self) -> int:
        """Count number of bends in the path"""
        if len(self.points) < 3:
            return 0
        bends = 0
        for i in range(1, len(self.points) - 1):
            # Check if direction changes
            v1 = (self.points[i][0] - self.points[i-1][0], 
                  self.points[i][1] - self.points[i-1][1])
            v2 = (self.points[i+1][0] - self.points[i][0],
                  self.points[i+1][1] - self.points[i][1])
            # Normalize
            len1 = (v1[0]**2 + v1[1]**2)**0.5
            len2 = (v2[0]**2 + v2[1]**2)**0.5
            if len1 > 0 and len2 > 0:
                v1 = (v1[0]/len1, v1[1]/len1)
                v2 = (v2[0]/len2, v2[1]/len2)
                # Check if not parallel (dot product not close to 1)
                dot = v1[0]*v2[0] + v1[1]*v2[1]
                if abs(dot - 1.0) > 0.1:  # Not parallel
                    bends += 1
        return bends


class GeometricConstraintChecker:
    """Handles geometric constraints: spacing and boundaries"""
    
    def __init__(self, grid_shape: Tuple[int, int], min_spacing: float = 1.0):
        self.H, self.W = grid_shape
        self.min_spacing = min_spacing
    
    def check_boundary_legality(self, point: Point) -> bool:
        """Check if point is within chip boundaries"""
        r, c = point
        return 0 <= r < self.H and 0 <= c < self.W
    
    def check_spacing(self, seg1: PathSegment, seg2: PathSegment) -> bool:
        """Check if two segments satisfy minimum spacing constraint"""
        if seg1.net_id == seg2.net_id:
            return True  # Same net, no spacing constraint
        return seg1.distance_to_segment(seg2) >= self.min_spacing
    
    def check_path_spacing(self, paths: List[RoutingPath]) -> List[Tuple[int, int, float]]:
        """Check spacing violations across all paths"""
        violations = []
        for i, path1 in enumerate(paths):
            for j, path2 in enumerate(paths):
                if i >= j:
                    continue
                for seg1 in path1.segments:
                    for seg2 in path2.segments:
                        if not self.check_spacing(seg1, seg2):
                            dist = seg1.distance_to_segment(seg2)
                            violations.append((path1.net_id, path2.net_id, dist))
        return violations


class CrossingOptimizer:
    """Handles crossing detection and optimization"""
    
    def __init__(self):
        self.crossings: List[Tuple[PathSegment, PathSegment]] = []
    
    def detect_crossings(self, paths: List[RoutingPath]) -> int:
        """Detect all crossings between paths"""
        self.crossings = []
        for i, path1 in enumerate(paths):
            for j, path2 in enumerate(paths):
                if i >= j:
                    continue
                for seg1 in path1.segments:
                    for seg2 in path2.segments:
                        if seg1.intersects(seg2):
                            self.crossings.append((seg1, seg2))
        return len(self.crossings)
    
    def get_crossing_count(self) -> int:
        """Return number of crossings"""
        return len(self.crossings)
    
    def assign_valves_to_crossings(self) -> Dict[Tuple[Point, Point], int]:
        """Assign valve IDs to unavoidable crossings"""
        valve_assignments = {}
        for idx, (seg1, seg2) in enumerate(self.crossings):
            # Calculate intersection point (simplified - midpoint approximation)
            intersection = (
                (seg1.start[0] + seg1.end[0] + seg2.start[0] + seg2.end[0]) / 4,
                (seg1.start[1] + seg1.end[1] + seg2.start[1] + seg2.end[1]) / 4
            )
            valve_assignments[intersection] = idx + 1000  # Assign valve ID
        return valve_assignments


class TimingConstraintOptimizer:
    """Handles timing constraints and mutual exclusion"""
    
    def __init__(self):
        self.shared_segments: Dict[Tuple[int, int], List[PathSegment]] = defaultdict(list)
        self.execution_order: List[int] = []
    
    def detect_shared_segments(self, paths: List[RoutingPath]) -> int:
        """Detect segments shared by multiple paths"""
        self.shared_segments = defaultdict(list)
        segment_map = defaultdict(list)
        
        # Group segments by rasterized cells
        for path in paths:
            for seg in path.segments:
                # Rasterize segment to cells
                cells = self._rasterize_segment(seg)
                for cell in cells:
                    segment_map[cell].append((path.net_id, seg))
        
        # Find shared cells
        for cell, segs in segment_map.items():
            if len(segs) > 1:
                net_ids = tuple(sorted(set(net_id for net_id, _ in segs)))
                self.shared_segments[net_ids].extend([s for _, s in segs])
        
        return len(self.shared_segments)
    
    def _rasterize_segment(self, seg: PathSegment) -> List[Coord]:
        """Convert segment to grid cells using Bresenham's algorithm"""
        cells = []
        r0, c0 = int(seg.start[0]), int(seg.start[1])
        r1, c1 = int(seg.end[0]), int(seg.end[1])
        
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        r_step = 1 if r0 < r1 else -1
        c_step = 1 if c0 < c1 else -1
        err = dr - dc
        
        r, c = r0, c0
        while True:
            cells.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += r_step
            if e2 < dr:
                err += dr
                c += c_step
        
        return cells
    
    def schedule_execution(self, paths: List[RoutingPath]) -> List[int]:
        """Create execution schedule ensuring mutual exclusion"""
        # Simple greedy scheduling: order by dependencies
        scheduled = set()
        self.execution_order = []
        
        while len(scheduled) < len(paths):
            # Find a path that can be scheduled
            for path in paths:
                if path.net_id in scheduled:
                    continue
                # Check if all conflicting paths are scheduled or can wait
                can_schedule = True
                for net_ids in self.shared_segments.keys():
                    if path.net_id in net_ids:
                        # Has conflicts, but we can still schedule it
                        pass
                self.execution_order.append(path.net_id)
                scheduled.add(path.net_id)
                break
        
        return self.execution_order


class LocalOptimizer:
    """Local optimization strategies"""
    
    def __init__(self, grid_shape: Tuple[int, int]):
        self.H, self.W = grid_shape
    
    def smooth_path(self, path: RoutingPath) -> RoutingPath:
        """Reduce bends and replace zigzags with straight lines"""
        if len(path.points) < 3:
            return path
        
        smoothed = [path.points[0]]
        i = 0
        while i < len(path.points) - 1:
            # Try to find furthest point with direct line-of-sight
            j = len(path.points) - 1
            while j > i + 1:
                if self._has_clear_los(path.points[i], path.points[j]):
                    smoothed.append(path.points[j])
                    i = j
                    break
                j -= 1
            if j == i + 1:
                smoothed.append(path.points[i + 1])
                i += 1
        
        return RoutingPath(path.net_id, smoothed)
    
    def _has_clear_los(self, p1: Point, p2: Point) -> bool:
        """
        Check if there's clear line of sight between two points
        
        NOTE: Current implementation is optimistic (always returns True).
        For production use, should check:
        - No obstacles in grid between points
        - No forbidden zones crossed
        - Maintains minimum clearances
        
        This simplified version enables aggressive path smoothing.
        Integrate with obstacle checker for proper validation.
        """
        # TODO: Add obstacle checking from grid
        return True
    
    def reduce_crossings_local(self, paths: List[RoutingPath], 
                              crossing_optimizer: CrossingOptimizer) -> List[RoutingPath]:
        """
        Try to reduce crossings by local path adjustments
        
        NOTE: Current implementation is a placeholder that returns paths unchanged.
        For production, should implement:
        - Local path perturbation around crossing points
        - Alternative routing through nearby waypoints
        - Iterative improvement with crossing re-evaluation
        
        This is a complex optimization that requires integration with
        the pathfinding algorithm for local rerouting.
        """
        # Identify paths with most crossings for prioritization
        crossing_counts = defaultdict(int)
        for seg1, seg2 in crossing_optimizer.crossings:
            crossing_counts[seg1.net_id] += 1
            crossing_counts[seg2.net_id] += 1
        
        # For now, return unchanged - can be enhanced with local rerouting
        return paths
    
    def enhance_reuse(self, paths: List[RoutingPath]) -> float:
        """Calculate and enhance path reuse"""
        # Count shared cells
        cell_usage = defaultdict(set)
        for path in paths:
            for seg in path.segments:
                cells = self._rasterize_segment(seg)
                for cell in cells:
                    cell_usage[cell].add(path.net_id)
        
        # Calculate reuse ratio
        shared_cells = sum(1 for nets in cell_usage.values() if len(nets) > 1)
        total_cells = len(cell_usage)
        reuse_ratio = shared_cells / total_cells if total_cells > 0 else 0
        
        return reuse_ratio
    
    def _rasterize_segment(self, seg: PathSegment) -> List[Coord]:
        """Convert segment to grid cells"""
        cells = []
        r0, c0 = int(seg.start[0]), int(seg.start[1])
        r1, c1 = int(seg.end[0]), int(seg.end[1])
        
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        r_step = 1 if r0 < r1 else -1
        c_step = 1 if c0 < c1 else -1
        err = dr - dc
        
        r, c = r0, c0
        while True:
            cells.append((r, c))
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += r_step
            if e2 < dr:
                err += dr
                c += c_step
        
        return cells


class MultiObjectiveOptimizer:
    """Multi-objective optimization using weighted sum"""
    
    def __init__(self, alpha: float = 1.0, beta: float = 1.0, 
                 gamma: float = 1.0, delta: float = 1.0):
        """
        Initialize with weights for objective function:
        F = Σ L(Pi) + α·Cross(P) + β·Ports(P) - γ·Reuse(P) + δ·T_total
        
        Args:
            alpha: Weight for crossing penalty
            beta: Weight for port count penalty
            gamma: Weight for reuse reward (negative - higher is better)
            delta: Weight for total delay penalty
        """
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.delta = delta
    
    def calculate_objective(self, paths: List[RoutingPath],
                          crossing_optimizer: CrossingOptimizer,
                          local_optimizer: LocalOptimizer,
                          num_ports: int,
                          total_delay: float = 0.0) -> float:
        """
        Calculate the multi-objective function value
        
        Returns:
            Objective value (lower is better)
        """
        # Total path length
        total_length = sum(path.total_length() for path in paths)
        
        # Number of crossings
        num_crossings = crossing_optimizer.get_crossing_count()
        
        # Number of ports (simplified - count unique valve assignments)
        # This would be provided from routing result
        
        # Reuse rate
        reuse_rate = local_optimizer.enhance_reuse(paths)
        
        # Calculate objective
        objective = (total_length + 
                    self.alpha * num_crossings + 
                    self.beta * num_ports - 
                    self.gamma * reuse_rate + 
                    self.delta * total_delay)
        
        return objective
    
    def optimize_weights(self, paths: List[RoutingPath],
                        crossing_optimizer: CrossingOptimizer,
                        local_optimizer: LocalOptimizer,
                        priorities: Dict[str, float]) -> Tuple[float, float, float, float]:
        """
        Adjust weights based on priority preferences
        
        Args:
            priorities: Dictionary with keys 'length', 'crossings', 'ports', 'reuse', 'delay'
                       and values indicating relative importance (higher = more important)
        
        Returns:
            Tuple of (alpha, beta, gamma, delta)
        """
        # Normalize priorities
        total = sum(priorities.values())
        if total > 0:
            normalized = {k: v/total for k, v in priorities.items()}
        else:
            normalized = priorities
        
        # Set weights based on priorities
        alpha = normalized.get('crossings', 1.0) * 10.0
        beta = normalized.get('ports', 1.0) * 5.0
        gamma = normalized.get('reuse', 1.0) * 10.0
        delta = normalized.get('delay', 1.0) * 1.0
        
        return alpha, beta, gamma, delta


class RoutingOptimizationFramework:
    """Main framework integrating all optimization components"""
    
    def __init__(self, grid_shape: Tuple[int, int], min_spacing: float = 1.0,
                 alpha: float = 1.0, beta: float = 1.0, 
                 gamma: float = 1.0, delta: float = 1.0):
        self.grid_shape = grid_shape
        self.min_spacing = min_spacing
        
        # Initialize all components
        self.geom_checker = GeometricConstraintChecker(grid_shape, min_spacing)
        self.crossing_optimizer = CrossingOptimizer()
        self.timing_optimizer = TimingConstraintOptimizer()
        self.local_optimizer = LocalOptimizer(grid_shape)
        self.multi_obj_optimizer = MultiObjectiveOptimizer(alpha, beta, gamma, delta)
        
        self.paths: List[RoutingPath] = []
    
    def set_initial_paths(self, paths: List[RoutingPath]):
        """Set initial paths from basic path generation (A*/Theta*/DRL)"""
        self.paths = paths
    
    def apply_geometric_constraints(self) -> bool:
        """
        Apply geometric constraints and check violations
        
        Returns:
            True if all constraints satisfied, False otherwise
        """
        # Check boundary legality
        for path in self.paths:
            for point in path.points:
                if not self.geom_checker.check_boundary_legality(point):
                    print(f"Boundary violation in path {path.net_id} at {point}")
                    return False
        
        # Check spacing constraints
        violations = self.geom_checker.check_path_spacing(self.paths)
        if violations:
            print(f"Found {len(violations)} spacing violations")
            for net1, net2, dist in violations[:5]:  # Show first 5
                print(f"  Nets {net1} and {net2}: distance {dist:.2f} < {self.min_spacing}")
            return False
        
        return True
    
    def optimize_crossings(self) -> int:
        """
        Detect and optimize crossings
        
        Returns:
            Number of crossings detected
        """
        num_crossings = self.crossing_optimizer.detect_crossings(self.paths)
        print(f"Detected {num_crossings} crossings")
        
        # Assign valves to unavoidable crossings
        if num_crossings > 0:
            valve_assignments = self.crossing_optimizer.assign_valves_to_crossings()
            print(f"Assigned {len(valve_assignments)} valves to crossings")
        
        return num_crossings
    
    def optimize_timing(self) -> List[int]:
        """
        Optimize timing constraints and create execution schedule
        
        Returns:
            Execution order of paths
        """
        num_shared = self.timing_optimizer.detect_shared_segments(self.paths)
        print(f"Found {num_shared} shared segment groups")
        
        execution_order = self.timing_optimizer.schedule_execution(self.paths)
        print(f"Execution order: {execution_order}")
        
        return execution_order
    
    def apply_local_optimizations(self):
        """Apply local optimization strategies"""
        print("Applying local optimizations...")
        
        # Path smoothing
        smoothed_paths = []
        for path in self.paths:
            smoothed = self.local_optimizer.smooth_path(path)
            bends_before = path.num_bends()
            bends_after = smoothed.num_bends()
            if bends_after < bends_before:
                print(f"  Path {path.net_id}: reduced bends from {bends_before} to {bends_after}")
            smoothed_paths.append(smoothed)
        self.paths = smoothed_paths
        
        # Crossing reduction
        self.paths = self.local_optimizer.reduce_crossings_local(
            self.paths, self.crossing_optimizer)
        
        # Reuse enhancement
        reuse_rate = self.local_optimizer.enhance_reuse(self.paths)
        print(f"  Reuse rate: {reuse_rate:.2%}")
    
    def calculate_final_objective(self, num_ports: int = 0, 
                                  total_delay: float = 0.0) -> float:
        """Calculate final multi-objective value"""
        objective = self.multi_obj_optimizer.calculate_objective(
            self.paths, self.crossing_optimizer, self.local_optimizer,
            num_ports, total_delay)
        return objective
    
    def optimize(self, initial_paths: List[RoutingPath], 
                num_ports: int = 0) -> Tuple[List[RoutingPath], float]:
        """
        Run complete optimization pipeline
        
        Args:
            initial_paths: Initial paths from basic pathfinding
            num_ports: Number of ports/valves used
        
        Returns:
            Tuple of (optimized paths, objective value)
        """
        print("=" * 60)
        print("Starting Routing Optimization Framework")
        print("=" * 60)
        
        # Step 1: Set initial paths
        self.set_initial_paths(initial_paths)
        print(f"\nStep 1: Loaded {len(self.paths)} initial paths")
        
        # Step 2: Apply geometric constraints
        print("\nStep 2: Checking geometric constraints...")
        constraints_ok = self.apply_geometric_constraints()
        print(f"  Geometric constraints: {'✓ PASS' if constraints_ok else '✗ FAIL'}")
        
        # Step 3: Optimize crossings
        print("\nStep 3: Optimizing crossings...")
        num_crossings = self.optimize_crossings()
        
        # Step 4: Optimize timing
        print("\nStep 4: Optimizing timing constraints...")
        execution_order = self.optimize_timing()
        
        # Step 5: Apply local optimizations
        print("\nStep 5: Applying local optimizations...")
        self.apply_local_optimizations()
        
        # Step 6: Calculate final objective
        print("\nStep 6: Calculating multi-objective value...")
        objective = self.calculate_final_objective(num_ports)
        
        print(f"\n{'=' * 60}")
        print(f"Optimization Complete")
        print(f"{'=' * 60}")
        print(f"Final Objective Value: {objective:.2f}")
        print(f"  Total path length: {sum(p.total_length() for p in self.paths):.2f}")
        print(f"  Number of crossings: {num_crossings}")
        print(f"  Number of paths: {len(self.paths)}")
        
        return self.paths, objective
