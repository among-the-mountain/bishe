#!/usr/bin/env python3
"""
Unit tests for routing optimization framework
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from routing_optimizer import (
    PathSegment, RoutingPath, Point,
    GeometricConstraintChecker,
    CrossingOptimizer,
    TimingConstraintOptimizer,
    LocalOptimizer,
    MultiObjectiveOptimizer,
    RoutingOptimizationFramework
)
import numpy as np


def test_path_segment():
    """Test PathSegment functionality"""
    print("Testing PathSegment...")
    
    # Test length calculation
    seg = PathSegment((0, 0), (3, 4), 0)
    assert abs(seg.length() - 5.0) < 0.01, "Length calculation failed"
    
    # Test intersection detection
    seg1 = PathSegment((0, 0), (2, 2), 0)
    seg2 = PathSegment((0, 2), (2, 0), 1)
    assert seg1.intersects(seg2), "Intersection detection failed"
    
    # Test non-intersection
    seg3 = PathSegment((0, 0), (1, 1), 0)
    seg4 = PathSegment((2, 2), (3, 3), 1)
    assert not seg3.intersects(seg4), "Non-intersection detection failed"
    
    print("✓ PathSegment tests passed")


def test_routing_path():
    """Test RoutingPath functionality"""
    print("Testing RoutingPath...")
    
    # Test path creation
    points = [(0, 0), (1, 1), (2, 1), (3, 3)]
    path = RoutingPath(0, points)
    
    assert len(path.segments) == 3, "Segment count wrong"
    assert path.total_length() > 0, "Total length calculation failed"
    
    # Test bend counting
    straight_path = RoutingPath(1, [(0, 0), (5, 5)])
    assert straight_path.num_bends() == 0, "Straight path should have no bends"
    
    bent_path = RoutingPath(2, [(0, 0), (1, 1), (2, 0)])
    assert bent_path.num_bends() > 0, "Bent path should have bends"
    
    print("✓ RoutingPath tests passed")


def test_geometric_constraint_checker():
    """Test GeometricConstraintChecker"""
    print("Testing GeometricConstraintChecker...")
    
    checker = GeometricConstraintChecker((10, 10), 2.0)
    
    # Test boundary checking
    assert checker.check_boundary_legality((5, 5)), "Valid point rejected"
    assert not checker.check_boundary_legality((15, 5)), "Invalid point accepted"
    assert not checker.check_boundary_legality((-1, 5)), "Negative coord accepted"
    
    # Test spacing
    seg1 = PathSegment((0, 0), (1, 1), 0)
    seg2 = PathSegment((5, 5), (6, 6), 1)
    assert checker.check_spacing(seg1, seg2), "Far segments should pass spacing"
    
    print("✓ GeometricConstraintChecker tests passed")


def test_crossing_optimizer():
    """Test CrossingOptimizer"""
    print("Testing CrossingOptimizer...")
    
    optimizer = CrossingOptimizer()
    
    # Create crossing paths
    paths = [
        RoutingPath(0, [(0, 0), (2, 2)]),
        RoutingPath(1, [(0, 2), (2, 0)])
    ]
    
    num_crossings = optimizer.detect_crossings(paths)
    assert num_crossings > 0, "Failed to detect crossing"
    assert optimizer.get_crossing_count() == num_crossings, "Count mismatch"
    
    # Test valve assignment
    valves = optimizer.assign_valves_to_crossings()
    assert len(valves) > 0, "No valves assigned"
    
    print("✓ CrossingOptimizer tests passed")


def test_timing_optimizer():
    """Test TimingConstraintOptimizer"""
    print("Testing TimingConstraintOptimizer...")
    
    optimizer = TimingConstraintOptimizer()
    
    # Create paths with shared segment
    paths = [
        RoutingPath(0, [(0, 0), (1, 1), (2, 2)]),
        RoutingPath(1, [(0, 2), (1, 1), (2, 0)])
    ]
    
    num_shared = optimizer.detect_shared_segments(paths)
    # Should detect shared cell at (1, 1)
    
    # Test execution scheduling
    order = optimizer.schedule_execution(paths)
    assert len(order) == len(paths), "Execution order incomplete"
    
    print("✓ TimingConstraintOptimizer tests passed")


def test_local_optimizer():
    """Test LocalOptimizer"""
    print("Testing LocalOptimizer...")
    
    optimizer = LocalOptimizer((10, 10))
    
    # Test path smoothing
    zigzag_path = RoutingPath(0, [(0, 0), (1, 1), (2, 0), (3, 1)])
    smoothed = optimizer.smooth_path(zigzag_path)
    
    assert len(smoothed.points) <= len(zigzag_path.points), "Smoothing failed"
    
    # Test reuse calculation
    paths = [
        RoutingPath(0, [(0, 0), (2, 2)]),
        RoutingPath(1, [(1, 1), (3, 3)])
    ]
    reuse = optimizer.enhance_reuse(paths)
    assert 0 <= reuse <= 1, "Reuse rate out of range"
    
    print("✓ LocalOptimizer tests passed")


def test_multi_objective_optimizer():
    """Test MultiObjectiveOptimizer"""
    print("Testing MultiObjectiveOptimizer...")
    
    optimizer = MultiObjectiveOptimizer(alpha=1.0, beta=1.0, gamma=1.0, delta=1.0)
    
    # Create simple test scenario
    paths = [RoutingPath(0, [(0, 0), (5, 5)])]
    crossing_opt = CrossingOptimizer()
    crossing_opt.detect_crossings(paths)
    local_opt = LocalOptimizer((10, 10))
    
    objective = optimizer.calculate_objective(paths, crossing_opt, local_opt, 10, 0.0)
    assert objective > 0, "Objective calculation failed"
    
    # Test weight optimization
    priorities = {'crossings': 2.0, 'reuse': 1.0, 'ports': 1.0, 'delay': 1.0}
    weights = optimizer.optimize_weights(paths, crossing_opt, local_opt, priorities)
    assert len(weights) == 4, "Weight tuple wrong size"
    
    print("✓ MultiObjectiveOptimizer tests passed")


def test_integration():
    """Test integrated framework"""
    print("Testing RoutingOptimizationFramework...")
    
    grid_shape = (20, 20)
    framework = RoutingOptimizationFramework(grid_shape, min_spacing=1.0)
    
    # Create simple paths
    paths = [
        RoutingPath(0, [(2, 2), (5, 5)]),
        RoutingPath(1, [(2, 5), (5, 2)])
    ]
    
    # Run optimization
    optimized, objective = framework.optimize(paths, num_ports=10)
    
    assert len(optimized) == len(paths), "Lost paths during optimization"
    assert objective >= 0, "Invalid objective value"
    
    print("✓ RoutingOptimizationFramework tests passed")


def run_all_tests():
    """Run all tests"""
    print("\n" + "="*70)
    print("RUNNING ROUTING OPTIMIZATION FRAMEWORK TESTS")
    print("="*70 + "\n")
    
    tests = [
        test_path_segment,
        test_routing_path,
        test_geometric_constraint_checker,
        test_crossing_optimizer,
        test_timing_optimizer,
        test_local_optimizer,
        test_multi_objective_optimizer,
        test_integration,
    ]
    
    failed = []
    
    for test in tests:
        try:
            test()
        except AssertionError as e:
            print(f"✗ {test.__name__} FAILED: {e}")
            failed.append(test.__name__)
        except Exception as e:
            print(f"✗ {test.__name__} ERROR: {e}")
            failed.append(test.__name__)
    
    print("\n" + "="*70)
    if failed:
        print(f"FAILED: {len(failed)} tests failed")
        for name in failed:
            print(f"  - {name}")
        return False
    else:
        print("✅ ALL TESTS PASSED")
        print("="*70)
        return True


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
