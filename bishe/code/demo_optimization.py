#!/usr/bin/env python3
"""
Demonstration of Routing Optimization Framework

This script demonstrates all six phases of the optimization framework:
1. Basic Path Generation (A*/Theta*/DRL)
2. Geometric Constraint Embedding
3. Crossing Constraint Optimization
4. Timing Constraint Optimization
5. Local Optimization Strategies
6. Multi-objective Optimization Function
"""

import sys
import os
import argparse
import numpy as np

# Add path to import modules
sys.path.insert(0, os.path.dirname(__file__))

from routing_optimizer import (
    RoutingPath, PathSegment, Point,
    GeometricConstraintChecker,
    CrossingOptimizer,
    TimingConstraintOptimizer,
    LocalOptimizer,
    MultiObjectiveOptimizer,
    RoutingOptimizationFramework
)
from optimized_router import OptimizedRouter, parse_benchmark


def demonstrate_phase_by_phase(benchmark_path: str):
    """
    Demonstrate each phase of the optimization framework step by step
    """
    print("\n" + "="*80)
    print("ROUTING OPTIMIZATION FRAMEWORK - DETAILED DEMONSTRATION")
    print("="*80)
    
    # Load benchmark
    print("\nLoading benchmark...")
    grid, nets = parse_benchmark(benchmark_path)
    print(f"✓ Grid: {grid.shape[0]} x {grid.shape[1]}")
    print(f"✓ Nets to route: {len(nets)}")
    
    H, W = grid.shape
    
    # =========================================================================
    print("\n" + "="*80)
    print("PHASE 1: BASIC PATH GENERATION")
    print("="*80)
    print("\nUsing any-angle search algorithms (A*/Theta*/DRL) to generate initial paths")
    print("These paths ensure connectivity but may not satisfy all constraints\n")
    
    # Generate simple initial paths for demonstration
    initial_paths = []
    for net_id, net in enumerate(nets):
        valve_positions = []
        for valve_id in net:
            abs_vid = abs(valve_id)
            if abs_vid > 0:
                coords = np.argwhere(grid == abs_vid)
                if len(coords) > 0:
                    r = float(np.mean(coords[:, 0]))
                    c = float(np.mean(coords[:, 1]))
                    valve_positions.append((r, c))
        
        if len(valve_positions) >= 2:
            path = RoutingPath(net_id, valve_positions)
            initial_paths.append(path)
    
    print(f"✓ Generated {len(initial_paths)} initial paths")
    print(f"✓ Total initial path length: {sum(p.total_length() for p in initial_paths):.2f}")
    
    # =========================================================================
    print("\n" + "="*80)
    print("PHASE 2: GEOMETRIC CONSTRAINT EMBEDDING")
    print("="*80)
    print("\nEmbedding geometric constraints:")
    print("  - Minimum spacing between paths (d_min)")
    print("  - Boundary legality (paths within routable area)")
    print("  - Local re-search when constraints violated\n")
    
    min_spacing = 1.0
    geom_checker = GeometricConstraintChecker((H, W), min_spacing)
    
    # Check boundary legality
    boundary_violations = 0
    for path in initial_paths:
        for point in path.points:
            if not geom_checker.check_boundary_legality(point):
                boundary_violations += 1
    
    print(f"✓ Boundary check: {boundary_violations} violations found")
    
    # Check spacing constraints
    spacing_violations = geom_checker.check_path_spacing(initial_paths)
    print(f"✓ Spacing check (d_min={min_spacing}): {len(spacing_violations)} violations")
    if spacing_violations:
        print(f"  Example violations:")
        for net1, net2, dist in spacing_violations[:3]:
            print(f"    - Nets {net1} ↔ {net2}: distance {dist:.2f} < {min_spacing}")
    
    # =========================================================================
    print("\n" + "="*80)
    print("PHASE 3: CROSSING CONSTRAINT OPTIMIZATION")
    print("="*80)
    print("\nDetecting and optimizing path crossings:")
    print("  - Detect all path intersections")
    print("  - Assign valve control for unavoidable crossings")
    print("  - Consider local rerouting if too many crossings\n")
    
    crossing_optimizer = CrossingOptimizer()
    num_crossings = crossing_optimizer.detect_crossings(initial_paths)
    
    print(f"✓ Detected {num_crossings} crossings")
    
    if num_crossings > 0:
        valve_assignments = crossing_optimizer.assign_valves_to_crossings()
        print(f"✓ Assigned {len(valve_assignments)} control valves to crossings")
        print(f"  First 3 valve assignments:")
        for i, (pos, valve_id) in enumerate(list(valve_assignments.items())[:3]):
            print(f"    - Valve {valve_id} at position ({pos[0]:.1f}, {pos[1]:.1f})")
    
    # =========================================================================
    print("\n" + "="*80)
    print("PHASE 4: TIMING CONSTRAINT OPTIMIZATION")
    print("="*80)
    print("\nOptimizing for mutual exclusion:")
    print("  - Shared path segments cannot execute simultaneously")
    print("  - Create execution schedule with time variables")
    print("  - Adjust execution order to resolve conflicts\n")
    
    timing_optimizer = TimingConstraintOptimizer()
    num_shared = timing_optimizer.detect_shared_segments(initial_paths)
    
    print(f"✓ Detected {num_shared} shared segment groups")
    
    execution_order = timing_optimizer.schedule_execution(initial_paths)
    print(f"✓ Created execution schedule:")
    print(f"  Order: {execution_order[:10]}{'...' if len(execution_order) > 10 else ''}")
    
    # =========================================================================
    print("\n" + "="*80)
    print("PHASE 5: LOCAL OPTIMIZATION STRATEGIES")
    print("="*80)
    print("\nApplying local optimizations:")
    print("  - Path smoothing: reduce bends, replace zigzags")
    print("  - Crossing resolution: offset paths to reduce valves")
    print("  - Reuse enhancement: merge adjacent segments")
    print("  - Connection point correction\n")
    
    local_optimizer = LocalOptimizer((H, W))
    
    # Path smoothing
    total_bends_before = sum(p.num_bends() for p in initial_paths)
    smoothed_paths = []
    for path in initial_paths:
        smoothed = local_optimizer.smooth_path(path)
        smoothed_paths.append(smoothed)
    total_bends_after = sum(p.num_bends() for p in smoothed_paths)
    
    print(f"✓ Path smoothing completed:")
    print(f"  Bends before: {total_bends_before}")
    print(f"  Bends after:  {total_bends_after}")
    print(f"  Reduction:    {total_bends_before - total_bends_after} ({(1-total_bends_after/max(total_bends_before,1))*100:.1f}%)")
    
    # Reuse enhancement
    reuse_rate = local_optimizer.enhance_reuse(smoothed_paths)
    print(f"\n✓ Reuse rate: {reuse_rate:.2%}")
    print(f"  Higher reuse → better resource utilization")
    
    # =========================================================================
    print("\n" + "="*80)
    print("PHASE 6: MULTI-OBJECTIVE OPTIMIZATION")
    print("="*80)
    print("\nUnified objective function:")
    print("  F = Σ L(Pᵢ) + α·Cross(P) + β·Ports(P) - γ·Reuse(P) + δ·T_total")
    print("\nComponents:")
    print("  - L(Pᵢ):     Total path length (minimize)")
    print("  - Cross(P):  Number of crossings (minimize)")
    print("  - Ports(P):  Number of ports/valves (minimize)")
    print("  - Reuse(P):  Reuse rate (maximize)")
    print("  - T_total:   Total timing delay (minimize)\n")
    
    # Calculate with different weight settings
    multi_obj = MultiObjectiveOptimizer(alpha=1.0, beta=1.0, gamma=1.0, delta=1.0)
    
    # Count ports
    all_valves = set()
    for net in nets:
        for v in net:
            if abs(v) > 0:
                all_valves.add(abs(v))
    num_ports = len(all_valves)
    
    # Recalculate crossings on smoothed paths
    crossing_optimizer_final = CrossingOptimizer()
    crossing_optimizer_final.detect_crossings(smoothed_paths)
    
    objective = multi_obj.calculate_objective(
        smoothed_paths, crossing_optimizer_final, local_optimizer,
        num_ports, total_delay=0.0
    )
    
    print(f"✓ Calculated multi-objective value: {objective:.2f}")
    print(f"\n  Breakdown:")
    print(f"    Total path length:   {sum(p.total_length() for p in smoothed_paths):.2f}")
    print(f"    Crossings:           {crossing_optimizer_final.get_crossing_count()}")
    print(f"    Ports:               {num_ports}")
    print(f"    Reuse rate:          {reuse_rate:.2%}")
    
    # Show effect of different priorities
    print(f"\n✓ Weight optimization scenarios:")
    
    scenarios = [
        ("Balanced", {'length': 1.0, 'crossings': 1.0, 'ports': 1.0, 'reuse': 1.0, 'delay': 1.0}),
        ("Minimize crossings", {'length': 0.5, 'crossings': 2.0, 'ports': 0.5, 'reuse': 1.0, 'delay': 0.5}),
        ("Maximize reuse", {'length': 0.5, 'crossings': 0.5, 'ports': 0.5, 'reuse': 2.0, 'delay': 0.5}),
        ("Minimize length", {'length': 2.0, 'crossings': 0.5, 'ports': 0.5, 'reuse': 0.5, 'delay': 0.5}),
    ]
    
    for scenario_name, priorities in scenarios:
        alpha, beta, gamma, delta = multi_obj.optimize_weights(
            smoothed_paths, crossing_optimizer_final, local_optimizer, priorities
        )
        opt = MultiObjectiveOptimizer(alpha, beta, gamma, delta)
        obj_val = opt.calculate_objective(
            smoothed_paths, crossing_optimizer_final, local_optimizer,
            num_ports, 0.0
        )
        print(f"    {scenario_name:20s}: objective = {obj_val:.2f} (α={alpha:.1f}, β={beta:.1f}, γ={gamma:.1f}, δ={delta:.1f})")
    
    # =========================================================================
    print("\n" + "="*80)
    print("SUMMARY: ALGORITHM REASONING AND OPTIMIZATION FLOW")
    print("="*80)
    print("""
The optimization framework follows this reasoning:

1. INITIAL PATH GENERATION
   → Generate connected paths using any-angle search (A*/Theta*)
   → Ensure basic connectivity without full constraint satisfaction

2. CONSTRAINT EMBEDDING
   → Embed geometric constraints (spacing, boundaries)
   → Identify violations for local re-search
   
3. CROSSING OPTIMIZATION
   → Detect all path intersections
   → Assign valves for unavoidable crossings
   → Consider rerouting when crossings excessive

4. TIMING OPTIMIZATION
   → Ensure mutual exclusion on shared segments
   → Schedule execution order to avoid conflicts
   
5. LOCAL REFINEMENT
   → Smooth paths to reduce bends
   → Resolve crossings where possible
   → Enhance segment reuse for efficiency
   
6. MULTI-OBJECTIVE BALANCE
   → Unified objective function balances all goals
   → Weight adjustment prioritizes different objectives
   → Achieve optimal trade-off solution

Result: Optimized routing that balances path length, crossings, 
        ports, reuse, and timing constraints.
""")
    
    print("="*80)


def compare_weight_configurations(benchmark_path: str):
    """
    Compare different weight configurations for the multi-objective function
    """
    print("\n" + "="*80)
    print("WEIGHT CONFIGURATION COMPARISON")
    print("="*80)
    
    grid, nets = parse_benchmark(benchmark_path)
    
    configurations = [
        ("Default (Balanced)", {'crossings': 1.0, 'ports': 1.0, 'reuse': 1.0, 'delay': 1.0}),
        ("Minimize Crossings", {'crossings': 3.0, 'ports': 1.0, 'reuse': 1.0, 'delay': 1.0}),
        ("Maximize Reuse", {'crossings': 1.0, 'ports': 1.0, 'reuse': 3.0, 'delay': 1.0}),
        ("Minimize Ports", {'crossings': 1.0, 'ports': 3.0, 'reuse': 1.0, 'delay': 1.0}),
        ("Quality-First", {'crossings': 2.0, 'ports': 2.0, 'reuse': 2.0, 'delay': 0.5}),
    ]
    
    results = []
    
    for config_name, weights in configurations:
        print(f"\nTesting: {config_name}")
        print(f"  Weights: {weights}")
        
        router = OptimizedRouter(grid, nets, min_spacing=1.0, 
                                optimization_weights=weights)
        router.route_with_optimization()
        report = router.get_optimization_report()
        
        results.append((config_name, report))
        
        print(f"  → Crossings: {report['num_crossings']}")
        print(f"  → Path length: {report['total_length']:.2f}")
        print(f"  → Reuse rate: {report['reuse_rate']:.2%}")
        print(f"  → Objective: {report['final_objective']:.2f}")
    
    # Print comparison table
    print("\n" + "="*80)
    print("COMPARISON TABLE")
    print("="*80)
    print(f"{'Configuration':<20} {'Crossings':>10} {'Length':>12} {'Reuse':>10} {'Objective':>12}")
    print("-"*80)
    for config_name, report in results:
        print(f"{config_name:<20} {report['num_crossings']:>10} "
              f"{report['total_length']:>12.2f} {report['reuse_rate']:>9.1%} "
              f"{report['final_objective']:>12.2f}")
    print("="*80)


def main():
    parser = argparse.ArgumentParser(
        description='Demonstrate Routing Optimization Framework',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run detailed phase-by-phase demonstration
  python demo_optimization.py ../benchmarks/R0.txt --demo
  
  # Compare different weight configurations
  python demo_optimization.py ../benchmarks/R0.txt --compare
  
  # Run both
  python demo_optimization.py ../benchmarks/R0.txt --demo --compare
        """
    )
    
    parser.add_argument('benchmark', help='Path to benchmark file')
    parser.add_argument('--demo', action='store_true',
                       help='Run phase-by-phase demonstration')
    parser.add_argument('--compare', action='store_true',
                       help='Compare different weight configurations')
    
    args = parser.parse_args()
    
    # Default to demo if nothing specified
    if not args.demo and not args.compare:
        args.demo = True
    
    if args.demo:
        demonstrate_phase_by_phase(args.benchmark)
    
    if args.compare:
        compare_weight_configurations(args.benchmark)


if __name__ == '__main__':
    main()
