#!/usr/bin/env python3
"""
Optimized Router Integration
Combines the existing geometric router with the new optimization framework
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'reference'))

import numpy as np
from typing import List, Tuple, Dict
from routing_optimizer import (
    RoutingPath, RoutingOptimizationFramework, Point
)


class OptimizedRouter:
    """
    Integrated router that combines basic pathfinding with optimization
    """
    
    def __init__(self, grid: np.ndarray, nets: List[List[int]], 
                 min_spacing: float = 1.0,
                 optimization_weights: Dict[str, float] = None):
        """
        Initialize the optimized router
        
        Args:
            grid: The chip layout grid
            nets: List of nets to route
            min_spacing: Minimum spacing constraint
            optimization_weights: Weights for multi-objective optimization
        """
        self.grid = grid
        self.nets = nets
        self.H, self.W = grid.shape
        
        # Default optimization weights
        if optimization_weights is None:
            optimization_weights = {
                'crossings': 1.0,
                'ports': 1.0,
                'reuse': 1.0,
                'delay': 1.0
            }
        
        # Initialize optimization framework
        alpha, beta, gamma, delta = self._weights_from_priorities(optimization_weights)
        self.optimizer = RoutingOptimizationFramework(
            (self.H, self.W), min_spacing, alpha, beta, gamma, delta
        )
        
        self.routed_paths: List[RoutingPath] = []
        self.final_objective: float = 0.0
    
    def _weights_from_priorities(self, priorities: Dict[str, float]) -> Tuple[float, float, float, float]:
        """Convert priority dict to weight tuple"""
        total = sum(priorities.values())
        if total > 0:
            normalized = {k: v/total for k, v in priorities.items()}
        else:
            normalized = priorities
        
        alpha = normalized.get('crossings', 1.0) * 10.0
        beta = normalized.get('ports', 1.0) * 5.0
        gamma = normalized.get('reuse', 1.0) * 10.0
        delta = normalized.get('delay', 1.0) * 1.0
        
        return alpha, beta, gamma, delta
    
    def generate_initial_paths_astar(self) -> List[RoutingPath]:
        """
        Generate initial paths using A* algorithm
        This is a placeholder - in practice, would use the existing router
        or implement A*/Theta* here
        """
        # For now, create simple straight-line paths as demonstration
        # In production, this would call the actual pathfinding algorithm
        paths = []
        
        for net_id, net in enumerate(self.nets):
            # Find valve positions for this net
            valve_positions = []
            for valve_id in net:
                abs_vid = abs(valve_id)
                if abs_vid > 0:
                    coords = np.argwhere(self.grid == abs_vid)
                    if len(coords) > 0:
                        # Use center of valve cells
                        r = float(np.mean(coords[:, 0]))
                        c = float(np.mean(coords[:, 1]))
                        valve_positions.append((r, c))
            
            # Create path through valve positions
            if len(valve_positions) >= 2:
                path = RoutingPath(net_id, valve_positions)
                paths.append(path)
        
        return paths
    
    def route_with_optimization(self, use_existing_router: bool = False) -> bool:
        """
        Complete routing with optimization pipeline
        
        Args:
            use_existing_router: If True, integrate with existing router
        
        Returns:
            True if routing successful
        """
        print("\n" + "="*70)
        print("OPTIMIZED ROUTING PIPELINE")
        print("="*70)
        
        # Step 1: Generate initial paths
        print("\nPhase 1: Basic Path Generation (A*/Theta*)")
        print("-" * 70)
        
        if use_existing_router:
            # Would integrate with existing SimpleGeometricRouter here
            print("Using existing geometric router for initial paths...")
            initial_paths = self.generate_initial_paths_astar()
        else:
            print("Generating initial paths with A*...")
            initial_paths = self.generate_initial_paths_astar()
        
        print(f"Generated {len(initial_paths)} initial paths")
        
        # Count total ports (unique valves used)
        all_valves = set()
        for net in self.nets:
            for v in net:
                if abs(v) > 0:
                    all_valves.add(abs(v))
        num_ports = len(all_valves)
        
        # Step 2: Run optimization framework
        print("\nPhase 2: Comprehensive Optimization")
        print("-" * 70)
        
        optimized_paths, objective = self.optimizer.optimize(initial_paths, num_ports)
        
        self.routed_paths = optimized_paths
        self.final_objective = objective
        
        return True
    
    def get_optimization_report(self) -> Dict:
        """
        Get detailed optimization report
        
        Returns:
            Dictionary with optimization metrics
        """
        if not self.routed_paths:
            return {}
        
        # Calculate metrics
        total_length = sum(p.total_length() for p in self.routed_paths)
        total_bends = sum(p.num_bends() for p in self.routed_paths)
        num_crossings = self.optimizer.crossing_optimizer.get_crossing_count()
        reuse_rate = self.optimizer.local_optimizer.enhance_reuse(self.routed_paths)
        
        report = {
            'num_paths': len(self.routed_paths),
            'total_length': total_length,
            'avg_length': total_length / len(self.routed_paths) if self.routed_paths else 0,
            'total_bends': total_bends,
            'avg_bends': total_bends / len(self.routed_paths) if self.routed_paths else 0,
            'num_crossings': num_crossings,
            'reuse_rate': reuse_rate,
            'final_objective': self.final_objective,
        }
        
        return report
    
    def print_report(self):
        """Print optimization report"""
        report = self.get_optimization_report()
        
        if not report:
            print("No routing completed yet")
            return
        
        print("\n" + "="*70)
        print("OPTIMIZATION REPORT")
        print("="*70)
        print(f"Number of paths:      {report['num_paths']}")
        print(f"Total path length:    {report['total_length']:.2f}")
        print(f"Average path length:  {report['avg_length']:.2f}")
        print(f"Total bends:          {report['total_bends']}")
        print(f"Average bends/path:   {report['avg_bends']:.1f}")
        print(f"Number of crossings:  {report['num_crossings']}")
        print(f"Reuse rate:           {report['reuse_rate']:.2%}")
        print(f"Final objective:      {report['final_objective']:.2f}")
        print("="*70)
    
    def visualize_result(self, output_path: str = None):
        """
        Visualize routing result
        
        Args:
            output_path: Path to save visualization
        """
        try:
            import matplotlib.pyplot as plt
            
            fig, ax = plt.subplots(figsize=(12, 10))
            
            # Draw grid
            ax.set_xlim(-0.5, self.W - 0.5)
            ax.set_ylim(-0.5, self.H - 0.5)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            
            # Draw valves
            for r in range(self.H):
                for c in range(self.W):
                    val = self.grid[r, c]
                    if val > 0:
                        ax.plot(c, r, 'bs', markersize=10)
                        ax.text(c, r, str(val), ha='center', va='center',
                               color='white', fontsize=8, fontweight='bold')
            
            # Draw paths with different colors
            import matplotlib.cm as cm
            colors = cm.rainbow(np.linspace(0, 1, len(self.routed_paths)))
            
            for path, color in zip(self.routed_paths, colors):
                points = path.points
                if len(points) >= 2:
                    xs = [p[1] for p in points]  # columns
                    ys = [p[0] for p in points]  # rows
                    ax.plot(xs, ys, 'o-', color=color, linewidth=2, 
                           markersize=4, alpha=0.7, label=f'Net {path.net_id}')
            
            ax.set_title('Optimized Routing Result', fontsize=14, fontweight='bold')
            ax.set_xlabel('Column')
            ax.set_ylabel('Row')
            ax.invert_yaxis()  # Match grid orientation
            
            # Add legend if not too many paths
            if len(self.routed_paths) <= 10:
                ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
            
            plt.tight_layout()
            
            if output_path:
                plt.savefig(output_path, dpi=150, bbox_inches='tight')
                print(f"Visualization saved to {output_path}")
            else:
                plt.savefig('/tmp/routing_result.png', dpi=150, bbox_inches='tight')
                print("Visualization saved to /tmp/routing_result.png")
            
            plt.close()
            
        except ImportError:
            print("matplotlib not available for visualization")


def main():
    """
    Main function demonstrating the optimized router
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='Optimized Router with Multi-objective Optimization')
    parser.add_argument('input', help='Input benchmark file')
    parser.add_argument('--min-spacing', type=float, default=1.0,
                       help='Minimum spacing constraint (default: 1.0)')
    parser.add_argument('--weight-crossings', type=float, default=1.0,
                       help='Weight for crossing penalty (default: 1.0)')
    parser.add_argument('--weight-ports', type=float, default=1.0,
                       help='Weight for port penalty (default: 1.0)')
    parser.add_argument('--weight-reuse', type=float, default=1.0,
                       help='Weight for reuse reward (default: 1.0)')
    parser.add_argument('--weight-delay', type=float, default=1.0,
                       help='Weight for delay penalty (default: 1.0)')
    parser.add_argument('--output', help='Output visualization path')
    
    args = parser.parse_args()
    
    # Parse benchmark file
    print(f"Loading benchmark from {args.input}...")
    grid, nets = parse_benchmark(args.input)
    print(f"Grid size: {grid.shape[0]} x {grid.shape[1]}")
    print(f"Number of nets: {len(nets)}")
    
    # Setup optimization weights
    weights = {
        'crossings': args.weight_crossings,
        'ports': args.weight_ports,
        'reuse': args.weight_reuse,
        'delay': args.weight_delay,
    }
    
    # Create and run optimized router
    router = OptimizedRouter(grid, nets, args.min_spacing, weights)
    success = router.route_with_optimization()
    
    if success:
        router.print_report()
        if args.output:
            router.visualize_result(args.output)
        else:
            router.visualize_result()


def parse_benchmark(filepath):
    """Parse benchmark file into grid and nets"""
    with open(filepath, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]
    
    # Find separator (empty line or first line with different format)
    grid_lines = []
    net_lines = []
    in_nets = False
    
    for line in lines:
        if not in_nets:
            # Check if this looks like a grid line (contains -1 or positive integers separated by tabs/spaces)
            parts = line.split()
            if parts and all(p.lstrip('-').isdigit() for p in parts):
                # Check if this is a net line (fewer numbers, no -1)
                if '-1' not in parts and len(parts) < 20:  # Heuristic
                    in_nets = True
                    net_lines.append(line)
                else:
                    grid_lines.append(line)
            else:
                in_nets = True
                net_lines.append(line)
        else:
            net_lines.append(line)
    
    # Parse grid
    grid = []
    for line in grid_lines:
        row = [int(x) for x in line.split()]
        grid.append(row)
    grid = np.array(grid)
    
    # Parse nets
    nets = []
    for line in net_lines:
        if line.strip():
            net = [int(x) for x in line.split()]
            nets.append(net)
    
    return grid, nets


if __name__ == '__main__':
    main()
