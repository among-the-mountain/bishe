# Routing Optimization Framework

A comprehensive routing optimization system for microfluidic chip routing with multi-objective optimization.

## Overview

This framework implements a complete 6-phase optimization pipeline for chip routing:

1. **Basic Path Generation** - Initial path generation using any-angle search (A*/Theta*/DRL)
2. **Geometric Constraint Embedding** - Minimum spacing and boundary constraints
3. **Crossing Constraint Optimization** - Detection and valve assignment for crossings
4. **Timing Constraint Optimization** - Mutual exclusion and execution scheduling
5. **Local Optimization Strategies** - Path smoothing, crossing resolution, reuse enhancement
6. **Multi-objective Optimization** - Unified objective function balancing all goals

## Architecture

### Core Modules

- **`routing_optimizer.py`** - Core optimization framework with all components:
  - `PathSegment` - Represents a routing path segment
  - `RoutingPath` - Complete routing path for a net
  - `GeometricConstraintChecker` - Handles spacing and boundary constraints
  - `CrossingOptimizer` - Detects and optimizes path crossings
  - `TimingConstraintOptimizer` - Manages timing constraints and scheduling
  - `LocalOptimizer` - Local optimization strategies
  - `MultiObjectiveOptimizer` - Multi-objective function calculation
  - `RoutingOptimizationFramework` - Main integration framework

- **`optimized_router.py`** - Integration with existing router:
  - `OptimizedRouter` - Combines pathfinding with optimization
  - Benchmark parsing and visualization
  - Command-line interface

- **`demo_optimization.py`** - Comprehensive demonstration script:
  - Phase-by-phase demonstration
  - Weight configuration comparison
  - Detailed explanations

## Multi-Objective Function

The framework optimizes the following unified objective:

```
F = Σ L(Pᵢ) + α·Cross(P) + β·Ports(P) - γ·Reuse(P) + δ·T_total
```

Where:
- **L(Pᵢ)** - Total path length (minimize)
- **Cross(P)** - Number of crossings (minimize)
- **Ports(P)** - Number of ports/valves (minimize)
- **Reuse(P)** - Path reuse rate (maximize - note the negative sign)
- **T_total** - Total timing delay (minimize)
- **α, β, γ, δ** - Adjustable weights for different priorities

## Installation

### Requirements

```bash
pip install numpy matplotlib
```

### Files

Place these files in the `bishe/code/` directory:
- `routing_optimizer.py`
- `optimized_router.py`
- `demo_optimization.py`

## Usage

### 1. Basic Usage - Run Optimized Router

```bash
cd bishe/code
python3 optimized_router.py ../benchmarks/R0.txt
```

This will:
- Load the benchmark
- Generate initial paths
- Apply all optimizations
- Print optimization report
- Save visualization to `/tmp/routing_result.png`

### 2. Custom Weight Configuration

```bash
python3 optimized_router.py ../benchmarks/R0.txt \
    --weight-crossings 2.0 \
    --weight-reuse 1.5 \
    --output result.png
```

Available weight options:
- `--weight-crossings` - Penalty for crossings (default: 1.0)
- `--weight-ports` - Penalty for number of ports (default: 1.0)
- `--weight-reuse` - Reward for path reuse (default: 1.0)
- `--weight-delay` - Penalty for timing delay (default: 1.0)
- `--min-spacing` - Minimum spacing constraint (default: 1.0)

### 3. Detailed Phase-by-Phase Demo

```bash
python3 demo_optimization.py ../benchmarks/R0.txt --demo
```

This shows:
- Each optimization phase in detail
- Metrics before and after each phase
- Algorithm reasoning and flow
- Breakdown of objective function

### 4. Compare Weight Configurations

```bash
python3 demo_optimization.py ../benchmarks/R0.txt --compare
```

This compares:
- Default balanced configuration
- Minimize crossings priority
- Maximize reuse priority
- Minimize ports priority
- Quality-first configuration

### 5. Full Demonstration

```bash
python3 demo_optimization.py ../benchmarks/R0.txt --demo --compare
```

## Optimization Framework Details

### Phase 1: Basic Path Generation

Uses any-angle search algorithms to generate initial paths:
- **A\* (A-star)** - Heuristic search with optimal path guarantee
- **Theta\*** - Any-angle variant allowing non-grid-aligned paths
- **DRL** - Deep Reinforcement Learning based approach

Initial paths guarantee connectivity but may violate constraints.

### Phase 2: Geometric Constraint Embedding

Enforces geometric constraints:

**Minimum Spacing (d_min)**
- Ensures paths maintain minimum distance
- Prevents fabrication issues
- Configurable via `--min-spacing`

**Boundary Legality**
- All paths must stay within chip boundaries
- Validates all path points

**Local Re-search**
- When constraints violated, triggers local path adjustment
- Attempts to reroute problematic segments

### Phase 3: Crossing Constraint Optimization

Manages path intersections:

**Crossing Detection**
- Uses line segment intersection algorithm
- Detects all crossing points between paths

**Valve Assignment**
- Unavoidable crossings get control valves
- Each crossing assigned unique valve ID
- Enables controlled flow switching

**Crossing Reduction**
- Identifies paths with excessive crossings
- Suggests local rerouting alternatives

### Phase 4: Timing Constraint Optimization

Ensures proper timing:

**Shared Segment Detection**
- Identifies path segments used by multiple nets
- Tracks segment ownership

**Mutual Exclusion**
- Shared segments cannot execute simultaneously
- Prevents flow conflicts

**Execution Scheduling**
- Creates valid execution order
- Resolves timing conflicts
- Optimizes for minimal total delay

### Phase 5: Local Optimization Strategies

Refines paths locally:

**Path Smoothing**
- Removes unnecessary bends
- Replaces zigzags with straight lines
- Uses line-of-sight optimization

**Crossing Resolution**
- Attempts to reduce crossings via local adjustments
- Offsets paths when possible
- Minimizes valve count

**Reuse Enhancement**
- Identifies opportunities to share path segments
- Increases resource utilization
- Reduces total routing area

**Connection Point Correction**
- Adjusts abstract endpoints to real boundaries
- Ensures proper valve connections

### Phase 6: Multi-Objective Optimization

Balances competing objectives:

**Objective Components**
1. Path length - Shorter paths preferred
2. Crossings - Fewer crossings reduce complexity
3. Ports - Fewer valves reduce cost
4. Reuse - Higher reuse improves efficiency
5. Delay - Lower delay improves throughput

**Weight Tuning**
- Adjust weights based on design priorities
- Example: High α emphasizes crossing reduction
- Example: High γ emphasizes reuse

**Trade-off Analysis**
- No single optimal solution
- Framework finds Pareto-optimal solutions
- User selects based on priorities

## Example Output

```
============================================================
Optimization Complete
============================================================
Final Objective Value: 2079.20
  Total path length: 742.18
  Number of crossings: 463
  Number of paths: 42

======================================================================
OPTIMIZATION REPORT
======================================================================
Number of paths:      42
Total path length:    742.18
Average path length:  17.67
Total bends:          0
Average bends/path:   0.0
Number of crossings:  463
Reuse rate:           19.21%
Final objective:      2079.20
======================================================================
```

## Algorithm Reasoning

The optimization follows this logical flow:

```
1. Generate Initial Paths (A*/Theta*)
   ↓
2. Check & Embed Geometric Constraints
   ↓ (violations found?)
3. Detect & Optimize Crossings
   ↓ (assign valves)
4. Optimize Timing & Schedule
   ↓ (ensure mutual exclusion)
5. Local Refinement
   ↓ (smooth, resolve, enhance)
6. Calculate Multi-Objective Score
   ↓
   Final Optimized Routing
```

## Configuration Scenarios

### Scenario 1: Minimize Crossings
```bash
python3 optimized_router.py benchmark.txt \
    --weight-crossings 3.0 \
    --weight-ports 1.0 \
    --weight-reuse 1.0
```
**Use when:** Design complexity is main concern, want fewer valves at crossings

### Scenario 2: Maximize Reuse
```bash
python3 optimized_router.py benchmark.txt \
    --weight-crossings 1.0 \
    --weight-ports 1.0 \
    --weight-reuse 3.0
```
**Use when:** Area efficiency is priority, willing to accept more crossings

### Scenario 3: Quality First
```bash
python3 optimized_router.py benchmark.txt \
    --weight-crossings 2.0 \
    --weight-ports 2.0 \
    --weight-reuse 2.0 \
    --weight-delay 0.5
```
**Use when:** Overall quality more important than speed

### Scenario 4: Fast Routing
```bash
python3 optimized_router.py benchmark.txt \
    --weight-crossings 0.5 \
    --weight-ports 0.5 \
    --weight-reuse 0.5 \
    --weight-delay 3.0
```
**Use when:** Timing performance is critical

## Benchmark Format

Input files should follow this format:

```
# Grid (valve layout)
-1  -1  -1   4  -1  ...
-1  -1  -1  -1  -1  ...
 1  -1   2  -1   3  ...
...

# Nets (valve sequences)
1 8 20 35 53
2 9 22 38 57
...
```

Where:
- `-1` = empty cell
- Positive integers = valve IDs
- Each net line lists valve IDs to connect

## Performance Metrics

The framework tracks:
- **Path length** - Total wire length
- **Bend count** - Number of direction changes
- **Crossing count** - Number of path intersections
- **Reuse rate** - Percentage of shared segments
- **Objective value** - Overall quality score

## Advanced Features

### Custom Constraint Functions

Extend `GeometricConstraintChecker` for custom constraints:

```python
class CustomConstraintChecker(GeometricConstraintChecker):
    def check_custom_constraint(self, path):
        # Your custom logic
        pass
```

### Custom Optimization Strategies

Add to `LocalOptimizer`:

```python
class ExtendedLocalOptimizer(LocalOptimizer):
    def my_optimization(self, paths):
        # Your optimization strategy
        pass
```

### Integration with Existing Router

The framework can integrate with the existing `SimpleGeometricRouter`:

```python
from optimized_router import OptimizedRouter

router = OptimizedRouter(grid, nets)
router.route_with_optimization(use_existing_router=True)
```

## Troubleshooting

**Issue: Too many spacing violations**
- Increase `--min-spacing` value
- Or accept violations and focus on other objectives

**Issue: Too many crossings**
- Increase `--weight-crossings`
- Use demo to see crossing distribution

**Issue: Low reuse rate**
- Increase `--weight-reuse`
- Check if benchmark allows more sharing

**Issue: Import errors**
- Ensure numpy and matplotlib are installed
- Check Python version (3.7+)

## References

This implementation is based on the problem statement requirements for comprehensive routing optimization including:
- Any-angle pathfinding (A*/Theta*/DRL)
- Geometric and crossing constraints
- Timing optimization
- Local refinement strategies
- Multi-objective optimization framework

## License

Academic/Research use. Part of the bishe (graduation project) repository.
