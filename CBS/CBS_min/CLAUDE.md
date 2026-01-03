# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **minimal C++ implementation of the Conflict-Based Search (CBS) algorithm** for multi-agent path planning. This codebase is a focused, educational implementation that demonstrates the core CBS algorithm with constraint-aware A* search as the low-level planner. Unlike the parent CBS project, this is a streamlined version designed for clarity and algorithm understanding.

## Architecture

### Core Components

1. **Data Structures**:
   - `Pos`: 2D position (x, y)
   - `Grid`: Map representation with obstacles ('.' = passable, '#' = obstacle)
   - `Constraint`: Vertex and edge constraints for agents with type enumeration
   - `ConstraintTable`: Fast lookup for constraints using `unordered_set<long long>`
   - `State`: Space-time state (x, y, t) with custom hash function
   - `Conflict`: Structure for conflict detection results

2. **Constraint System**:
   - **Vertex constraints**: Prevent agents from being at specific positions at specific times
   - **Edge constraints**: Prevent agents from moving between specific positions at specific times
   - Constraints are packed into `long long` keys for efficient hash set storage using bit shifting
   - Key functions: `keyVertex(x, y, t)` and `keyEdge(x1, y1, x2, y2, t)`
   - Helper functions: `violatesVertex()` and `violatesEdge()` for constraint checking

3. **Search Algorithms**:
   - `spaceTimeAStar()`: Constraint-aware A* search with vertex/edge constraint checking (lines 96-154)
   - **Note**: This minimal version does NOT include `spaceTimeBFS()` from the parent project
   - Operates in space-time (x, y, t) domain with Manhattan distance heuristic

4. **Movement Model**:
   - 5-direction movement (up, down, left, right, wait) implemented in `getNeighbors()` function
   - Wait action allows agents to stay in place

5. **Conflict Detection**:
   - `detectFirstConflict()`: Detects both vertex conflicts (same position) and edge conflicts (swapping positions)
   - `posAt()`: Helper function to get position from path at time t, with bounds checking

6. **CBS Algorithm Core**:
   - `CBS()`: Main algorithm implementation (lines 231-306)
   - Uses priority queue (`CTNode`) for constraint tree node expansion
   - Supports both vertex and edge constraint resolution
   - `replanAgent()`: Lambda function for agent replanning with constraints
   - `sumOfCosts()`: Objective function measuring total path lengths

### Key Implementation Details

- **Coordinate System**: Uses `g[y][x]` for grid access (y = row, x = column)
- **Time Dimension**: All search algorithms include time dimension for multi-agent coordination
- **Chinese Code Comments**: Extensive Chinese comments explain algorithm logic with section headers
- **Single File Design**: Entire implementation is in `CBS_min.cpp` for simplicity
- **Efficient Data Structures**: Uses `unordered_set` and `unordered_map` with custom hash functions
- **Constraint Propagation**: Builds constraint tables per agent from global constraint list

## Development Commands

### Building
```bash
# Using VS Code task (configured in .vscode/tasks.json)
# This compiles the active file with debug symbols
g++ -fdiagnostics-color=always -g CBS_min.cpp -o CBS_min.exe

# Manual compilation with optimizations
g++ -std=c++11 -O2 CBS_min.cpp -o CBS_min.exe
```

### Running
```bash
./CBS_min.exe
```

### Testing
- No formal test framework
- Testing is done via the `main()` function with sample grid and agent configurations
- The current test case uses a 10x5 grid with two agents crossing paths
- Modify the sample grid, start/goal positions, or add agents in `main()` to test different scenarios
- Output shows each agent's path with timestep coordinates

## Development Workflow

1. **Code Structure**: All code is in `CBS_min.cpp` - modify this file for changes
2. **Build**: Use the VS Code build task (Ctrl+Shift+B) or manual g++ compilation
3. **Test**: Run the executable and observe path output for the two-agent crossing scenario
4. **Debug**: The VS Code configuration includes debug symbols (`-g` flag) for debugging

## Important Notes

- **Chinese Language**: Code comments and section headers are in Chinese, but variable names are mixed English/Chinese
- **Educational Focus**: This is a minimal implementation for algorithm understanding and experimentation
- **Constraint System**: Uses efficient bit packing (`long long` keys) for performance in constraint checking
- **Space-Time Search**: All algorithms search in (x, y, t) space, essential for multi-agent coordination
- **Max Time Limit**: The `maxT` parameter in `spaceTimeAStar()` prevents infinite waiting (currently hardcoded to 80)
- **Complete CBS Implementation**: Includes both vertex and edge constraints, conflict detection, and constraint tree search

## Common Modifications

1. **Change Grid Map**: Modify the `grid.g` initialization in `main()` (lines 312-318)
2. **Add/Remove Agents**: Modify `starts` and `goals` vectors in `main()` (lines 323-324)
3. **Adjust Search Parameters**: Modify `maxT` value in `replanAgent()` lambda (line 241)
4. **Add Custom Constraints**: Manually add constraints to test specific scenarios
5. **Change Movement Model**: Modify `getNeighbors()` function for different action sets
6. **Experiment with Heuristics**: Modify Manhattan distance in `spaceTimeAStar()` for different search behavior

## File Structure
```
CBS_min/
├── CBS_min.cpp              # Main implementation (minimal CBS)
├── .vscode/tasks.json       # VS Code build configuration for MinGW64
├── .claude/settings.local.json # Claude permissions
└── CBS_min.exe             # Compiled executable (generated)
```

## Differences from Parent CBS Project

1. **File Name**: Uses `CBS_min.cpp` instead of `CBS.cpp`
2. **Algorithm Focus**: Complete CBS implementation with constraint-aware A* only (no BFS variant)
3. **Code Organization**: Clear Chinese section headers mark different components
4. **Testing Scenario**: Two-agent crossing paths instead of single-agent with constraints
5. **Implementation Style**: More streamlined with lambda functions and modern C++ patterns

This implementation demonstrates the complete CBS algorithm workflow: constraint tree search, conflict detection, constraint propagation, and constraint-aware path planning.