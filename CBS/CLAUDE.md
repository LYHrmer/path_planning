# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a C++ implementation of the **Conflict-Based Search (CBS)** algorithm for multi-agent path planning. The codebase is a research/educational implementation focused on space-time search with constraint propagation for conflict resolution.

## Architecture

### Core Components

1. **Data Structures**:
   - `Pos`: 2D position (x, y)
   - `Grid`: Map representation with obstacles ('.' = passable, '#' = obstacle)
   - `Constraint`: Vertex and edge constraints for agents
   - `ConstraintTable`: Fast lookup for constraints using hash sets
   - `State`: Space-time state (x, y, t)

2. **Constraint System**:
   - **Vertex constraints**: Prevent agents from being at specific positions at specific times
   - **Edge constraints**: Prevent agents from moving between specific positions at specific times
   - Constraints are packed into `long long` keys for efficient hash set storage
   - Key functions: `keyVertex(x, y, t)` and `keyEdge(x1, y1, x2, y2, t)`

3. **Search Algorithms**:
   - `spaceTimeBFS()`: Basic space-time BFS without constraints
   - `spaceTimeAStar()`: Constraint-aware A* search with vertex/edge constraint checking
   - Both algorithms operate in space-time (x, y, t) domain

4. **Movement Model**:
   - 4-direction movement (up, down, left, right) + wait action
   - Implemented in `getNeighbors()` function

### Key Implementation Details

- **Coordinate System**: Uses `g[y][x]` for grid access (y = row, x = column)
- **Time Dimension**: All search algorithms include time dimension for multi-agent coordination
- **Chinese Code Comments**: Extensive Chinese comments explain algorithm logic and implementation details
- **Single File Design**: Entire implementation is in `CBS.cpp` for simplicity

## Development Commands

### Building
```bash
# Using VS Code task (configured in .vscode/tasks.json)
gcc -fdiagnostics-color=always -g CBS.cpp -o CBS.exe

# Manual compilation
g++ -std=c++11 -O2 CBS.cpp -o CBS.exe
```

### Running
```bash
./CBS.exe
```

### Testing
- No formal test framework
- Testing is done via the `main()` function with sample grid and constraints
- Modify the sample grid and constraints in `main()` to test different scenarios
- The current test case uses a 10x5 grid with a vertex constraint at (5,1) at time t=6

## Development Workflow

1. **Code Structure**: All code is in `CBS.cpp` - modify this file for changes
2. **Build**: Use the VS Code build task or manual g++ compilation
3. **Test**: Run the executable and observe path output
4. **Debug**: The VS Code configuration includes debug symbols (`-g` flag)

## Important Notes

- **Chinese Language**: Code comments and variable names are in Chinese
- **Research Focus**: This is a prototype implementation for algorithm exploration
- **Constraint System**: The constraint table uses efficient bit packing for performance
- **Space-Time Search**: All algorithms search in (x, y, t) space, not just (x, y)
- **Max Time Limit**: Algorithms include `maxT` parameter to prevent infinite waiting

## Common Modifications

1. **Change Grid Map**: Modify the `grid.g` initialization in `main()`
2. **Add Constraints**: Use `ct.forbV.insert(keyVertex(x, y, t))` for vertex constraints or `ct.forbE.insert(keyEdge(x1, y1, x2, y2, t))` for edge constraints
3. **Adjust Search Parameters**: Modify `maxT` value in `main()`
4. **Add Multiple Agents**: Currently single-agent; would require extending constraint system and search algorithms

## File Structure
```
CBS/
├── CBS.cpp                    # Main implementation
├── .vscode/tasks.json        # VS Code build configuration
└── .claude/settings.local.json # Claude permissions
```

The implementation demonstrates core CBS concepts with constraint-aware A* search as the low-level planner.