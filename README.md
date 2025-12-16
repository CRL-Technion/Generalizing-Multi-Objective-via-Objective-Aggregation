# Generalizing Multi-Objective Search via Objective-Aggregation Functions

C++ implementation of the objective-aggregation framework for multi-objective search, as described in "Generalizing Multi-Objective Search via Objective-Aggregation Functions".

## Overview

This repository implements a generalized multi-objective search (MOS) formulation that distinguishes between **hidden objectives** (tracked during search) and **solution objectives** (defined via aggregation functions). This enables efficient handling of complex objective interactions in robotics planning problems.

**Key Features:**
- Two algorithms: **NAMOAdr** and **A*pex**
- Two modes: **Original** (all objectives independent) and **Aggregation** (hidden objectives aggregated)
- Runtime mode selection via command-line arguments (no recompilation needed)

## Building the Project

### Prerequisites
```bash
sudo apt-get install build-essential cmake g++ libboost-all-dev
```

### Build
```bash
cmake .
make
```

Executables will be in `bin/`:
- `multiobj` - Main solver
- `preprocess_graph` - Preprocessing tool for obstacle uncertainty problems

## Usage

### Basic Example (4 Obstacles, Aggregation Mode)
```bash
./bin/multiobj \
  -m resources/point_robot/exp_4_obs/file_0.gr \
     resources/point_robot/exp_4_obs/file_1.gr \
     resources/point_robot/exp_4_obs/file_2.gr \
     resources/point_robot/exp_4_obs/file_3.gr \
     resources/point_robot/exp_4_obs/file_4.gr \
  --e_time resources/point_robot/exp_4_obs/file_5.gr \
  -s 134 -g 4517 \
  -a NAMOAdr --namoa-mode=aggregation \
  -o output.txt
```

### Key Command-Line Parameters

| Parameter | Short | Description | Default |
|-----------|-------|-------------|---------|
| `--algorithm` | `-a` | `Apex` or `NAMOAdr` | `Apex` |
| `--map` | `-m` | Graph files (first n-1 are safety, last is cost) | Required |
| `--start` | `-s` | Start node ID | Required |
| `--goal` | `-g` | Goal node ID | Required |
| `--output` | `-o` | Output file name | Required |
| `--namoa-mode` | | `original` or `aggregation` | `original` |
| `--apex-mode` | | `original` or `aggregation` | `original` |
| `--eps` | | Epsilon approximation (applies to path length only) | `0` |
| `--cutoffTime` | | Time limit in seconds | `300` |
| `--e_time` | | Edge time file (optional) | - |

### Algorithm Modes

**Original Mode** - Standard multi-objective A*, treats all objectives independently:
```bash
./bin/multiobj -a NAMOAdr --namoa-mode=original [options...]
./bin/multiobj -a Apex --apex-mode=original [options...]
```

**Aggregation Mode** - Aggregates hidden objectives for dramatic speedups:
```bash
./bin/multiobj -a NAMOAdr --namoa-mode=aggregation [options...]
./bin/multiobj -a Apex --apex-mode=aggregation [options...]
```

## Input Files

### Graph Files (`.gr` format)
- **First (n-1) files**: Safety probabilities [0,1] for each obstacle (automatically converted to risk = 1 - safety)
- **Last file**: Cost/length values

### Preprocessing for Obstacle Uncertainty

The `preprocess_graph` tool converts geometric obstacle data into `.gr` files:

```bash
./bin/preprocess_graph \
  resources/point_robot/exp_12_obs/obstacle_shadow_definitions.txt \
  resources/point_robot/exp_12_obs/vertex_positions.txt \
  resources/point_robot/exp_12_obs/edges.txt \
  output/file
```

**Input files:**
1. `obstacle_shadow_definitions.txt` - Obstacle shadow regions with risk values
2. `vertex_positions.txt` - Vertex coordinates for collision checking
3. `edges.txt` - Graph edge connectivity

**Output:** `file_0.gr` through `file_N-1.gr` (per-obstacle safety), `file_N.gr` (length), and `file_edge_times.gr`

## Supported Applications

**Note:** This implementation is specifically designed for planning under obstacle uncertainty problems with risk and path length objectives. Different problem types may require code modifications.

**Planning under Obstacle Uncertainty (OU)**: Motion planning with uncertain obstacle positions. Included datasets:
- Point robot navigation (4, 8, 12 obstacles)
