# Solving Pathfinding Problems

The goal is to implement and compare two classical graph search algorithms, Dijkstra's algorithm and A*. This is done on a grid-based environment inspired by video game pathfinding.

The movement costs are defined as:

- **1.0** for cardinal directions (N, S, E, W)  
- **1.5** for diagonal directions (NE, NW, SE, SW)

All experiments are conducted on the `brc000d.map` grid using 30 test instances defined in `testinstances.txt`, and the maps are stored in the `dao-map/` folder.

When you run the program, it will generate 2 plots:

- **nodes_expanded.png:** compares the number of nodes expanded by Dijkstra vs A*
- **running_time.png:** compares execution time between the two algorithms

Each point in the plot represents one search problem.

The dataset is taken from [movingai.com](https://movingai.com).
