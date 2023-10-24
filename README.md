# Potential Field Planning in C++

## Overview


## Quick start
1. `mkdir build && cd build`
2. `cmake ..`
3. `make`

You should see the below

```
Map BEFORE: 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
-------
Map With Obstacle: 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 100.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 100.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 100.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 100.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 100.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 
-------
Map With Distance to Goal: 
12.73 12.04 11.40 10.82 10.30 9.85 9.49 9.22 9.06 9.00 
12.04 111.31 10.63 10.00 9.43 8.94 8.54 8.25 8.06 8.00 
11.40 10.63 109.90 9.22 8.60 8.06 7.62 7.28 7.07 7.00 
10.82 10.00 9.22 108.49 7.81 7.21 6.71 6.32 6.08 6.00 
10.30 9.43 8.60 7.81 107.07 6.40 5.83 5.39 5.10 5.00 
9.85 8.94 8.06 7.21 6.40 105.66 5.00 4.47 4.12 4.00 
9.49 8.54 7.62 6.71 5.83 5.00 4.24 3.61 3.16 3.00 
9.22 8.25 7.28 6.32 5.39 4.47 3.61 2.83 2.24 2.00 
9.06 8.06 7.07 6.08 5.10 4.12 3.16 2.24 1.41 1.00 
9.00 8.00 7.00 6.00 5.00 4.00 3.00 2.00 1.00 0.00 
-------
Path found!
Path: (0, 0) (1, 0) (2, 1) (3, 2) (4, 3) (5, 4) (6, 5) (7, 6) (8, 7) (9, 8) (9, 9) 
```