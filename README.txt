#Navigation System

Now this project just finish some basic functions.
At least i finish my project framework and some basic data structures.
Such as Node Edge RoadNetwork.
Some spatial indexes to load a virtual grid.
The most important is i finish a map loader to deal some Real data(OSM) and break a wall between MY TOY CODE and a little tool in my life. 


==================================================================================================================================================

Now I had finished most of part of core alorithm
I use Dijkstra and A* 
Two alorithms to find the shortest road between two real point.
I get some datas such as:

Running main() from ./googletest/src/gtest_main.cc
[==========] Running 2 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 2 tests from Benchmark
[ RUN      ] Benchmark.RealMapRouting

=== Map Load Statistics ===
Nodes: 13912
Edges: 25675
Load time: 164.151ms

=== Routing Performance ===
Successful queries: 77/100
Dijkstra avg time: 1.17467ms
A* avg time: 0.858487ms
Speedup: 1.3683x
Dijkstra avg explored: 6820
A* avg explored: 4864
Node reduction: 28.6707%
[       OK ] Benchmark.RealMapRouting (422 ms)
[ RUN      ] Benchmark.OptimizedDijkstra
Testing: 12674461525 -> 2623878073
Original: 0.588641ms, Optimized: 0.554559ms, Speedup: 1.06146x
[       OK ] Benchmark.OptimizedDijkstra (134 ms)
[----------] 2 tests from Benchmark (556 ms total)

[----------] Global test environment tear-down
[==========] 2 tests from 1 test suite ran. (556 ms total)
[  PASSED  ] 2 tests.



And maybe I can see some different between them.
Some more things I do inclue I find an optimization about containers.
Different containers will get different performances
