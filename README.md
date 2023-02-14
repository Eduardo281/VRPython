# VRPython

## A Python tool designed to solve Vehicle Routing and Traveling Salesman Problems

The main purpose of VRPython is to be an easy-to-use solving tool for different versions of Vehicle Routing Problem (VRP), including the one-vehicle case, which can be identified as the Traveling Salesperson Problem (TSP). To reach this goal, we are developing a multi-classes framework, where the user can import just the solver needed to face the problem, and setting just some hyperparameters (if wanted). It can be used to research purposes too, being an easy tool to run from classic models to modern heuristics, but different from other similar packages, the objective of VRPython is to be used by non Operations Research specialists too, and so some MIP/CP modelling tools may be not very immediate.

At this moment we are developing it totally in Python, focusing in keeping it scalable and user-friendly. We consider the possible need to add some C/C++ extensions, depending on future performance needs.

**Completed Steps:**

* MIP models for the classic TSP;

* Iterative solver using MIP model for the classic TSP;

* CP solver for the classic TSP using OR-Tools.

**Next Steps:**

* MIP models for the classic VRP;

* Iterative solver using MIP model for the classic VRP;

* CP solver for the classic VRP using OR-Tools;

* MIP models for some common TSP variants;

* CP solvers for some common TSP variants;

* MIP models for some common VRP variants;

* CP solvers for some common VRP variants;

* Usage example file.

**Future Steps:**

* Add heuristics to both TSP and VRP, like Local Search, Variable Neighborhood Search, Ant Colony Optimization, Genetic Algorithms, among others.
