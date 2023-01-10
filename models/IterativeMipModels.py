import mip
import time
from networkx import simple_cycles, DiGraph
from BasicMipModels import MatrixTspBaseModel

class Matrix_Iterative_TSP_DFJ_Model(MatrixTspBaseModel):
    """Class to instantiate an iterative solution TSP model based on the classic DFJ formulation."""
    def __init__(self, c, relax_X_vars=False, solver="CBC"):
        MatrixTspBaseModel.__init__(self, c, relax_X_vars, solver)

        self.routesWithWeights = list()
        self.G = DiGraph()

    def solve(self, totalRuntime=None, heur=None, printSolverLog=False, printIterationsLog=True):
        if(totalRuntime != None):
            self.model.max_seconds = totalRuntime
        if(heur != None):
            pass
        if(printSolverLog):
            self.model.verbose = 1
        else:
            self.model.verbose = 0

        self.totalSolutionTime = 0
        self.totalIter = 1

        while True:
            self.start_solving_time = time.time()
            self.model.optimize()
            self.totalSolutionTime += (time.time() - self.start_solving_time)
            self.routesWithWeights = [(i, j, self.c[i, j]) for (i, j) in self.A if self.x[i, j].x > 0.5]
            if(printIterationsLog):
                print('Iteration {}:'.format(self.totalIter))
                print("\tPresent objective value: {}".format(self.model.objective_value))
                print("\tTotal solution time: {}s".format(self.totalSolutionTime))

            self.G.clear()
            self.G.add_weighted_edges_from(self.routesWithWeights)

            self.sub_circuits = list(simple_cycles(self.G))
            
            if(len(self.sub_circuits)) == 1:
                self.updateSolution()
                return
            if(totalRuntime != None):
                if(self.totalSolutionTime > totalRuntime):
                    try:
                        (self.x[self.A[0]].x <= 2) == True
                    except:
                        return
                    self.updateSolution()

            for cycle in self.sub_circuits:
                self.model += (mip.xsum(self.x[i, j] for i in cycle for j in cycle if (i != j)) <= len(cycle) - 1)

            self.totalIter += 1
