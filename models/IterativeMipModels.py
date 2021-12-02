import mip
import time
from networkx import simple_cycles, DiGraph
from BasicMipModels import TspBaseModel

class Iterative_TSP_DFJ_Model(TspBaseModel):
    """Class to instantiate an iterative solution TSP model based on the classic DFJ formulation."""
    def __init__(self, c, points=None, dist_fun=None, relax=False, solver="CBC"):
        TspBaseModel.__init__(self, c, points, dist_fun, relax, solver)

        self.routes_with_weights = list()
        self.time_module = time

    def solve(self, time=None, heur=None, log=False, DFJ_log=True):
        if(time != None):
            self.model.max_seconds = time
        if(heur != None):
            pass
        if(log):
            self.model.verbose = 1
        else:
            self.model.verbose = 0

        self.totalSolutionTime = 0
        self.totalIter = 1

        while True:
            self.start_solving_time = self.time_module.time()
            self.model.optimize()
            self.totalSolutionTime += (self.time_module.time() - self.start_solving_time)
            self.routes_with_weights = [(i, j, self.c[i, j]) for (i, j) in self.A if self.x[i, j].x > 0.5]
            if(DFJ_log):
                print('Iteration {}:'.format(self.totalIter))
                print("\tPresent objective value: {}".format(self.model.objective_value))
                print("\tTotal solution time: {}s".format(self.totalSolutionTime))

            self.G = DiGraph()
            self.G.add_weighted_edges_from(self.routes_with_weights)

            self.sub_circuits = list(simple_cycles(self.G))
            
            if(len(self.sub_circuits)) == 1:
                self.updateRoute()
                self.updateRouteList()
                return
            if(time != None):
                if(self.totalSolutionTime > time):
                    try:
                        (self.x[self.A[0]].x <= 2) == True
                    except:
                        return
                    self.updateRoute()
                    self.updateRouteList()

            for cycle in self.sub_circuits:
                self.model += (mip.xsum(self.x[i, j] for i in cycle for j in cycle if (i != j)) <= len(cycle) - 1)

            self.totalIter += 1