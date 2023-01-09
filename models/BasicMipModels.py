import mip
import numpy as np
from itertools import chain, combinations

class MatrixTspBaseModel(object):
    """Class to instantiate the common \"Base TSP\" model, that is, a binary assignment model,
    with functions to solve the model, print the $x$ variables, and show points and solutions."""
    def __init__(self, c:np.ndarray, relax_X_vars:bool=False, solver:str="CBC", labels:np.ndarray=None):
        if(not isinstance(c, np.ndarray)):
            try:
                self.c = np.array(c)
            except:
                raise TypeError("[ERROR] Could not convert data entered into a numpy.ndarray!")
        else:
            self.c = c
        if(self.c.shape[0] != self.c.shape[1]):
            raise RuntimeError("[ERROR] Data must be a matrix having the same number of lines and columns!")
        self.n = len(self.c)
        self.V = set(range(self.n))
        self.A = [(i, j) for i in self.V for j in self.V if(i != j)]
        self.route = []
        self.routeList = []
        self.objectiveValue = None

        self.modelSense = mip.MINIMIZE
        if(solver.lower()=="cbc"):
            self.model = mip.Model(solver_name=mip.CBC, sense=self.modelSense)
        elif(solver.lower()=="gurobi"):
            self.model = mip.Model(solver_name=mip.GUROBI, sense=self.modelSense)
        else:
            raise NameError('Invalid Solver name!')

        if(relax_X_vars):
            self.x = {(i, j): self.model.add_var(lb=0, ub=1) for (i, j) in self.A}
        else:
            self.x = {(i, j): self.model.add_var(var_type=mip.BINARY) for (i, j) in self.A}

        # Objective Function
        self.model.objective = mip.xsum(self.c[i, j] * self.x[i, j] for (i, j) in self.A)

        # All nodes must be visited exactly one time
        for i in self.V:
            self.model += mip.xsum(self.x[i, j] for j in self.V if (i, j) in self.A) == 1

        # All nodes must be left exactly one time
        for j in self.V:
            self.model += mip.xsum(self.x[i, j] for i in self.V if (i, j) in self.A) == 1

    def updateRoute(self) -> None:
        self.route = [(i, j) for (i, j) in self.A if self.x[i, j].x > 0.5]

    def updateRouteList(self) -> None:
        self.routeList = [0]
        self.v0 = 0
        self.v1 = -1
        while(self.v1 != 0):
            for item in self.route:
                if item[0] == self.v0:
                    self.v1 = item[1]
                    self.routeList.append(self.v1)
                    break
            self.v0 = self.v1

    def updateRouteAndRouteList(self):
        self.updateRoute()
        self.updateRouteList()
    
    def updateSolution(self):
        self.updateRouteAndRouteList()
        self.objectiveValue = self.model.objective_value

    def solve(self, totalRuntime=None, heur=None, printSolverLog:bool=False) -> None:
        if(totalRuntime != None):
            self.model.max_seconds = totalRuntime
        if(heur != None):
            pass
        if(printSolverLog):
            self.model.verbose = 1
        else:
            self.model.verbose = 0

        self.model.optimize()

        try:
            (self.x[self.A[0]].x <= 2) == True
        except:
            return
        self.updateSolution()
        
    def printX(self, limX:float=0.5) -> None:
        try:
            (self.x[self.A[0]].x <= 2) == True
        except:
            return
        for msg in ( print('x[{}, {}] = {}'.format(i, j, self.x[i, j].x)) for (i, j) in self.A if self.x[i, j].x > limX ):
            msg
        
    def passRouteToVars(self, route):
        self.model.reset()
        for (i, j) in route:
            self.x[i, j].start = 1
        #self.model.update()
        self.route = route.copy()
        self.updateRouteList()

    def printRoute(self) -> None:
        if (self.route == []):
            print('No route available up to this moment!')
            return
        print('\nROUTE BUILT:\n')
        for item in self.routeList[:-1]:
            print('{} -> '.format(item), end='')
        print('0', end='')
        print('\n')
        return

class Matrix_TSP_DFJ_Model(MatrixTspBaseModel):
    """Class to instantiate the TSP model based on the classic DFJ formulation."""
    def __init__(self, c, relax_X_vars=False, solver="CBC"):
        MatrixTspBaseModel.__init__(self, c, relax_X_vars, solver)

        # Setting the DFJ Sub-tour elimination constraints:
        self.POWER_SET = list(chain.from_iterable( combinations(self.V, r) for r in range(2, len(self.V)) ))

        # for S in self.POWER_SET:
        #     self.model += (mip.xsum(self.x[i, j] for i in S for j in S if i != j) <= len(S) - 1)

        for cst in (mip.xsum(self.x[i, j] for i in S for j in S if i != j) <= len(S) - 1 for S in self.POWER_SET):
            self.model += cst

class Matrix_TSP_MTZ_Model(MatrixTspBaseModel):
    """Class to instantiate the TSP model based on the classic MTZ formulation."""
    def __init__(self, c, relax_X_vars=False, relax_U_vars=True, solver="CBC", modelType="classic"):
        MatrixTspBaseModel.__init__(self, c, relax_X_vars, solver)
        self.modelType = modelType

        if(relax_U_vars):
            self.u = {i: self.model.add_var(lb=0, ub=(self.n-1)) for i in self.V}
        else:
            self.u = {i: self.model.add_var(var_type=mip.INTEGER, lb=0, ub=(self.n-1)) for i in self.V}
        
        if(self.modelType.lower() in ('classic', 'mtz')):
            for(i, j) in self.A:
                if(j!=0):
                    self.model += (self.u[i] - self.u[j] + self.n * self.x[i, j] <= self.n - 1)
        elif(self.modelType.lower() in ('lift', 'lifted', 'dl')):
            for(i, j) in self.A:
                if(j!=0):
                    self.model += (self.u[i] - self.u[j] + self.n * self.x[i, j] + (self.n-2) * self.x[j, i] <= self.n - 1)
            for i in self.V:
                if(0, i) in self.A:
                    self.model += (self.u[i] >= (self.n - 2) * self.x[i, 0] + 
                        mip.xsum(self.x[j, i] for j in self.V if (i, j) in self.A if j != 0))

                    self.model += (self.u[i] <= self.n - (self.n - 2) * self.x[0, i] -  
                        mip.xsum(self.x[i, j] for j in self.V if (i, j) in self.A if j != 0))
        # elif(self.model_type.lower() in ('rlt', 'sd')):
        #     if(relax):
        #         self.u = {i: self.model.add_var(lb=0, ub=(self.n-1)) for i in self.V}
        #     else:
        #         #self.u = {i: self.model.add_var(var_type=mip.INTEGER, lb=0, ub=(self.n-1)) for i in self.V}
        #         self.u = {i: self.model.add_var(lb=0, ub=(self.n-1)) for i in self.V}
        #     for(i, j) in self.A:
        #         if(j!=0):
        #             self.model += (self.u[i] - self.u[j] + self.n * self.x[i, j] <= self.n - 1)
        else:
            raise NameError('Undefined model type!')

    def printU(self):
        try:
            (self.u[0].x <= self.n) == True
        except:
            return
        for msg in (print('u[{}] = {}'.format(j, self.u[j].x)) for j in self.V):
            msg
