import mip
from itertools import chain, combinations

class TspBaseModel(object):
    """Class to instantiate the common \"Base TSP\" model, that is, a binary assignment model,
    with functions to solve the model, print the $x$ variables, and show points and solutions."""
    def __init__(self, c, points=None, dist_fun=None, relax=False, solver="CBC"):
        self.c = c
        self.n = len(self.c)
        self.V = set(range(self.n))
        self.A = [(i, j) for i in self.V for j in self.V if(i != j)]
        self.route = []
        self.routeList = []

        self.model_sense = mip.MINIMIZE
        if(solver.lower()=="cbc"):
            self.model = mip.Model(solver_name=mip.CBC, sense=self.model_sense)
        elif(solver.lower()=="gurobi"):
            self.model = mip.Model(solver_name=mip.GUROBI, sense=self.model_sense)
        else:
            raise NameError('Invalid Solver name!')

        if(relax):
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

    def updateRoute(self):
        self.route = [(i, j) for (i, j) in self.A if self.x[i, j].x > 0.5]

    def updateRouteList(self):
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

    def solve(self, time=None, heur=None, log=False):
        if(time != None):
            self.model.max_seconds = time
        if(heur != None):
            pass
        if(log):
            self.model.verbose = 1
        else:
            self.model.verbose = 0

        self.model.optimize()

        try:
            (self.x[self.A[0]].x <= 2) == True
        except:
            return
        self.updateRoute()
        self.updateRouteList()
        
    def printX(self, limX = 0.5):
        try:
            (self.x[self.A[0]].x <= 2) == True
        except:
            return
        for msg in ( print('x[{}, {}] = {}'.format(i, j, self.x[i, j].x)) for (i, j) in self.A if self.x[i, j].x > limX ):
            msg
        
    def routeToVars(self, route):
        self.model.reset()
        for (i, j) in route:
            self.x[i, j].start = 1
        #self.model.update()
        self.route = route.copy()
        self.updateRouteList()

    def printRoute(self):
        if (self.route == []):
            print('No route available up to this moment!')
            return
        print('\nROUTE BUILT:\n')
        for item in self.routeList[:-1]:
            print('{} -> '.format(item), end='')
        print('0', end='')
        print('\n')
        return

class TSP_DFJ_Model(TspBaseModel):
    """Class to instantiate the TSP model based on the classic DFJ formulation."""
    def __init__(self, c, points=None, dist_fun=None, relax=False, solver="CBC"):
        TspBaseModel.__init__(self, c, points, dist_fun, relax, solver)

        # Setting the DFJ Sub-tour cutting constraint
        self.POWER_SET = list()
        for z in chain.from_iterable( combinations(self.V, r) for r in range(2, len(self.V)) ):
            self.POWER_SET.append(list(z))

        for S in self.POWER_SET:
            self.model += (mip.xsum(self.x[i, j] for i in S for j in S if i != j) <= len(S) - 1)

class TSP_MTZ_Model(TspBaseModel):
    """Class to instantiate the TSP model based on the classic MTZ formulation."""
    def __init__(self, c, points=None, dist_fun=None, relax=False, solver="CBC"):
        TspBaseModel.__init__(self, c, points, dist_fun, relax, solver)

        if(relax):
            self.u = {i: self.model.add_var(lb=0, ub=(self.n-1)) for i in self.V}
        else:
            #self.u = {i: self.model.add_var(var_type=mip.INTEGER, lb=0, ub=(self.n-1)) for i in self.V}
            self.u = {i: self.model.add_var(lb=0, ub=(self.n-1)) for i in self.V}
        
        for(i, j) in self.A:
            if(j!=0):
                self.model += (self.u[i] - self.u[j] + self.n * self.x[i, j] <= self.n - 1)

    def printU(self):
        try:
            (self.u[0].x <= self.n) == True
        except:
            return
        for msg in (print('u[{}] = {}'.format(j, self.u[j].x)) for j in self.V):
            msg