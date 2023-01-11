import mip
from itertools import chain, combinations
from BaseTspMipModels import MatrixTspBaseModel, MatrixTspMtzBaseModel

class Matrix_TSP_DFJ_Model(MatrixTspBaseModel):
    """Class to instantiate the TSP model based on the classic DFJ formulation."""
    def __init__(self, c, relax_X_vars=False, solver="CBC"):
        MatrixTspBaseModel.__init__(self, c, relax_X_vars, solver)

        # Setting the DFJ Sub-tour elimination constraints:
        self.POWER_SET = list(chain.from_iterable( combinations(self.V, r) for r in range(2, len(self.V)) ))

        for cst in (mip.xsum(self.x[i, j] for i in S for j in S if i != j) <= len(S) - 1 for S in self.POWER_SET):
            self.model += cst

class Matrix_TSP_MTZ_Model(MatrixTspMtzBaseModel):
    """Class to instantiate the TSP model based on the classic MTZ formulation."""
    def __init__(self, c, relax_X_vars=False, relax_U_vars=True, solver="CBC"):
        MatrixTspMtzBaseModel.__init__(self, c, relax_X_vars, relax_U_vars, solver)
        
        for(i, j) in self.A:
            if(j!=0):
                self.model += (self.u[i] - self.u[j] + self.n * self.x[i, j] <= self.n - 1)

class Matrix_TSP_DL_Model(MatrixTspMtzBaseModel):
    """Class to instantiate the TSP model based on the lifted MTZ formulation."""
    def __init__(self, c, relax_X_vars=False, relax_U_vars=True, solver="CBC"):
        MatrixTspMtzBaseModel.__init__(self, c, relax_X_vars, relax_U_vars, solver)

        for(i, j) in self.A:
            if(j!=0):
                self.model += (self.u[i] - self.u[j] + self.n * self.x[i, j] + (self.n-2) * self.x[j, i] <= self.n - 1)
        for i in self.V:
            if(0, i) in self.A:
                self.model += (self.u[i] >= (self.n - 2) * self.x[i, 0] + 
                    mip.xsum(self.x[j, i] for j in self.V if (i, j) in self.A if j != 0))

                self.model += (self.u[i] <= self.n - (self.n - 2) * self.x[0, i] -  
                    mip.xsum(self.x[i, j] for j in self.V if (i, j) in self.A if j != 0))

class Matrix_TSP_SD_Model(MatrixTspMtzBaseModel):
    """Class to instantiate the TSP model based on the lifted MTZ formulation."""
    def __init__(self, c, relax_X_vars=False, relax_U_vars=True, solver="CBC"):
        MatrixTspMtzBaseModel.__init__(self, c, relax_X_vars, relax_U_vars, solver)

        ###
        # TODO
        ###