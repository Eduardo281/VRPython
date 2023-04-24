from BaseTspMipModels import MatrixTspIterativeMipBaseModel

class Matrix_Iterative_TSP_DFJ_Model(MatrixTspIterativeMipBaseModel):
    """Class to instantiate an iterative solution TSP model based on the classic 
    Dantzig-Fulkerson-Johnson, 1954 (DFJ) formulation."""
    def __init__(self, c, relax_X_vars:bool=False, solver:str="CBC"):
        MatrixTspIterativeMipBaseModel.__init__(self, c, relax_X_vars, solver)
