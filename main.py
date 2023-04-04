import random
random.seed(42)

from models.TSP_MIP_Models import *
from models.TSP_CP_Models import *

def main():
    points_int_15 = [
        [(random.randint(0, 200)-100), (random.randint(0, 200)-100)]
        for _ in range(15)
    ]

    distanceMatrix_float_10 = [
        [random.normalvariate(0, 1) * 100 for _ in range(10)]
        for _ in range(10)
    ]

    for i in range(len(distanceMatrix_float_10)):
        for j in range(len(distanceMatrix_float_10)):
            if(i < j):
                distanceMatrix_float_10[i][j] = distanceMatrix_float_10[j][i]
            elif(i == j):
                distanceMatrix_float_10[i][j] = 0

    distanceMatrix_int_20 = [
        [random.randint(0, 100) for _ in range(20)]
        for _ in range(20)
    ]

    for i in range(len(distanceMatrix_int_20)):
        for j in range(len(distanceMatrix_int_20)):
            if(i < j):
                distanceMatrix_int_20[i][j] = distanceMatrix_int_20[j][i]
            elif(i == j):
                distanceMatrix_int_20[i][j] = 0

    model_MIP_MTZ = Matrix_TSP_MTZ_Model(
        c=distanceMatrix_float_10,
        relax_X_vars=False,
        relax_U_vars=True,
        solver="CBC"
    )
    model_MIP_DL = Matrix_TSP_DL_Model(
        c=distanceMatrix_float_10,
        relax_X_vars=False,
        relax_U_vars=True,
        solver="CBC"
    )
    model_CP_Matrix = Matrix_TSP_CP_Model(
        c=distanceMatrix_int_20
    )
    model_CP_Points = Points_TSP_CP_Model (
        points = points_int_15
    )

    print("Solving MTZ MIP model...")
    model_MIP_MTZ.solve()
    print("Solve completed!")
    print("Solution found:")
    model_MIP_MTZ.printRoute()
    print()

    print("Solving MTZ DL model...")
    model_MIP_DL.solve()
    print("Solve completed!")
    print("Solution found:")
    model_MIP_DL.printRoute()
    print()

    print("Solving CP Matrix model (time limit: 10s)...")
    model_CP_Matrix.solve(totalRuntime=10)
    print("Solve completed!")
    print()

    print("Solving CP Points model (time limit: 10s)...")
    model_CP_Points.solve(totalRuntime=10)
    print("Solve completed!")
    print()

main()
