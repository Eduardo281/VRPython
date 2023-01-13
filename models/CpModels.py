import math
from BaseTspCpModels import CommonTspCpBaseModel

class Matrix_TSP_CP_Model(CommonTspCpBaseModel):
    def __init__(self, c):
        CommonTspCpBaseModel.__init__(self, len(c[0]))
        self.c = c

        self.transitCallbackIndex = self.routing.RegisterTransitCallback(self.distanceCallback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(self.transitCallbackIndex)

    def distanceCallback(self, from_index, to_index):
        return self.c[self.manager.IndexToNode(from_index)][self.manager.IndexToNode(to_index)]

class Points_TSP_CP_Model(CommonTspCpBaseModel):
    def __init__(self, points: list):
        CommonTspCpBaseModel.__init__(self, len(points))
        self.points = points.copy()

        self.transitCallbackIndex = self.routing.RegisterTransitCallback(self.distanceCallback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(self.transitCallbackIndex)

    def calculateEuclidianDistance(self, i, j): 
        return round(math.hypot((self.points[i][0] - self.points[j][0]), (self.points[i][1] - self.points[j][1])))

    def distanceCallback(self, from_index, to_index):
        return self.calculateEuclidianDistance(self.manager.IndexToNode(from_index), self.manager.IndexToNode(to_index))
