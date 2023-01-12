from BaseTspCpModels import CommonTspCpBaseModel

class Matrix_TSP_CP_Model(CommonTspCpBaseModel):
    def __init__(self, c):
        CommonTspCpBaseModel.__init__(self, len(c[0]))
        self.c = c

        self.transitCallbackIndex = self.routing.RegisterTransitCallback(self.distanceCallback)
        self.routing.SetArcCostEvaluatorOfAllVehicles(self.transitCallbackIndex)

    def distanceCallback(self, from_index, to_index):
        return self.c[self.manager.IndexToNode(from_index)][self.manager.IndexToNode(to_index)]
