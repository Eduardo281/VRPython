from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class CommonTspCpBaseModel(object):
    def __init__(self, n):
        self.n = n
        self.route = []
        self.routeList = []
        self.objectiveValue = None
        self.solution = None

        self.manager = pywrapcp.RoutingIndexManager(self.n, 1, 0)
        self.routing = pywrapcp.RoutingModel(self.manager)

        self.searchParameters = pywrapcp.DefaultRoutingSearchParameters()

        # self.searchParameters.first_solution_strategy = (
        #     routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        # )

        self.searchParameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )

    def solve(self, totalRuntime=None, printSolverLog=False):
        if(totalRuntime != None):
            self.searchParameters.time_limit.seconds = totalRuntime
        else:
            print("The user need to specify a time limit for the solution procedure!")
            return
        if(printSolverLog):
            self.searchParameters.log_search = True
        self.solution = self.routing.SolveWithParameters(self.searchParameters)

        if self.solution:
            self.updateSolution()

    def printRoute(self):
        print("Objective Value: {}".format(self.objectiveValue))
        index = self.routing.Start(0)
        plan_output = "Route found:\n\n"
        while not self.routing.IsEnd(index):
            plan_output += " {} ->".format(self.manager.IndexToNode(index))
            index = self.solution.Value(self.routing.NextVar(index))
        plan_output += " {}\n".format(self.manager.IndexToNode(index))
        print(plan_output)

    def updateRoute(self):
        self.route = []
        index = self.routing.Start(0)
        while not self.routing.IsEnd(index):
            new_index = self.solution.Value(self.routing.NextVar(index))
            self.route.append((self.manager.IndexToNode(index), self.manager.IndexToNode(new_index)))
            index = new_index

    def updateRouteList(self):
        self.routeList = []
        index = self.routing.Start(0)
        self.routeList = [self.manager.IndexToNode(index)]
        while not self.routing.IsEnd(index):
            index = self.solution.Value(self.routing.NextVar(index))
            self.routeList.append(self.manager.IndexToNode(index))

    def updateRouteAndRouteList(self):
        self.route = []
        self.routeList = []
        index = self.routing.Start(0)
        self.routeList = [self.manager.IndexToNode(index)]
        while not self.routing.IsEnd(index):
            new_index = self.solution.Value(self.routing.NextVar(index))
            self.route.append((self.manager.IndexToNode(index), self.manager.IndexToNode(new_index)))
            self.routeList.append(self.manager.IndexToNode(new_index))
            index = new_index

    def updateSolution(self):
        self.updateRouteAndRouteList()
        self.objectiveValue = self.solution.ObjectiveValue()
        self.solution = {
            "objectiveValue": self.objectiveValue,
            "sequence": self.routeList
        }

    def createTransitCallbackIndex(self, distanceCallbackFunction):
        self.transitCallbackIndex = self.routing.RegisterTransitCallback(distanceCallbackFunction)
        self.routing.SetArcCostEvaluatorOfAllVehicles(self.transitCallbackIndex)
