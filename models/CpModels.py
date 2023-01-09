from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class Matrix_TSP_CP_Model(object):
    def __init__(self, c):
        self.c = c
        self.route = []
        self.routeList = []

        self.manager = pywrapcp.RoutingIndexManager(len(self.c), 1, 0)
        self.routing = pywrapcp.RoutingModel(self.manager)

        self.transitCallbackIndex = self.routing.RegisterTransitCallback(self.distanceCallback)

        self.routing.SetArcCostEvaluatorOfAllVehicles(self.transitCallbackIndex)

        self.searchParameters = pywrapcp.DefaultRoutingSearchParameters()
        # self.searchParameters.first_solution_strategy = (
        #     routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        # )

    def solve(self, totalRuntime=None, printSolutionLog=False, sol_log=False):
        if(totalRuntime != None):
            self.searchParameters.time_limit.seconds = totalRuntime
        self.solution = self.routing.SolveWithParameters(self.searchParameters)

        if self.solution:
            self.updateRouteAndRouteList()

    def distanceCallback(self, from_index, to_index):
        return self.c[self.manager.IndexToNode(from_index)][self.manager.IndexToNode(to_index)]

    def printSolution(self):
        print('Objective Value: {}'.format(self.solution.ObjectiveValue()))
        index = self.routing.Start(0)
        plan_output = 'Route built:\n\n'
        route_distance = 0
        while not self.routing.IsEnd(index):
            plan_output += ' {} ->'.format(self.manager.IndexToNode(index))
            previous_index = index
            index = self.solution.Value(self.routing.NextVar(index))
            route_distance += self.routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += ' {}\n'.format(self.manager.IndexToNode(index))
        print(plan_output)
        plan_output += 'Route distance: {}miles\n'.format(route_distance)

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
