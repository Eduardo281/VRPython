from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

class TSP_CP_Model(object):
    def __init__(self, c, points=None, dist_fun=None, scaler=None):
        self.c = c
        self.route = []
        self.routeList = []

        self.manager = pywrapcp.RoutingIndexManager(len(self.c), 1, 0)
        self.routing = pywrapcp.RoutingModel(self.manager)

        self.transit_callback_index = self.routing.RegisterTransitCallback(self.distance_callback)

        self.routing.SetArcCostEvaluatorOfAllVehicles(self.transit_callback_index)

        self.search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        self.search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )

        self.solution = self.routing.SolveWithParameters(self.search_parameters)

        if self.solution:
            self.print_solution()

    def solve(self, time=None, log=False, sol_log=False):
        pass

    def distance_callback(self, from_index, to_index):
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.c[from_node][to_node]

    def print_solution(self):
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

    def updateRouteList(self):
        """Get vehicle routes from a solution and store them in an array."""
        # Get vehicle routes and store them in a two dimensional array whose
        # i,j entry is the jth location visited by vehicle i along its route.
        self.routeList = []
        index = self.routing.Start(0)
        self.routeList = [self.manager.IndexToNode(index)]
        while not self.routing.IsEnd(index):
            index = self.solution.Value(self.routing.NextVar(index))
            self.routeList.append(self.manager.IndexToNode(index))