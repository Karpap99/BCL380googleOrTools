import math
import time

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def read_data(path): #reading file with data, your file must be as bcl380 in repository
    data = {}
    with open(path, 'r') as f:
        for each in f:
            x = each.split(" ")
            data[str(x[0])] = ((int(x[1])*10,int(x[2])*10))
    return data


def euclid_distance(A, B): #calculate distance within points
    return int(math.sqrt(((B[0]-A[0])**2) + ((B[1]-A[1])**2)))


def create_data_model(datalist):
    """Stores the data for the problem."""
    data = {}
    distances = []
    for i in datalist:
        distances_for_one = []
        for j in datalist:
            distances_for_one.append(euclid_distance(datalist[i], datalist[j]))
        distances.append(distances_for_one)
    data["distance_matrix"] = distances
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data


def print_solution(manager, routing, solution, solutionprice):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()} miles")
    print(f"solution error:{int(100 - (solutionprice / solution.ObjectiveValue() * 100))}%")
    index = routing.Start(0)
    plan_output = "Route for vehicle 0:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += f" {manager.IndexToNode(index)} ->"
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += f" {manager.IndexToNode(index)}\n"
    print(plan_output)
    plan_output += f"Route distance: {route_distance}miles\n"


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    solution_price = int(input("input solution price>"))
    data = create_data_model(read_data("bcl380.tsp"))
    start = time.time_ns()
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    end = time.time_ns()
    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution, solution_price)
    print(f"solution time:{(end - start) / 1000000000} seconds")

if __name__ == "__main__":
    main()