#!/usr/local/bin/python3
# -*- coding: utf-8 -*-

# --------------------------------------------------------------------
#   Solving TSP with Google OR-Tools (https://developers.google.com/optimization/routing/tsp)
#   Before running this script, you need to install or-tools v9.4
#
#   Author: Naoki Nishimura <nishimura.n.ab@gmail.com>
#   Date: 2022/08/17
# --------------------------------------------------------------------

# import modules -----------------------------------------------------
import sys
import time
import math
import argparse
import networkx as netx
import matplotlib.pyplot as plt
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# constant -----------------------------------------------------------
TIME_LIMIT = 60  # default time limit for iterated local search

# --------------------------------------------------------------------
#   TSP data
# --------------------------------------------------------------------
class Tsp:
    # constructor ----------------------------------------------------
    def __init__(self):
        self.name = ""  # name of TSP instance
        self.num_node = 0  # number of nodes
        self.coord = []  # coordinate list of nodes

    # read TSP data --------------------------------------------------
    def read(self, args):
        # open file
        input_file = open(args.filename, "r")
        data = input_file.readlines()
        input_file.close()

        # read data
        for i in range(len(data)):
            data[i] = (data[i].rstrip()).split()
            data[i] = list(filter(lambda str: str != ":", data[i]))  # remove colon
            if len(data[i]) > 0:
                data[i][0] = data[i][0].rstrip(":")
                if data[i][0] == "NAME":
                    self.name = data[i][1]
                elif data[i][0] == "TYPE":
                    if data[i][1] != "TSP":
                        print("Problem type is not TSP!")
                        sys.exit(1)
                elif data[i][0] == "DIMENSION":
                    self.num_node = int(data[i][1])
                elif data[i][0] == "EDGE_WEIGHT_TYPE":  # NOTE: accept only EUC_2D
                    if data[i][1] != "EUC_2D":
                        print("Edge weight type is not EUC_2D")
                        sys.exit(1)
                elif data[i][0] == "NODE_COORD_SECTION":
                    sec_coord = i

        # coord section
        self.coord = [(0.0, 0.0)] * self.num_node
        line_cnt = sec_coord + 1
        for i in range(self.num_node):
            (self.coord)[int(data[line_cnt][0]) - 1] = (
                float(data[line_cnt][1]),
                float(data[line_cnt][2]),
            )
            line_cnt += 1

    # print TSP data -------------------------------------------------
    def write(self):
        print("\n[TSP data]")
        print("name:\t{}".format(self.name))
        print("#node:\t{}".format(self.num_node))
        print("coord:\t{}".format(self.coord))

    # calculate distance (rounded euclidian distance in 2D) ----------
    def dist(self, v1, v2):
        xd = float((self.coord)[v1][0] - (self.coord)[v2][0])
        yd = float((self.coord)[v1][1] - (self.coord)[v2][1])
        return int(math.sqrt(xd * xd + yd * yd) + 0.5)


# --------------------------------------------------------------------
#   working data
# --------------------------------------------------------------------
class Work:
    # constructor ----------------------------------------------------
    def __init__(self, tsp):
        self.tour = [i for i in range(tsp.num_node)]  # tour of salesman
        self.obj = self.calc_obj(tsp)  # objective value

    # calculate tour length ------------------------------------------
    def calc_obj(self, tsp):
        length = 0
        for i in range(tsp.num_node):
            length += tsp.dist((self.tour)[i], (self.tour)[(i + 1) % tsp.num_node])
        return length

    # write WORK data ------------------------------------------------
    def write(self, tsp):
        print("\n[Tour data]")
        print("length= {}".format(self.calc_obj(tsp)))

    # draw obtained tour ---------------------------------------------
    def draw(self, tsp):
        graph = netx.Graph()
        graph.add_nodes_from([i for i in range(tsp.num_node)])
        coord = {i: ((tsp.coord)[i][0], (tsp.coord)[i][1]) for i in range(tsp.num_node)}
        netx.add_path(graph, self.tour + [(self.tour)[0]])
        netx.draw(graph, coord, with_labels=True)
        plt.axis("off")
        plt.show()


# OR-Tools function -----------------------------------------------------------
def create_data_model(tsp):
    """Stores the data for the problem."""
    data = {}
    # Locations in block units
    data["locations"] = tsp.coord
    data["num_vehicles"] = 1
    data["depot"] = 0
    return data


def compute_euclidean_distance_matrix(tsp):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter in range(tsp.num_node):
        distances[from_counter] = {}
        for to_counter in range(tsp.num_node):
            distances[from_counter][to_counter] = tsp.dist(from_counter, to_counter)
    return distances


def set_solution(tsp, work, routing, solution):
    # OR-Tools objective value function
    # print("Objective: {}".format(solution.ObjectiveValue()))
    index = routing.Start(0)
    tour = []
    while not routing.IsEnd(index):
        tour.append(index)
        index = solution.Value(routing.NextVar(index))
    work.tour = tour[:]
    # calculate tour length
    work.obj = work.calc_obj(tsp)
    # print tour length
    print("length= {}".format(work.obj))


# --------------------------------------------------------------------
#   running OR-Tools
#
#   tsp(I): TSP data
#   work(I/O): working data
# --------------------------------------------------------------------
def run_or_tools(tsp, work):
    # Instantiate the data problem
    data = create_data_model(tsp)

    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(
        len(data["locations"]), data["num_vehicles"], data["depot"]
    )
    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)
    distance_matrix = compute_euclidean_distance_matrix(tsp)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes"""
        # Convert from routing variable Index to distance matrix NodeIndex
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting search parameters
    # https://developers.google.com/optimization/routing/routing_options
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = TIME_LIMIT
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC
    )

    # Solve the problem.
    print("\n[solving with OR-Tools]")
    solution = routing.SolveWithParameters(search_parameters)

    # set solution to work data
    set_solution(tsp, work, routing, solution)


# --------------------------------------------------------------------
#   parse arguments
#
#   argv(I): arguments
# --------------------------------------------------------------------
def parse_args(argv):
    parser = argparse.ArgumentParser("TSP")
    # input filename of instance
    parser.add_argument("filename", action="store")
    # timelimit for solver
    parser.add_argument(
        "-t",
        "--time",
        help="time limit for iterated local search",
        type=float,
        default=TIME_LIMIT,
    )
    # draw obtained tour
    parser.add_argument("-d", "--draw", action="store_true", help="draw obtained tour")
    return parser.parse_args()


# --------------------------------------------------------------------
#   main
# --------------------------------------------------------------------
def main(argv=sys.argv):
    # parse arguments
    args = parse_args(argv)

    # set starting time
    start_time = time.time()

    # read instance
    tsp = Tsp()
    tsp.read(args)
    tsp.write()

    # solve TSP
    work = Work(tsp)
    # run OR-Tools
    run_or_tools(tsp, work)

    work.write(tsp)
    # set completion time
    end_time = time.time()

    # display computation time
    print("\nTotal time:\t%.3f sec" % (end_time - start_time))

    # draw obtained tour
    if args.draw == True:
        work.draw(tsp)


# main ---------------------------------------------------------------
if __name__ == "__main__":
    main()

# --------------------------------------------------------------------
#   end of file
# --------------------------------------------------------------------
