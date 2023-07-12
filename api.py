from flask import Flask
from flask_restful import Resource, Api, reqparse
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import json

app = Flask(__name__)
api = Api(app)

def dataFormatting(data, manager, routing, solution):
  response = {};
  response['total_distance'] = 0
  for vehicle_id in range(data['num_of_vehicles']):
      response[vehicle_id] = {}
      response[vehicle_id]['path'] = []
      index = routing.Start(vehicle_id)
      route_distance = 0
      while not routing.IsEnd(index):
          response[vehicle_id]['path'].append(manager.IndexToNode(index))
          previous_index = index
          index = solution.Value(routing.NextVar(index))
          route_distance += routing.GetArcCostForVehicle(
              previous_index, index, vehicle_id)
      response[vehicle_id]['path'].append(manager.IndexToNode(index))
      response[vehicle_id]['route_distance'] = route_distance
      response['total_distance'] += route_distance
  return response

parser = reqparse.RequestParser()

class VRPDataClass(Resource):
  def post(self):
    parser.add_argument("data")
    args = parser.parse_args()
    #convert string to object
    data = json.loads(args["data"])

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_of_vehicles'], data['depot'])
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc.
    def distance_callback(from_index, to_index):
        """Returns the manhattan distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
      response = dataFormatting(data, manager, routing, solution)

    return response

api.add_resource(VRPDataClass, '/vrp/')

if __name__ == "__main__":
  app.run(debug=True)