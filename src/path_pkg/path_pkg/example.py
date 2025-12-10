from cuopt import routing
from cuopt import distance_engine
import numpy as np
import cudf
# class ttt():
#     def __init__(self):
#         pass
#     def pp(self):
#         print('pp')
factory_open_time = 0
factory_close_time = 100
# offsets = np.array([0, 1, 3, 7, 9, 11, 13, 15, 17, 20, 22])
# edges =   np.array([2, 2, 4, 0, 1, 3, 5, 2, 6, 1, 7, 2, 8, 3, 9, 4, 8, 5, 7, 9, 6, 8])
# weights = np.array([2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 2, 1, 1, 2, 1, 2, 2, 1, 2])
# target_locations = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

offsets = np.array([0, 2, 5, 7, 9, 12, 14, 16, 18])
edges   = np.array([1, 5, 0, 2, 4, 1, 3, 2, 7, 1, 6, 7, 0, 6, 4, 5, 3, 4])
weights = np.array([2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1])
target_locations = np.array([0, 1, 2, 3, 4, 5, 6, 7])



waypoint_graph = distance_engine.WaypointMatrix(
    offsets,
    edges,
    weights
)

cost_matrix = waypoint_graph.compute_cost_matrix(target_locations)
transit_time_matrix = cost_matrix.copy(deep=True)
target_map = {v:k for k, v in enumerate(target_locations)}
index_map = {k:v for k, v in enumerate(target_locations)}
print(f"Waypoint graph node to time matrix index mapping \n{target_map}\n")
print(cost_matrix)
print("def path_distance")
print("\n\n")




# transport_order_data = cudf.DataFrame({
#     "pickup_location":       [4,5],
#     "delivery_location":     [6,6],
#     "order_demand":          [1,2],
#     "earliest_pickup":       [0,0],
#     "latest_pickup":         [10,10],
#     "pickup_service_time":   [2,2],
#     "earliest_delivery":     [0,0],
#     "latest_delivery":       [45,45],
#     "delivery_serivice_time":[2,2]
# })
# transport_order_data
transport_order_data = cudf.DataFrame({
    "pickup_location":       [7],
    "delivery_location":     [7],
    "order_demand":          [2],
    "earliest_pickup":       [0],
    "latest_pickup":         [100],
    "pickup_service_time":   [0],
    "earliest_delivery":     [0],
    "latest_delivery":       [100],
    "delivery_serivice_time":[0]
})
transport_order_data




n_robots = 2
robot_data = {
    "robot_ids": [i for i in range(n_robots)],
    "carrying_capacity":[2, 4]
}
robot_data = cudf.DataFrame(robot_data).set_index('robot_ids')
robot_data


n_locations = len(cost_matrix)
n_vehicles = len(robot_data)

# a pickup order and a delivery order are distinct with additional pad for the depot with 0 demand
n_orders = len(transport_order_data) * 2
data_model = routing.DataModel(n_locations, n_vehicles, n_orders)
data_model.add_cost_matrix(cost_matrix)
data_model.add_transit_time_matrix(transit_time_matrix)
data_model.set_drop_return_trips(cudf.Series([False,False]))





# This is the number of parts that needs to be moved.
raw_demand = transport_order_data["order_demand"]

# When dropping off parts we want to remove one unit of demand from the robot.
drop_off_demand = raw_demand * -1

# Create pickup and delivery demand.
order_demand = cudf.concat([raw_demand, drop_off_demand], ignore_index=True)
# Add the capacity dimension.
data_model.add_capacity_dimension("demand", order_demand, robot_data['carrying_capacity'])

# print(raw_demand)
print(order_demand)




pickup_order_locations = cudf.Series([target_map[loc] for loc in transport_order_data['pickup_location'].to_arrow().to_pylist()])
delivery_order_locations = cudf.Series([target_map[loc] for loc in transport_order_data['delivery_location'].to_arrow().to_pylist()])
order_locations = cudf.concat([pickup_order_locations, delivery_order_locations], ignore_index=True)

print(order_locations)

# add order locations
data_model.set_order_locations(order_locations)

print("def order")
print("\n\n")




# IMPORTANT NOTE : Pickup and delivery pairs are indexed into the order locations array.
npair_orders = int(len(order_locations)/2)
pickup_orders = cudf.Series([i for i in range(npair_orders)])
delivery_orders = cudf.Series([i + npair_orders for i in range(npair_orders)])
# Add pickup and delivery pairs.
data_model.set_pickup_delivery_pairs(pickup_orders, delivery_orders)






# create earliest times
vehicle_earliest_time = cudf.Series([factory_open_time] * n_vehicles)
order_time_window_earliest = cudf.concat([transport_order_data["earliest_pickup"], transport_order_data["earliest_delivery"]], ignore_index=True)

# create latest times
vehicle_latest_time = cudf.Series([factory_close_time] * n_vehicles)
order_time_window_latest = cudf.concat([transport_order_data["latest_pickup"], transport_order_data["latest_delivery"]], ignore_index=True)

# create service times
order_service_time = cudf.concat([transport_order_data["pickup_service_time"], transport_order_data["delivery_serivice_time"]], ignore_index=True)

# add time window constraints
data_model.set_order_time_windows(order_time_window_earliest, order_time_window_latest)
data_model.set_order_service_times(order_service_time)
data_model.set_vehicle_time_windows(vehicle_earliest_time, vehicle_latest_time)




solver_settings = routing.SolverSettings()

# solver_settings will run for given time limit.  Larger and/or more complex problems may require more time.
solver_settings.set_time_limit(10)



routing_solution = routing.Solve(data_model, solver_settings)
if routing_solution.get_status() == 0:
    print("Cost for the routing in time: ", routing_solution.get_total_objective())
    print("Vehicle count to complete routing: ", routing_solution.get_vehicle_count())
    # print(routing_solution.route)
else:
    print("NVIDIA cuOpt Failed to find a solution with status : ", routing_solution.get_status())



target_loc_route = [index_map[loc] for loc in routing_solution.route['location'].to_arrow().to_pylist()]
routing_solution.route['order_array_index'] = routing_solution.route['route']
routing_solution.route['route'] = target_loc_route
print(routing_solution.route)

# print("def order")
# print("\n\n")


unique_robot_ids = routing_solution.route['truck_id'].unique()
all_routes = routing_solution.get_route()

for robot in unique_robot_ids.to_arrow().to_pylist():
    route = all_routes[all_routes['truck_id']==robot]
    waypoint_route = waypoint_graph.compute_waypoint_sequence(target_locations, route)
    print(f"Target location level route for robot {robot}:\n{all_routes[all_routes['truck_id']==robot]['route']}\n\n")
    print(f"Waypoint level route for robot {robot}:\n{waypoint_route}\n\n")