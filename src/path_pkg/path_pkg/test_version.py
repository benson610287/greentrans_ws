import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int64
from interfaces.srv import Allinfo

from cuopt import routing
from cuopt import distance_engine
import numpy as np
import cudf
import json
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.srv = self.create_service(Allinfo, '/path_planning/path_plan', self.task_callback)




        self.factory_open_time = 0
        self.factory_close_time = 100
        self.offsets = []
        self.edges = []
        self.weights = []
        self.final_path = ""
        self.end_node=[]
        self.start_node=[]
    def task_callback(self, request, response):
        start_time = time.time()
        self.get_logger().info(f"recieve {request.data}")
        data=json.loads(request.data)
        data_task_key=list(data["task"].keys())
        self.robot_num=len(data["robot"])
        self.task_num=len(data["task"])
        self.start_node=[]
        self.pickup_node=[]
        self.end_node=[]
        self.get_logger().info(f"========={data_task_key}==============")
        self.get_logger().info(f"Assignment variable robot")

        for i in range(self.robot_num):
            if data["robot"][str(i)]["state"]=="free":
                self.start_node.append(data["robot"][str(i)]["node"])

        self.get_logger().info(f"Assignment variable pickup")
        for i in range(self.task_num):
            self.pickup_node.append(data["task"][data_task_key[i]]["pick"])
            # self.pickup_node.append(data["task"][str(i)]["pickup_node"])
        self.get_logger().info(f"Assignment variable end")

        for i in range(self.task_num):
            self.end_node.append(data["task"][data_task_key[i]]["place"])
            # self.end_node.append(data["task"][str(i)]["end_node"])

        self.get_logger().info(f"create_cost_matrix")
        self.create_cost_matrix()
        self.get_logger().info(f"creat_robot")
        # self.create_mission([0],self.end_node)
        self.creat_robot(self.robot_num)
        self.get_logger().info(f"creat_data_model")
        self.creat_data_model()
        self.get_logger().info(f"compute_path")
        self.compute_path()

        end_time = time.time()
        execution_time = end_time - start_time
        self.get_logger().warning(f'cost time {execution_time}')


        # print(type(data))
        # print(data)
        response.success=True
        response.path=str(self.output_path)
        response.message="done"
        return response

    def load_map(self,map_path):
        with open(map_path, "r") as f:
            map_json = json.load(f)
        self.offsets = np.array(map_json["offsets"])
        self.edges = np.array(map_json["edges"])
        self.weights = np.array(map_json["weights"])
        # print("=============================")
        # print(self.offsets)
        # print(self.edges)
        # print(self.weights)
        # print("=============================")

        
        # return np.array(offsets), np.array(edges), np.array(weights)



    def create_cost_matrix(self):
        # offsets = np.array([0, 2, 5, 7, 9, 12, 14, 16, 18])
        # edges   = np.array([1, 5, 0, 2, 4, 1, 3, 2, 7, 1, 6, 7, 0, 6, 4, 5, 3, 4])
        # weights = np.array([2, 2, 2, 2, 2, 2, 2, 2, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1])
        self.target_locations = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        self.waypoint_graph = distance_engine.WaypointMatrix(
            self.offsets,
            self.edges,
            self.weights
        )
        self.cost_matrix = self.waypoint_graph.compute_cost_matrix(self.target_locations)
        self.transit_time_matrix = self.cost_matrix.copy(deep=True)
        self.target_map = {v:k for k, v in enumerate(self.target_locations)}
        self.index_map = {k:v for k, v in enumerate(self.target_locations)}

    def create_mission(self):
        if not isinstance(self.pickup_node, list):
            self.get_logger().error("start must be list")
            return False
        if not isinstance(self.end_node, list):
            self.get_logger().error("end must be list")
            return False
        if self.pickup_node==[]:
            self.get_logger().error(f'start is empty')
            return False
        if self.end_node==[]:
            self.get_logger().error(f'end is empty')
            return False
        if len(self.pickup_node)!=len(self.end_node):
            self.get_logger().error(f'lenth of start and end is not same')
            return False
        
        print(type(self.pickup_node))
        #==============暫留區==============================
        # order_demand=[2] * len(self.start_node)
        # delivery_location=[0] * len(self.start_node)
        # latest_pickup=[100] * len(self.start_node)
        # pickup_service_time=[0] * len(self.start_node)
        # earliest_delivery=[0] * len(self.start_node)
        # latest_delivery=[100] * len(self.start_node)
        # delivery_serivice_time=[0] * len(self.start_node)
        # self.transport_order_data = cudf.DataFrame({
        #     "pickup_location":       self.start_node,
        #     "delivery_location":     self.end_node,
        #     "order_demand":          [2,1],
        #     "earliest_pickup":       [0,2],
        #     "latest_pickup":         [100,22],
        #     "pickup_service_time":   [0,2],
        #     "earliest_delivery":     [0,2],
        #     "latest_delivery":       [100,22],
        #     "delivery_serivice_time":[0,22]
        # })
        # self.transport_order_data
        #==============暫留區==============================

        order_demand=[2] * len(self.pickup_node)
        delivery_location=[0] * len(self.pickup_node)
        latest_pickup=[100] * len(self.pickup_node)
        pickup_service_time=[0] * len(self.pickup_node)
        earliest_delivery=[0] * len(self.pickup_node)
        latest_delivery=[100] * len(self.pickup_node)
        delivery_serivice_time=[0] * len(self.pickup_node)





        self.transport_order_data = cudf.DataFrame({
            "pickup_location":       self.pickup_node,
            "delivery_location":     self.end_node,
            "order_demand":          order_demand,
            "earliest_pickup":       delivery_location,
            "latest_pickup":         latest_pickup,
            "pickup_service_time":   pickup_service_time,
            "earliest_delivery":     earliest_delivery,
            "latest_delivery":       latest_delivery,
            "delivery_serivice_time":delivery_serivice_time
        })
        self.transport_order_data


        return True


    def creat_robot(self,robot_num: int):
        self.robot_data = {
            "robot_ids": [i for i in range(robot_num)],
            "carrying_capacity":[2] * robot_num
        }
        self.robot_data = cudf.DataFrame(self.robot_data).set_index('robot_ids')
    
    def creat_data_model(self):
        n_locations = len(self.cost_matrix)
        n_vehicles = self.robot_num
        n_orders = len(self.pickup_node) * 2
        self.data_model = routing.DataModel(n_locations, n_vehicles,n_orders)
        self.data_model.add_cost_matrix(self.cost_matrix)
        self.data_model.add_transit_time_matrix(self.transit_time_matrix)
        self.data_model.set_drop_return_trips(cudf.Series([False,False]))
        self.data_model.set_vehicle_locations(cudf.Series(self.start_node),cudf.Series(self.start_node))
        
        #============================設定載重能力(未完成)===================================
        # This is the number of parts that needs to be moved.
        # raw_demand = self.transport_order_data["order_demand"]
        # # When dropping off parts we want to remove one unit of demand from the robot.
        # drop_off_demand = raw_demand * -1
        # # Create pickup and delivery demand.
        # order_demand = cudf.concat([raw_demand, drop_off_demand], ignore_index=True)
        # # Add the capacity dimension.
        # self.data_model.add_capacity_dimension("demand", order_demand, self.robot_data['carrying_capacity'])
        # print(order_demand)
        #==============================設定載重能力(未完成)==================================
        
        #===========設定訂單資訊==============
        pickup_order_locations = cudf.Series(self.pickup_node)
        delivery_order_locations = cudf.Series(self.end_node)
        order_locations = cudf.concat([pickup_order_locations,delivery_order_locations], ignore_index=True)
        # print("====================================================")
        # print(order_locations)
        # print("====================================================")
        #===========設定訂單資訊==============

        #===========設定取貨放貨資訊==============
        self.data_model.set_order_locations(order_locations)
        pickup_indices   = cudf.Series([i for i in range(len(self.pickup_node))])
        delivery_indices = cudf.Series([i + len(self.pickup_node) for i in range(len(self.pickup_node))])

        self.data_model.set_pickup_delivery_pairs(
            pickup_indices, delivery_indices
        )
        #===========設定取貨放貨資訊==============

        #================指定機器人去指定訂單(未完成)====================
        # self.data_model.add_order_vehicle_match(2,cudf.Series(0))
        # self.data_model.add_order_vehicle_match(1,cudf.Series(1))
        self.data_model.set_min_vehicles(min(self.robot_num,self.task_num))
        #================指定機器人去指定訂單(未完成)====================


        
        #============================設定時間窗(未完成)=================================
        # create earliest times
        # vehicle_earliest_time = cudf.Series([self.factory_open_time] * n_vehicles)
        # order_time_window_earliest = cudf.concat([self.transport_order_data["earliest_pickup"], self.transport_order_data["earliest_delivery"]], ignore_index=True)
        # # create latest times
        # vehicle_latest_time = cudf.Series([self.factory_close_time] * n_vehicles)
        # order_time_window_latest = cudf.concat([self.transport_order_data["latest_pickup"], self.transport_order_data["latest_delivery"]], ignore_index=True)
        # # create service times
        # order_service_time = cudf.concat([self.transport_order_data["pickup_service_time"], self.transport_order_data["delivery_serivice_time"]], ignore_index=True)
        # # add time window constraints
        # self.data_model.set_order_time_windows(order_time_window_earliest, order_time_window_latest)
        # self.data_model.set_order_service_times(order_service_time)
        # self.data_model.set_vehicle_time_windows(vehicle_earliest_time, vehicle_latest_time)
        #============================設定時間窗(未完成)=================================

    def compute_path(self):
        solver_settings = routing.SolverSettings()
        solver_settings.set_time_limit(10)

        routing_solution = routing.Solve(self.data_model, solver_settings)
        if routing_solution.get_status() == 0:
            print("Cost for the routing in time: ", routing_solution.get_total_objective())
            print("Vehicle count to complete routing: ", routing_solution.get_vehicle_count())
            # print(routing_solution.route)
        else:
            print("NVIDIA cuOpt Failed to find a solution with status : ", routing_solution.get_status())

        target_loc_route = [self.index_map[loc] for loc in routing_solution.route['location'].to_arrow().to_pylist()]
        routing_solution.route['order_array_index'] = routing_solution.route['route']
        routing_solution.route['route'] = target_loc_route
        print(routing_solution.route)

        # print("def order")
        # print("\n\n")


        unique_robot_ids = routing_solution.route['truck_id'].unique()
        all_routes = routing_solution.get_route()
        self.output_path={}
        
        for robot in unique_robot_ids.to_arrow().to_pylist():
            tmp=[]
            route = all_routes[all_routes['truck_id']==robot]
            waypoint_route = self.waypoint_graph.compute_waypoint_sequence(self.target_locations, route)
            print(f"Target location level route for robot {robot}:\n{all_routes[all_routes['truck_id']==robot]['route']}\n\n")
            print(f"Waypoint level route for robot {robot}:\n{waypoint_route}\n\n")

            # self.get_logger().error(f"{type(waypoint_route.values_host)}\n")
            # self.get_logger().error(f"===={waypoint_route.values_host}=====")
            # self.get_logger().error(f"+++++++{waypoint_route.values_host[0][0]}+++++++++")
            
            for i in waypoint_route.values_host:
                if tmp==[]:
                    tmp.append(i[0])
                elif i[0] == tmp[-1]:
                    pass
                else:
                    tmp.append(i[0])
            self.output_path.setdefault(str(robot),tmp)


        # for i in waypoint_route.values_host:
        #     self.output_path.add(i[0])
        #     if i[1]=="Delivery":
        #         # print("aa")
        #         break
        self.output_path=json.dumps(self.output_path)
        self.get_logger().info(f'route={self.output_path}')
        self.get_logger().info(f'route={type(self.output_path)}')

        
        
    



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    minimal_subscriber.load_map("map.json")
    # print(minimal_subscriber.create_mission([1,2],[2,2]))
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()