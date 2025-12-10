from example_interfaces.srv import AddTwoInts
from rclpy.executors import  MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
#MutuallyExclusiveCallbackGroup 是不能同時有兩個callback
#ReentrantCallbackGroup 是同時可以有，但是要小心改變同個變數
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from interfaces.srv import Allinfo , PathRequest
import json

from std_msgs.msg import String


class PathChcker(Node):

    def __init__(self):
        super().__init__('path_checker')
        path_request_group = ReentrantCallbackGroup()
        map_group = MutuallyExclusiveCallbackGroup()

        self.map_server = self.create_service(AddTwoInts, '/map_info_csr', self.map_update_callback,callback_group=map_group)
        self.path_req_server = self.create_service(PathRequest, '/path_planning/path_request', self.path_request_callback,callback_group=path_request_group)
        self.path_plan_client = self.create_client(Allinfo, '/path_planning/path_plan',callback_group=path_request_group)
        self.subscription = self.create_subscription(
            String,
            '/robot_state',
            self.robot_callback,
            10
        )
        while not self.path_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('cuOpt server not available, waiting again...')


        self.req = Allinfo.Request()
        with open("task_map.json", "r") as f:
            self.task_json = json.load(f)

        with open("self_map.json", "r") as f:
            self.map_json = json.load(f)
        
        self.path=None
        self.cuopt_response=Allinfo.Response()

    def map_update_callback(self, request, response):  #not yet
        # response.sum = request.a + request.b
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response
    
    def path_request_callback(self, request, response):
        self.get_logger().info(f'Incoming request\ndata: {request.task_json}')
        # temp = json.dumps(self.task_json)
        self.get_logger().info(f'========')

        self.path_request=json.loads(request.task_json)
        self.get_logger().info(f'========')

        self.path_request_key=list(self.path_request.keys())
        self.get_logger().info(f'========')

        self.path_response = self.path_request
        self.get_logger().info(f'========')

        self.path_response_key = list(self.path_response.keys())
        self.get_logger().info(f'========{self.path_response_key}=====')

        self.send_request(self.path_request)
        self.get_logger().info(f'========')
        response.result_json=json.dumps(self.path_response)
        return response

        # if request.data==True:
        #     flag=self.send_request()
        #     if flag:
        #         response.success=True
        #         response.message=self.path
        #         return response
        #     else:
        #         response.success=False
        #         response.message="route error"
        #         return response
        # else:
        #     response.success=False
        #     response.message="recieve False"
        #     return response
    
        

    def path_checker(self,robot_id:int,path:list):
        for i in range(len(path)-1):
            if path[i+1] not in self.map_json["node"][str(path[i])]["neighbors"]:
                self.get_logger().info(f'check fail {robot_id}\'s path is miss')
                self.get_logger().error(f'node {path[i+1]} is not connect with node {self.map_json["node"][str(path[i])]["neighbors"]}')
                return True
        self.get_logger().info(f'checked complete robot{robot_id}\'s path is {path}')
        return False
        # task_json
        # {
        #     "task_0": {
        #         "type": "delivery",
        #         "robot_id": null,
        #         "pick": 1,
        #         "place": 2,
        #     },
        #     "task_1": {
        #         "type": "delivery",
        #         "robot_id": null,
        #         "pick": 1,
        #         "place": 2,
        #     }
        # }
    def send_request(self,task_dict):
        # self.req.data = json.dumps(self.task_json)
        self.get_logger().info(f'---------------')
        self.robot_dict=self.task_json["robot"]
        self.get_logger().info(f'--------------')
        # self.task_dict=json.loads(task_json)
        # self.get_logger().info(f'---------------')
        merge={
            "robot": self.robot_dict,
            "task": task_dict
        }
        self.get_logger().info(f'========')
        self.get_logger().info(f'merge \ndata: {merge}')
        self.req.data = json.dumps(merge)


        self.cuopt_response= self.path_plan_client.call(self.req)
        self.get_logger().info(f'recieve cuOpt response\ndata: {self.cuopt_response}')
        # self.path_uncheck = [int(x) for x in self.cuopt_response.path.strip('{}').split(',')]
        self.get_logger().info(f'{type(self.cuopt_response.path)}')
        self.get_logger().info(f'{self.cuopt_response.path}')
        self.path_uncheck=json.loads(self.cuopt_response.path)
        self.get_logger().info(f'{type(self.path_uncheck)}')
        self.get_logger().info(f'{self.cuopt_response}')
        for robot_id, path_list in self.path_uncheck.items():
            task_key = self.path_response_key[int(robot_id)]
            if self.path_checker(robot_id,path_list):
                # self.path_response[self.path_response[self.path_response_key[robot_id]]]["robot_id"]=int(robot_id)+1
                # self.path_response[self.path_response[self.path_response_key[robot_id]]].setdefault("path",path_list)
                # self.path_response[self.path_response[self.path_response_key[robot_id]]].setdefault("success",False)
                # self.path_response[self.path_response[self.path_response_key[robot_id]]].setdefault("message","path  error")
                self.path_response[task_key]["robot_id"] = int(robot_id)+1
                self.path_response[task_key]["success"] = False
                self.path_response[task_key]["message"] = "path  error"
                self.path_response[task_key]["path"] = path_list
                self.get_logger().error('path error')
            else:
                pass
                self.path_response[task_key]["robot_id"] = int(robot_id)+1
                self.path_response[task_key]["success"] = True
                self.path_response[task_key]["message"] = "complete"
                self.path_response[task_key]["path"] = path_list
                # self.path_response[self.path_response[self.path_response_key[int(robot_id)]]].setdefault("robot_id",int(robot_id)+1)
                # self.path_response[self.path_response[self.path_response_key[robot_id]]]["robot_id"]=int(robot_id)+1
                # self.path_response[self.path_response[self.path_response_key[robot_id]]].setdefault("path",path_list)
                # self.path_response[self.path_response[self.path_response_key[robot_id]]].setdefault("success",True)
                # self.path_response[self.path_response[self.path_response_key[robot_id]]].setdefault("message","complete")
        # print("=======")

        print(type(self.cuopt_response.path))
        # self.path=json.dumps(self.cuopt_response.path)
        # self.path=self.cuopt_response.path
        return True
    


    def future_callback(self, future):
        try:
            result = future.result()
            print(result)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def robot_callback(self,msg):
        self.get_logger().info(f'recieve data: {msg.data}')


def main():

    rclpy.init()


    print(f'=============')
    executor = MultiThreadedExecutor(num_threads=3)
    path_chcker = PathChcker()

    executor.add_node(path_chcker)
    print(f'=============')
    executor.spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()




###準備要加callback_group