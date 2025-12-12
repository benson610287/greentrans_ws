"""============================================================
# Import packages
============================================================"""
# ros2 packages
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# ros2 interfaces
from std_msgs.msg import Bool
from interfaces.srv import SystemSwitch, PathRequest, SendTask

# other packages
import json

# custom modules
from .task_manager import TaskManager




"""============================================================
# MainController class
============================================================"""
class MainController(Node):

    def __init__(self):
        super().__init__('main_controller')

        # ==================== Task Manager ====================
        self.task_manager = TaskManager(self.get_logger())

        # ======================== flag ========================
        # Emergency Stop flag
        self.e_stop = False

        # ====================== Publisher ======================
        # Emergency Stop Publisher
        self.publisher_ = self.create_publisher(Bool, 'e_stop', 10)
        self.e_stop_timer = self.create_timer(1/10.0, self.e_stop_callback) # 10 Hz

        # ====================== Subscriber =====================


        # ======================= Services ======================
        # UI Switch Service (ui -> controller)
        self.switch_service = self.create_service(SystemSwitch, '/ui/switch', self.switch_service_callback)

        # UI Task Service (ui -> controller)
        self.task_service = self.create_service(SendTask, '/ui/task', self.task_service_callback)

        # Path Planning Service (controller -> path_checker)
        self.path_request_cli = self.create_client(PathRequest, '/path_planning/path_request')
        while not self.path_request_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')


    # ==========================================================
    # Task Request service callback
    # ==========================================================
    def task_service_callback(self, request, response):
        # 1. add task to queue via TaskManager
        success, message, task_dict = self.task_manager.add_task_from_json(request.task_json)
        if not success:
            response.result_json = json.dumps(
                {"success": False, "message": message},
                ensure_ascii=False
            )
            return response
        
        # 2. (TODO) send path request to path planning module
        path_request = PathRequest.Request()
        path_request.task_json = request.task_json
        self.get_logger().info(f'===========================')
        path_response = self.path_request_cli.call(path_request)
        # rclpy.spin_until_future_complete(self, future)
        # if future.result() is not None:
        #     path_response = future.result()
        #     self.get_logger().info(f'Received path response: {path_response.result_json}')
        # else:
        #     self.get_logger().error('Path planning service call failed')
        
        # 3. send request to ui
        self.get_logger().info(f'Received path response: {path_response}')
        response.result_json = json.dumps(
            {"success": True, "message": "Request received and parsed."},
            ensure_ascii=False
        )
        return response


    # ==========================================================
    # Emergency Stop service callback
    # ==========================================================
    def switch_service_callback(self, request, response):
        self.e_stop = request.system_switch    # 更新緊急開關狀態
        if self.e_stop:
            self.get_logger().error('UI sent Emergency Stop!')
        else:
            self.get_logger().info('UI released Emergency Stop!')

        response.success = True
        response.message = 'Emergency Stop state changed successfully.'
        return response


    # ==========================================================
    # Emergency Stop callback
    # ==========================================================
    def e_stop_callback(self):
        msg = Bool()
        msg.data = self.e_stop
        self.publisher_.publish(msg)
        # self.get_logger().info(f'{msg.data}')







"""============================================================
# Main function
============================================================"""
def main(args=None):
    rclpy.init(args=args)
    node = MainController()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()




""" for test only """
# run this node
# ros2 run controller main_controller 

# e_stop service 
# ros2 service call /ui/switch interfaces/srv/SystemSwitch "{system_switch: true}"
# ros2 service call /ui/switch interfaces/srv/SystemSwitch "{system_switch: false}"

# ui task service
# ros2 service call /ui/task interfaces/srv/SendTask "{task_json: '{\"3369577\": {\"type\": \"delivery\", \"robot_id\": null, \"pick\": 1, \"place\": 2}}'}"
