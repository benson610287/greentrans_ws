import sys

from example_interfaces.srv import AddTwoInts
from interfaces.srv import Allinfo
import rclpy
from rclpy.node import Node, Executor
import json

class Mission_client(Node):

    def __init__(self):
        super().__init__('mission_client')
        self.cli = self.create_client(Allinfo, 'aa')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # self.req = AddTwoInts.Request()
        self.req = Allinfo.Request()
        with open("task_map.json", "r") as f:
            self.task_json = json.load(f)
        # print(type(self.task_json))
    def send_request(self):
        self.req.data = json.dumps(self.task_json)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=15)
        # print("aaaaa")
        print(self.future.result())
        return self.future.result()






def main(args=None):
    rclpy.init(args=args)

    minimal_client = Mission_client()
    while 1:
        input("press enter to send request")
        minimal_client.send_request()
    # executor=Executor()
    # executor.add_node(minimal_client)
    # rclpy.spin(minimal_client)
    print('Hi from fake_main.')


if __name__ == '__main__':
    main()
