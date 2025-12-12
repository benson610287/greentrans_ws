import sys

from example_interfaces.srv import AddTwoInts
from tf2_msgs.msg import TFMessage
from interfaces.msg import RobotState

from interfaces.srv import Allinfo
import rclpy
from rclpy.node import Node, Executor
import json


from collections import defaultdict
class RobotMain(Node):

    def __init__(self):
        super().__init__('RobotMain')
        self.subscription = self.create_subscription(
            TFMessage,
            'TB0/tf2',
            self.robot_callback,
            10
        )
        self.subscription = self.create_subscription(
            TFMessage,
            'TB1/tf2',
            self.robot1_callback,
            10
        )
        self.publisher=self.create_publisher(RobotState,"/robot_state",10)
        self.timer=self.create_timer(0.1,self.pub_callback)

        self.robot_state = defaultdict(lambda: defaultdict(dict))
        # print(type(self.task_json))
    def robot_callback(self,msg):
        
        self.robot_state["0"]["Pose2D"]["x"]=msg.transforms[0].transform.translation.x
        self.robot_state["0"]["Pose2D"]["y"]=msg.transforms[0].transform.translation.y
        self.robot_state["0"]["Pose2D"]["z"]=msg.transforms[0].transform.translation.z
        if self.robot_state["0"]["Pose2D"]["x"]>0.0:
            # self.robot_state["0"]["topo"]["previous_node"] = 5
            # self.robot_state["0"]["topo"]["next_node"] = 4
            self.robot_state["0"]["topo"]["previous_node"] = 4
            self.robot_state["0"]["topo"]["next_node"] = 5
        else:
            self.robot_state["0"]["topo"]["previous_node"] = 3
            self.robot_state["0"]["topo"]["next_node"] = 4
        
    def robot1_callback(self,msg):
        self.robot_state["1"]["Pose2D"]["x"]=msg.transforms[0].transform.translation.x
        self.robot_state["1"]["Pose2D"]["y"]=msg.transforms[0].transform.translation.y
        self.robot_state["1"]["Pose2D"]["z"]=msg.transforms[0].transform.translation.z
        if self.robot_state["1"]["Pose2D"]["y"]<0.0:
            # self.robot_state["1"]["topo"]["previous_node"] = 7
            # self.robot_state["1"]["topo"]["next_node"] = 4
            self.robot_state["1"]["topo"]["previous_node"] = 4
            self.robot_state["1"]["topo"]["next_node"] = 7
        else:
            self.robot_state["1"]["topo"]["previous_node"] = 1
            self.robot_state["1"]["topo"]["next_node"] = 4

    def pub_callback(self):
        self.pub_msg=RobotState()
        self.pub_msg.robot_info=json.dumps(self.robot_state)
        self.robot_state = defaultdict(lambda: defaultdict(dict))
        self.publisher.publish(self.pub_msg)

    def send_request(self):
        self.req.data = json.dumps(self.task_json)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=15)
        # print("aaaaa")
        print(self.future.result())
        return self.future.result()



def main(args=None):
    rclpy.init(args=args)

    minimal_client = RobotMain()
    # while 1:
    #     input("press enter to send request")
    #     minimal_client.send_request()
    executor=Executor()
    executor.add_node(minimal_client)
    rclpy.spin(minimal_client)
    print('Hi from fake_main.')


if __name__ == '__main__':
    main()
