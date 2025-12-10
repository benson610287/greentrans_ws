import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from  nav_msgs.msg import Odometry

import threading as thread
import json
import math

def is_point_on_segment(P, A, B, tol=0.05):
    """判斷 P 是否在線段 AB 上（容忍誤差 tol，可以調整）"""

    (px, py) = P
    (ax, ay) = A
    (bx, by) = B

    # 向量
    AP = (px - ax, py - ay)
    AB = (bx - ax, by - ay)
    BP = (px - bx, py - by)
    BA = (ax - bx, ay - by)

    # 叉積判斷是否共線（容忍一些誤差）
    cross = AP[0] * AB[1] - AP[1] * AB[0]
    if abs(cross) > tol:
        return False

    # 點積判斷是否在 A 與 B 之間
    dot1 = AP[0] * AB[0] + AP[1] * AB[1]
    dot2 = BP[0] * BA[0] + BP[1] * BA[1]

    if dot1 < 0 or dot2 < 0:
        return False

    return True


def locate_robot_on_map(robot_pos, all_map_info):
    """返回 robot 所在的 edge ID，如果不在任何 edge 回傳 None"""

    px, py = robot_pos

    for edge_id, edge in all_map_info["edge"].items():
        n1, n2 = edge["connector"]

        A = all_map_info["node"][str(n1)]["pos"]
        B = all_map_info["node"][str(n2)]["pos"]

        if is_point_on_segment((px, py), A, B):
            return int(edge_id)

    return None




class Intersection():
    def __init__(self,intersection_node: int= None ):
        self.intersection_node=intersection_node
        self.traffic_light=dict()
        self.road_type=dict()  #road_type: list= ["narrow","wide","wide","wide"]
        self.can_pass_over=None  #can_pass_over: bool= True
        self.road_pos=[]
        for i in range(len(all_map_info["edge"])):
            if self.intersection_node in all_map_info["edge"][str(i)]["connector"]:
                self.road_type.setdefault(str(all_map_info["edge"][str(i)]["connector"][0])+'_'+str(all_map_info["edge"][str(i)]["connector"][1]),all_map_info["edge"][str(i)]["road_type"])
        tmp=list(self.road_type.items())

        # print(tmp)
        # print("\n\n\n")
        # print(type(tmp))
        for i in range(len(tmp)):
                for j in range(len(tmp)):
                    if i!=j:
                        self.traffic_light.setdefault(str(tmp[i][0])+"-"+str(tmp[j][0]),"green")
                        self.road_pos.append(())

        if len(self.road_type)>2:
            self.can_pass_over=True
        else:
            self.can_pass_over=False
        print(
            f"{self.intersection_node}.intersection_node:\t{self.intersection_node}\n"
            f"{self.intersection_node}.road_type:\t{self.road_type}\n"
            f"{self.intersection_node}.traffic_light:\t{self.traffic_light}\n"
            f"{self.intersection_node}.can_pass_over:\t{self.can_pass_over}\n"
            
        )



class Robot_state_subscriber(Node):

    def __init__(self):
        global robot_state 
        super().__init__('Robot_state_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/turtlebot3_burger_ROS/odom',
            self.robot_callback,
            10
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/turtlebot3_burger_ROS_01/odom',
            self.robot1_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.timer=self.create_timer(0.1,self.calc_callback)



        self.intersection=[]
        # for i in range(len(all_map_info["node"])):  #should create thread(not done)
        #     self.intersection.append(Intersection(i))
        self.intersection.append(Intersection(0))


        #判斷機器人在哪個路段
        # robot_pos = (2.1, 0)   # 現在機器人的座標
        # edge_id = locate_robot_on_map(robot_pos, all_map_info)
        # if edge_id is None:
        #     print("Robot is not on any road.")
        # else:
        #     print(f"Robot is on edge {edge_id}")
        #判斷機器人在哪個路段
        self.robot_state=[]
        self.robot1_state=[]


    def robot_callback(self,msg):
        self.get_logger().info(f'\nrobot_state: \n\tx: {msg.pose.pose.position.x}\n\ty: {msg.pose.pose.position.y}\n\tz: {msg.pose.pose.position.z}')
        self.robot_state=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]    


    def robot1_callback(self,msg):
        self.get_logger().info(f'\nrobot1_state: \n\tx: {msg.pose.pose.position.x}\n\ty: {msg.pose.pose.position.y}\n\tz: {msg.pose.pose.position.z}')
        self.robot1_state=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        

    def calc_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.pose.pose.position)
        for i in range(2):
            robot_pos = (2.1, 0)   # 現在機器人的座標
            edge_id = locate_robot_on_map(robot_pos, all_map_info)
            if edge_id is None:
                print("Robot is not on any road.")
            else:
                print(f"Robot is on edge {edge_id}")



        
        



def main(args=None):
    global all_map_info
    with open("./self_map.json",'r') as json_file:
        all_map_info=json.load(json_file)
        # print(all_map_info["edge"]['0']["connector"])
    # for i in range(len(all_map_info["node"])):  #should create thread(not done)
    #     intersection(i)
    rclpy.init(args=args)

    robot_state_subscriber = Robot_state_subscriber()

    rclpy.spin(robot_state_subscriber)

    robot_state_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    