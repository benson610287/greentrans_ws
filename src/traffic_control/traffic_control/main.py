import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from  nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from interfaces.msg import RobotState
import threading as thread



import json
import math
from collections import defaultdict
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
        self.road_type={f'{self.intersection_node}' : {}}  #road_type: list= ["narrow","wide","wide","wide"]
        self.can_pass_over=None  #can_pass_over: bool= True
        # self.road_pos=[]
        for i in range(len(all_map_info["edge"])):
            if self.intersection_node in all_map_info["edge"][str(i)]["connector"]:
                for j in all_map_info["edge"][str(i)]["connector"]:
                    if j != self.intersection_node:
                        self.road_type[f'{self.intersection_node}'].setdefault(f"{j}",{"type":"wide", "robot_in": False, "robot_id": None})
                # self.road_type.setdefault(str(all_map_info["edge"][str(i)]["connector"][0])+'_'+str(all_map_info["edge"][str(i)]["connector"][1]),all_map_info["edge"][str(i)]["road_type"])
        tmp=list(self.road_type.items())


        #=================原本的紅綠燈資料格式=========================
        # print(tmp)
        # print("\n\n\n")
        # print(type(tmp))
        # for i in range(len(tmp)):
        #         for j in range(len(tmp)):
        #             if i!=j:
        #                 self.traffic_light.setdefault(str(tmp[i][0])+"-"+str(tmp[j][0]),"green")
        #                 self.road_pos.append(())
        #=================原本的紅綠燈資料格式=========================
        #=================新的紅綠燈資料格式=========================
        self.traffic_light={str(self.intersection_node): {}}
        
        for i in all_map_info["node"][str(self.intersection_node)]["neighbors"]:
            self.traffic_light[str(self.intersection_node)].setdefault(str(i),{})
            for j in all_map_info["node"][str(self.intersection_node)]["neighbors"]:
                if i!=j:
                    self.traffic_light[str(self.intersection_node)][str(i)].setdefault(j ,"green")
        #=================新的紅綠燈資料格式=========================




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
# road_type:{'4': {'1': {'type': 'wide', 'car_in': False}, '3': {'type': 'wide', 'car_in': False}, '5': {'type': 'wide', 'car_in': False}, '7': {'type': 'wide', 'car_in': False}}}
# traffic_light:{'4': {'1': {3: 'green', 5: 'green', 7: 'green'}, '3': {1: 'green', 5: 'green', 7: 'green'}, '5': {1: 'green', 3: 'green', 7: 'green'}, '7': {1: 'green', 3: 'green', 5: 'green'}}}

    def update_traffic_light(self):
        for i in self.road_type[f'{self.intersection_node}']:
            if self.road_type[f'{self.intersection_node}'][i]["robot_in"] == True:
                for j in self.traffic_light[f'{self.intersection_node}'][i]:
                    self.traffic_light[f'{self.intersection_node}'][i][j] = "red"
            else:
                for j in self.traffic_light[f'{self.intersection_node}'][i]:
                    self.traffic_light[f'{self.intersection_node}'][i][j] = "green"
                    # print("++++++++")
        # print(self.traffic_light[f'{self.intersection_node}'])

    def lock_robot_in(self,prev_node: str, next_node: str, robot_id: str):
        # pass
        # print(f'prev_node={prev_node}\n next_ode={next_node}')
        if next_node == str(self.intersection_node):  #要來節點的路上
            self.road_type[f'{self.intersection_node}'][prev_node]["robot_in"]=True  
            self.road_type[f'{self.intersection_node}'][prev_node]["robot_id"]=robot_id
            print("lock")
        
        
        # print("[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]")
        # print(self.road_type[f'{self.intersection_node}'])
        # print("[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]")
        self.update_traffic_light()

    def unlock_robot_in(self,prev_node: str, next_node: str, robot_id: str):
        for i in self.road_type[f'{self.intersection_node}']:
            if self.road_type[f'{self.intersection_node}'][i]["robot_id"]== robot_id:
                self.road_type[f'{self.intersection_node}'][i]["robot_in"]=False 
                self.road_type[f'{self.intersection_node}'][i]["robot_id"]=None
        # if prev_node == str(self.intersection_node):
        #     self.road_type[f'{self.intersection_node}'][next_node]["robot_in"]=False  
        #     self.road_type[f'{self.intersection_node}'][prev_node]["robot_id"]=None
        #     print("unlock")
        
        # print("========================================")
        # print(self.road_type[f'{self.intersection_node}'])
        # print("========================================")
        self.update_traffic_light()


# road_type:{'4': {'1': {'type': 'wide', 'car_in': False}, '3': {'type': 'wide', 'car_in': False}, '5': {'type': 'wide', 'car_in': False}, '7': {'type': 'wide', 'car_in': False}}}
# traffic_light:{'4': {'1': {3: 'green', 5: 'green', 7: 'green'}, '3': {1: 'green', 5: 'green', 7: 'green'}, '5': {1: 'green', 3: 'green', 7: 'green'}, '7': {1: 'green', 3: 'green', 5: 'green'}}}
    def lock_all_light(self,prev_node: str, next_node: str, robot_id: str):
        for i in self.road_type[f'{self.intersection_node}']:
            if self.road_type[f'{self.intersection_node}'][i];
        #     if self.road_type[f'{self.intersection_node}'][i]["robot_id"]== robot_id:
        #         self.road_type[f'{self.intersection_node}'][i]["robot_in"]=False 
        #         self.road_type[f'{self.intersection_node}'][i]["robot_id"]=None
        pass






    def clear_robot_in(self):
        for i in self.road_type[f'{self.intersection_node}']:
            self.road_type[f'{self.intersection_node}'][i]["robot_in"]=False

            


class Robot_state_subscriber(Node):

    def __init__(self):
        global robot_state 
        super().__init__('Robot_state_subscriber')
        self.subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.robot_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        # self.timer=self.create_timer(0.3,self.calc_callback)


        #　需要交管的路口節點
        self.intersection=dict()
        # for i in range(len(all_map_info["node"])):  #should create thread(not done)
        #     self.intersection.append(Intersection(i))
        self.intersection.setdefault("4",None)
        self.intersection.setdefault("1",None)
        self.intersection.setdefault("3",None)
        self.intersection.setdefault("5",None)
        self.intersection.setdefault("7",None)
        self.intersection["4"]=Intersection(4)
        self.intersection["1"]=Intersection(1)
        self.intersection["3"]=Intersection(3)
        self.intersection["5"]=Intersection(5)
        self.intersection["7"]=Intersection(7)

        # self.robot_state=dict()
        #判斷機器人在哪個路段
        # robot_pos = (2.1, 0)   # 現在機器人的座標
        # edge_id = locate_robot_on_map(robot_pos, all_map_info)
        # if edge_id is None:
        #     print("Robot is not on any road.")
        # else:
        #     print(f"Robot is on edge {edge_id}")
        #判斷機器人在哪個路段
        self.robot_state=dict()
        self.robot_in_edge=dict()
        # self.robot_state = defaultdict(dict)
        # self.robot1_state=[]


    def robot_callback(self,msg):
        msg_dict=json.loads(msg.robot_info)
        prev_node=[]
        next_node=[]
        # print(msg_dict)
        for robot in msg_dict:
            # print(f'{type(msg_dict[robot]["topo"]["next_node"])}')
            if msg_dict[robot]["topo"]["next_node"] == 4:
                self.intersection[f'{msg_dict[robot]["topo"]["next_node"]}'].lock_robot_in(f'{msg_dict[robot]["topo"]["previous_node"]}',f'{msg_dict[robot]["topo"]["next_node"]}',f'{robot}')
            elif msg_dict[robot]["topo"]["previous_node"] == 4:
                self.intersection[f'{msg_dict[robot]["topo"]["previous_node"]}'].unlock_robot_in(f'{msg_dict[robot]["topo"]["previous_node"]}',f'{msg_dict[robot]["topo"]["next_node"]}',f'{robot}')
            # print("==========")
            # self.get_logger().info(f'{robot}=={self.intersection[str(msg_dict[robot]["topo"]["next_node"])].road_type}')
            # self.get_logger().info(f'{self.intersection[str(msg_dict[robot]["topo"]["next_node"])].traffic_light}')
            # else:
            #     pass
        

        self.get_logger().info(f'{self.intersection[str(4)].traffic_light}')


        # self.get_logger().info(f'\nrobot_state: \n\tx: {msg.transforms[0].transform.translation}')
        pass


    def robot1_callback(self,msg):
        # self.get_logger().info(f'\nrobot_state: \n\tx: {msg.transforms[0].transform.translation}')
        self.robot_state.setdefault("1", {})
        self.robot_state["1"]["pos"]=msg.transforms[0].transform.translation
        # self.robot1_state=[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        

    def calc_callback(self):
        if "0" in self.robot_state and "1" in self.robot_state:
            # self.get_logger().info(f"robot={self.robot_state['0']['pos']}\n robot1={self.robot_state['1']['pos']}")
            for i in range(2):
                robot_pos = (self.robot_state[str(i)]['pos'].x,self.robot_state[str(i)]['pos'].y)   # 現在機器人的座標
                self.robot_in_edge.setdefault(str(i),None)
                self.robot_in_edge[str(i)]=locate_robot_on_map(robot_pos, all_map_info)
                # edge_id = locate_robot_on_map(robot_pos, all_map_info)
                if self.robot_in_edge[str(i)] is not None:
                    print(f"Robot{i} is on edge {self.robot_in_edge[str(i)]}")
                    tmp_connect=all_map_info["edge"][f"{self.robot_in_edge[str(i)]}"]["connector"]
                    # print(tmp_connect)
                    # print(type(tmp_connect))
                for j in self.intersection:
                    if j.intersection_node == 4:
                        for k in j.traffic_light[f"{j.intersection_node}"].keys():
                            if k not in tmp_connect:
                                j.traffic_light[f"{j.intersection_node}"][k]
                                pass
                            
                #             # j.traffic_light[str(j.intersection_node)][all_map_info["edge"][f'{self.robot_in_edge[str(i)]}']["connector"][0]]
                #             print("aaa")

                else:
                    print(f"Robot{i} is not on any road.")
        else:
            self.get_logger().info(f"robot_state not recieve")
        # self.get_logger().info(f'robot1={self.robot_state["0"]["pos"]} \nrobot2={self.robot_state["1"]["pos"]}')

        

        # for j in self.intersection:
        #     for k in 
        #     self.get_logger().warn()

        
        



def main(args=None):
    global all_map_info
    with open("./self_map_test.json",'r') as json_file:
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
    