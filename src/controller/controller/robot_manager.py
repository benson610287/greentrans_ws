import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from interfaces.action import Task
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')

        self.group = ReentrantCallbackGroup()

        self._action_client = ActionClient(self, Task, 'robot_task')

        self.loop_timer = self.create_timer(0.5, self.loop, callback_group=self.group)

    def loop(self):
        self.get_logger().info("main...")
        self.send_goal()

    def send_goal(self):
        self.get_logger().info("send...")
        goal = Task.Goal()
        goal.task_type = Task.Goal.NAVIGATION
        goal.task_path = PoseArray()
        goal.task_path.header.frame_id = "map"

        # 建一條簡單路徑
        p1 = Pose()
        p1.position = Point(x=1.0, y=2.0, z=0.0)
        p1.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        p2 = Pose()
        p2.position = Point(x=3.0, y=4.0, z=0.0)
        p2.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        goal.task_path.poses = [p1, p2]

        self.get_logger().info("等待機器人 action server ...")
        self._action_client.wait_for_server()

        self.get_logger().info("發送任務：NAVIGATION")
        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'[Feedback] {fb.result_message} (成功: {fb.task_result})')

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('任務被拒絕')
            return

        self.get_logger().info('任務已被接受，等待結果...')
        self._goal_handle = goal_handle

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'任務結束。狀態: {result.task_status}, 訊息: {result.status_message}')


def main(args=None):
    rclpy.init(args=args)

    node = RobotManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
