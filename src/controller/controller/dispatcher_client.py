#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from interfaces.action import Task
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

class DispatcherClient(Node):
    def __init__(self, enable_cancel: bool, cancel_after_sec: float):
        super().__init__('dispatcher_client')
        self._action_client = ActionClient(self, Task, 'robot_task')
        self.enable_cancel = enable_cancel
        self.cancel_after_sec = cancel_after_sec
        self._goal_handle = None
        self._cancel_timer = None
        self._cancel_requested = False

    def send_goal(self):
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

        # 若啟用中斷，排程在指定秒數後送出取消請求
        if self.enable_cancel and not self._cancel_requested:
            self.get_logger().warn(f'將在 {self.cancel_after_sec:.1f} 秒後嘗試取消目標...')
            self._cancel_timer = self.create_timer(self.cancel_after_sec, self._cancel_goal_once)

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_cb)

    def _cancel_goal_once(self):
        # 只執行一次
        if self._cancel_timer:
            self._cancel_timer.cancel()
            self._cancel_timer = None
        if self._goal_handle and not self._cancel_requested:
            self._cancel_requested = True
            self.get_logger().warn('送出取消請求...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_response)

    def _on_cancel_response(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().warn('取消請求已被伺服器接受')
        else:
            self.get_logger().warn('取消請求被拒絕或目標已完成')

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'任務結束。狀態: {result.task_status}, 訊息: {result.status_message}')
        rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--cancel', action='store_true', help='啟用中斷示範（稍後自動取消）')
    parser.add_argument('--cancel-after', type=float, default=0.1, help='幾秒後嘗試取消')
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = DispatcherClient(enable_cancel=cli_args.cancel, cancel_after_sec=cli_args.cancel_after)
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
