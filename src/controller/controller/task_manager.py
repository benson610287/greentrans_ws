import json
from threading import Lock


"""============================================================
# Task Manager class
#   add_task_from_json(self, task_json: str)
#   get_task_quantity(self) -> int
#   get_all_tasks(self) -> list
#   get_one_task(self, id: int) -> dict
#   remove_one_task(self, id: int) -> bool
#   remove_all_tasks(self)
============================================================"""
class TaskManager:
    def __init__(self, logger):
        # logger 由 MainController 傳進來，這樣這個 class 也可以用 ROS 的 logger
        self._logger = logger
        self.task_queue = []  # 任務佇列
        self._lock = Lock()


    # ==========================================================
    # add task from json
    # ==========================================================
    def add_task_from_json(self, task_json: str):
        # input:
        #   task_json (str)     - JSON string of the task
        # output: 
        #   success (bool)      - True if task added successfully, False otherwise
        #   message (str)       - message indicating success or failure
        #   task_dict (dict)    - parsed task dictionary if successful, None otherwise
        try:
            task_dict = json.loads(task_json)
            self._logger.info(f'task_dict type: {type(task_dict)}')
        except json.JSONDecodeError as e:
            self._logger.error(f'JSON decode error: {e}')
            return False, f"Invalid JSON: {e}", None

        with self._lock:
            self.task_queue.append(task_dict)
            self._logger.info(f'{type(self.task_queue)}')
        self._logger.info(f'Task added to queue. Current queue length: {len(self.task_queue)}')
        self._logger.info(f'Current task queue: {self.task_queue}')
        return True, "Request received and parsed.", task_dict


    # ==========================================================
    # Get the members of task queue
    # ========================================================== 
    def get_task_quantity(self) -> int:
        # input: None
        # output: int   - number of tasks in the queue
        with self._lock:
            return len(self.task_queue)


    # ==========================================================
    # Get all tasks
    # ==========================================================
    def get_all_tasks(self) -> list:
        # input: None
        # output: list  - list of all tasks in the queue
        with self._lock:
            return self.task_queue.copy()
    

    # ==========================================================
    # Get the oldest task
    # ==========================================================
    def get_one_task(self, id: int) -> dict:
        # input:
        #   id (int)    - task id
        # output:
        #   dict        - task dictionary if found, None otherwise
        with self._lock:
            for task in self.task_queue:
                if id in task:
                    return task
        return None
    

    # ==========================================================
    # Remove task with a specific number
    # ==========================================================
    def remove_one_task(self, id: str) -> bool:
        # input:
        #   id (str)    - task id
        # output:
        #   bool        - True if task removed successfully, False otherwise
        with self._lock:
            for task in self.task_queue:
                if id in task:
                    self.task_queue.remove(task)
                    self._logger.info(f'Task {id} removed from queue.')
                    return True
        self._logger.warning(f'Task {id} not found in queue.')
        return False


    # ==========================================================
    # Remove all tasks
    # ==========================================================
    def remove_all_tasks(self):
        # input: None
        # output: None
        with self._lock:
            self.task_queue.clear()
        self._logger.info('All tasks removed from queue.')




if __name__ == "__main__":
    import logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("task_manager_test")

    task_manager = TaskManager(logger)


    """============================================================
    # Test add_task_from_json
    # add one task
    ============================================================"""
    print("=== Test1: Add Task from JSON ===")
    # create a sample task JSON
    sample_task_json = {
        "3369577": {
            "type": "delivery",
            "robot_id": None,
            "pick": 1,
            "place": 2,
        }
    }

    # convert to JSON string
    task_json_str = json.dumps(sample_task_json)
    # add task
    success, message, task_dict = task_manager.add_task_from_json(task_json_str)

    # test outputs
    task_queue = task_manager.get_all_tasks()
    print(f'Task Queue: {task_queue}')


    """============================================================
    # Test add another task
    ============================================================"""
    print("=== Test2: Add Another Task from JSON ===")
    # create another sample task JSON
    another_task_json = {
        "4720263": {
            "type": "delivery",
            "robot_id": None,
            "pick": 3,
            "place": 4,
        }
    }

    # convert to JSON string
    another_task_json_str = json.dumps(another_task_json)
    # add another task
    success, message, task_dict = task_manager.add_task_from_json(another_task_json_str)

    # test outputs again
    task_queue = task_manager.get_all_tasks()
    print(f'Task Queue: {task_queue}')


    """============================================================
    # Test get_task_quantity
    ============================================================"""
    print("=== Test3: Get Task Quantity ===")
    quantity = task_manager.get_task_quantity()
    print(f'Task Quantity: {quantity}')

    
    """============================================================
    # Test get_one_task
    ============================================================"""
    print("=== Test4: Get One Task ===")
    task = task_manager.get_one_task("3369577")
    print(f'Get Task 3369577: {task}')

    """============================================================
    # Test remove_one_task
    ============================================================"""
    print("=== Test5: Remove One Task ===")
    removed = task_manager.remove_one_task("3369577")
    print(f'Removed Task 3369577: {removed}')
    task_queue = task_manager.get_all_tasks()
    print(f'Task Queue after removal: {task_queue}')

    """============================================================
    # Test remove_all_tasks
    ============================================================"""
    print("=== Test6: Remove All Tasks ===")
    task_manager.remove_all_tasks()
    task_queue = task_manager.get_all_tasks()
    print(f'Task Queue after removing all tasks: {task_queue}')