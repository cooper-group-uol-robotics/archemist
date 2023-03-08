from archemist.core.state.robot import Robot
from archemist.core.processing.handler import RobotHandler
from threading import Thread
import time

class GenericRobotHandler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        self._thread = None

    def run(self):
        print(f'{self._robot}_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print(f'{self._robot}_handler is terminating!!!')

    def execute_op(self):
        handled_robot_op = self._robot.get_assigned_op()
        print(f'[{self.__class__.__name__}] executing {str(handled_robot_op)}')
        self._thread = Thread(target=self._mock_execution)
        self._thread.start()

    def is_op_execution_complete(self):
        if self._thread is not None and self._thread.is_alive():
            return False
        else:
            return True

    def get_op_result(self) -> bool:
        return True

    def _mock_execution(self):
        time.sleep(1)