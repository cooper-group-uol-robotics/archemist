from archemist.state.robot import Robot
from archemist.processing.handler import RobotHandler
import time

class GenericRobot_Handler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)

    def run(self):
        print(f'{self._robot}_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print(f'{self._robot}_handler is terminating!!!')

    def execute_job(self):
        self._handled_robot_op = self._robot.assigned_job
        self._handled_robot_op.add_start_timestamp()
        print(f'[{self.__class__.__name__}] executing {str(self._handled_robot_op)}')
        time.sleep(1)
        self._robot.start_job_execution()

    def is_job_execution_complete(self):
        return True