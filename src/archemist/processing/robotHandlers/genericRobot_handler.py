from archemist.state.robot import Robot, MoveSampleOp
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
        station_robot_job = self._robot.assigned_job
        station_robot_job.robot_op.add_timestamp()
        print(f'executing {station_robot_job.robot_op}')
        time.sleep(1)
        station_robot_job.robot_op.output.has_result = True
        station_robot_job.robot_op.output.success = True
        station_robot_job.robot_op.output.add_timestamp()
        station_robot_job.robot_op.output.executing_robot = str(self._robot)
        return station_robot_job