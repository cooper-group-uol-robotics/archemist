from archemist.state.robots import PandaFranka
from archemist.state.state import State
from archemist.util.location import Location
from archemist.processing.handler import RobotHandler
import time
class EmuPandaHandler(RobotHandler):
    def __init__(self, robot_id: int):
        super().__init__('PandaFranka', robot_id)
        print(self._robot_name + f" with id:{robot_id} handler is running")
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print('Terminating!!!')
            pass

    def execute_job(self):
        (assigned_job, station) = self._robot.assigned_job
        print('executing robot job')
        time.sleep(3)
        assigned_job.add_timestamp()
        assigned_job.output.has_result = True
        assigned_job.output.success = True
        assigned_job.output.add_timestamp()


if __name__ == '__main__':
    panad_handler = EmuPandaHandler(1)