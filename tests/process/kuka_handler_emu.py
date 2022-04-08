from archemist.state.robot import Robot
from archemist.state.robots import KukaLBRIIWA
from archemist.state.state import State
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.util.location import Location
from archemist.processing.handler import RobotHandler
import time

class EmuKukaLBRIIWA_Handler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)

    def execute_job(self):
        station_robot_job = self._robot.assigned_job
        print(f'executing robot job {station_robot_job.robot_op}')
        time.sleep(3)
        station_robot_job.robot_op.add_timestamp()
        station_robot_job.robot_op.output.has_result = True
        station_robot_job.robot_op.output.success = True
        station_robot_job.robot_op.output.add_timestamp()
        station_robot_job.robot_op.output.executing_robot = str(self._robot)

        return station_robot_job

    def run(self):
        print(f'{self._robot}_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print(f'{self._robot}_handler is terminating!!!')


if __name__ == '__main__':
    p_manager = PersistenceManager('test')
    state = p_manager.construct_state_from_db()
    robot = state.get_robot('KukaLBRIIWA', 1)
    kuka_handler = EmuKukaLBRIIWA_Handler(robot)