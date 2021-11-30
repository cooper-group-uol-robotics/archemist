from archemist.state.robot import Robot, RobotState, PickBatchToDeckOp, PlaceBatchFromDeckOp, PickAndPlaceBatchOp, MoveSampleOp
from archemist.state.robots.kukaLBRIIWA import KukaLBRIIWA
from archemist.state.robots.pandaFranka import PandaFranka
from archemist.state.state import State


class RobotScheduler():
    def __init__(self):
        pass

    def schedule(self, job_station_queue: list, state:State):
        pass

    def _commit_robot_job(self, state: State, robot: Robot, job_station_tup: tuple):
        robot.assign_job(job_station_tup)
        state.modifyObjectDB(robot)


class SimpleRobotScheduler(RobotScheduler):
    def __init__(self):
        super().__init__()

    def schedule(self, job_station_queue: list, state: State):
        unassigned_jobs = list()
        while job_station_queue:
            (robot_job, station) = job_station_queue.pop()
            job_assigned = False
            if isinstance(robot_job, PickBatchToDeckOp) or isinstance(robot_job, PlaceBatchFromDeckOp) or isinstance(robot_job, PickAndPlaceBatchOp):
                robot = self._state.getRobot('KukaLBRIIWA',1)
                if robot.state == RobotState.IDLE:
                    job_station_tup = tuple((robot_job, station))
                    self._commit_robot_job(state, robot, job_station_tup)
                    job_assigned = True
            elif isinstance(robot_job, MoveSampleOp):
                for robot in state.robots:
                    if robot.state == RobotState.IDLE:
                        if robot.location.get_map_coordinates() == robot_job.start_pos.get_map_coordinates():
                            if robot_job.start_pos.frame_name in robot.saved_frames and robot_job.end_pos.frame_name in robot.saved_frames:
                                job_station_tup = tuple((robot_job, station))
                                self._commit_robot_job(state, robot, job_station_tup)
                                job_assigned = True
            if not job_assigned:
                unassigned_jobs.append(tuple((robot_job, station)))
        job_station_queue.extend(unassigned_jobs)
            