from archemist.core.state.robot import RobotTaskType, RobotState
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaNAVTask, KukaLBRMaintenanceTask
from archemist.robots.panda_robot.state import PandaFranka
from archemist.core.state.state import State


class RobotScheduler():
    def __init__(self):
        pass

    def schedule(self, job_station_queue: list, state:State):
        pass


class SimpleRobotScheduler(RobotScheduler):
    def __init__(self):
        super().__init__()

    def schedule(self, job_station_queue: list, state: State):
        unassigned_jobs = list()
        while job_station_queue:
            station_robot_job = job_station_queue.pop()
            job_assigned = False
            robot_job = station_robot_job.robot_op
            if isinstance(robot_job, KukaLBRMaintenanceTask):
                robot = state.get_robot('KukaLBRIIWA',1)
                if robot.state == RobotState.IDLE:
                    robot.assign_op(station_robot_job)
                    job_assigned = True
            if isinstance(robot_job, KukaNAVTask) or isinstance(robot_job, KukaLBRTask):
                robot = state.get_robot('KukaLBRIIWA',1) # this can be replaced by querying a list with robots that are KUKA
                if robot.operational and robot.state == RobotState.IDLE:
                    robot.assign_op(station_robot_job)
                    job_assigned = True
            # elif isinstance(robot_job, MoveSampleOp):
            #     for robot in state.robots:
            #         if robot.state == RobotState.IDLE and isinstance(robot,PandaFranka):
            #             robot.assign_op(station_robot_job)
            #             job_assigned = True
            #             if robot.location.get_map_coordinates() == robot_job.start_location.get_map_coordinates():
            #                 if robot_job.start_location.frame_name in robot.saved_frames and robot_job.target_location.frame_name in robot.saved_frames:
            #                     robot.assign_op(station_robot_job)
            #                     job_assigned = True
            if not job_assigned:
                unassigned_jobs.append(station_robot_job)
        
        job_station_queue.extend(unassigned_jobs)

class MultiBatchRobotScheduler(RobotScheduler):
    def __init__(self):
        super().__init__()

    def _is_next_station_free(self, batch_id: int, state: State):
        batch = state.get_batch(batch_id)
        next_station_name, next_station_id = batch.recipe.get_next_station(True)
        next_station_free = False
        if next_station_name != 'end':
            station = state.get_station(next_station_name, next_station_id)
            if len(station.assigned_batches) == 0:
                next_station_free = True
        else:
            next_station_free = True
        return next_station_free

    def schedule(self, job_station_queue: list, state: State):
        unassigned_jobs = list()
        while job_station_queue:
            robot_job = job_station_queue.pop()
            job_assigned = False
            if isinstance(robot_job, KukaLBRMaintenanceTask):
                robot = state.get_robot('KukaLBRIIWA',1)
                if robot.state == RobotState.IDLE:
                    robot.assign_op(robot_job)
                    job_assigned = True
            elif isinstance(robot_job, KukaNAVTask):
                robot = state.get_robot('KukaLBRIIWA',1) # this can be replaced by querying a list with robots that are KUKA
                if robot.operational and robot.state == RobotState.IDLE:
                    robot.assign_op(robot_job)
                    job_assigned = True
            elif isinstance(robot_job, KukaLBRTask):
                robot = state.get_robot('KukaLBRIIWA',1) # this can be replaced by querying a list with robots that are KUKA
                if robot.operational and robot.state == RobotState.IDLE:
                    if robot_job.task_type == RobotTaskType.LOAD_TO_ROBOT: #load to robot if it has capacity and next station is free
                        if not robot.is_onboard_capacity_full():
                            if self._is_next_station_free(robot_job.related_batch_id,state):
                                robot.assign_op(robot_job)
                                job_assigned = True
                    elif robot_job.task_type == RobotTaskType.UNLOAD_FROM_ROBOT:
                        if robot.is_batch_onboard(robot_job.related_batch_id):
                            if self._is_next_station_free(robot_job.related_batch_id,state):
                                robot.assign_op(robot_job)
                                job_assigned = True
                    elif robot_job.task_type == RobotTaskType.MANIPULATION:
                        robot.assign_op(robot_job)
                        job_assigned = True
            if not job_assigned:
                unassigned_jobs.append(robot_job)
        
        job_station_queue.extend(unassigned_jobs)
            