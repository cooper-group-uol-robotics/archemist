from archemist.core.state.robot import RobotTaskType, RobotState, MobileRobotMode
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaNAVTask, KukaLBRMaintenanceTask
from archemist.robots.panda_robot.state import PandaRobotTask
from archemist.robots.yumi_robot.state import YuMiRobotTask
from archemist.core.state.state import State


class RobotScheduler():
    def __init__(self):
        pass

    def schedule(self, state:State):
        pass


class SimpleRobotScheduler(RobotScheduler):
    def __init__(self):
        super().__init__()

    def schedule(self, state: State):
        unassigned_jobs = list()
        while state.robot_ops_queue:
            station_robot_job = state.robot_ops_queue.pop()
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
        
        state.robot_ops_queue.extend(unassigned_jobs)

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

    def schedule(self, state: State):
        unassigned_jobs = list()
        while state.robot_ops_queue:
            robot_job = state.robot_ops_queue.pop()
            job_assigned = False
            if isinstance(robot_job, KukaLBRMaintenanceTask):
                kuka_robots = state.get_robots('KukaLBRIIWA')
                for robot in kuka_robots:
                    if robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            elif isinstance(robot_job, KukaNAVTask):
                kuka_robots = state.get_robots('KukaLBRIIWA')
                for robot in kuka_robots:
                    if robot.operational and robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            elif isinstance(robot_job, KukaLBRTask):
                kuka_robots = state.get_robots('KukaLBRIIWA')
                for robot in kuka_robots:
                    if robot.operational and robot.state == RobotState.IDLE:
                        if robot_job.task_type == RobotTaskType.LOAD_TO_ROBOT: #load to robot if it has capacity and next station is free
                            if not robot.is_onboard_capacity_full():
                                if self._is_next_station_free(robot_job.related_batch_id,state):
                                    robot.assign_op(robot_job)
                                    job_assigned = True
                                    break
                        elif robot_job.task_type == RobotTaskType.UNLOAD_FROM_ROBOT:
                            if robot.is_batch_onboard(robot_job.related_batch_id):
                                if self._is_next_station_free(robot_job.related_batch_id,state):
                                    robot.assign_op(robot_job)
                                    job_assigned = True
                                    break
                        elif robot_job.task_type == RobotTaskType.MANIPULATION:
                            robot.assign_op(robot_job)
                            job_assigned = True
                            break
            elif isinstance(robot_job, YuMiRobotTask):
                yumi_robots = state.get_robots('YuMiRobot')
                for robot in yumi_robots:
                    if robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            elif isinstance(robot_job, PandaRobotTask):
                panda_robots = state.get_robots('PandaFranka')
                for robot in panda_robots:
                    if robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            if not job_assigned:
                unassigned_jobs.append(robot_job)
        
        state.robot_ops_queue.extend(unassigned_jobs)

class LazyRobotScheduler(RobotScheduler):
    def __init__(self, allow_mixed_batches: bool=False):
        super().__init__()
        self._allow_mixed_batches = allow_mixed_batches

    def _is_next_station_free(self, batch_id: int, state: State):
        batch = state.get_batch(batch_id)
        next_station_name, next_station_id = batch.recipe.get_next_station(True)
        next_station_free = False
        if next_station_name != 'end':
            station = state.get_station(next_station_name, next_station_id)
            if station.has_free_batch_capacity():
                next_station_free = True
        else:
            next_station_free = True
        return next_station_free
    
    def _are_batches_from_same_station(self, batch_id: int, other_batch_id: int, state: State) -> bool:
        batch = state.get_batch(batch_id)
        station_type, station_id = batch.recipe.get_current_station()
        other_batch = state.get_batch(other_batch_id)
        other_station_type, other_station_id = other_batch.recipe.get_current_station()
        if station_type == other_station_type and station_id == other_station_id:
            station = state.get_station(station_type, station_id)
            batch_process_id = station.get_batch_process_uuid(batch)
            other_batch_process_id = station.get_batch_process_uuid(other_batch)
            if batch_process_id == other_batch_process_id:
                return True
        return False

    def schedule(self, state: State):
        unassigned_jobs = list()
        while state.robot_ops_queue:
            robot_job = state.robot_ops_queue.pop()
            job_assigned = False
            if isinstance(robot_job, KukaLBRMaintenanceTask) or isinstance(robot_job, KukaNAVTask) or isinstance(robot_job, KukaLBRTask):
                kuka_robots = state.get_robots('KukaLBRIIWA')
                for robot in kuka_robots:
                    if robot.state == RobotState.IDLE:
                        if isinstance(robot_job, KukaLBRMaintenanceTask):
                            robot.assign_op(robot_job)
                            job_assigned = True
                        elif robot.operational_mode != MobileRobotMode.MAINTENANCE:
                            if isinstance(robot_job, KukaNAVTask):
                                if not robot.locked_to_station:
                                    robot.assign_op(robot_job)
                                    job_assigned = True
                            elif isinstance(robot_job, KukaLBRTask):
                                if robot_job.task_type == RobotTaskType.LOAD_TO_ROBOT:
                                    if robot.operational_mode == MobileRobotMode.OPERTIAONAL:
                                        if not robot.is_onboard_capacity_full():
                                            if not self._allow_mixed_batches:
                                                if robot.onboard_batches:
                                                    batch_id = robot.onboard_batches[0]
                                                    if not self._are_batches_from_same_station(batch_id, robot_job.related_batch_id, state):
                                                        continue
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
                        if job_assigned:
                            break
            elif isinstance(robot_job, YuMiRobotTask):
                yumi_robots = state.get_robots('YuMiRobot')
                for robot in yumi_robots:
                    if robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            elif isinstance(robot_job, PandaRobotTask):
                panda_robots = state.get_robots('PandaFranka')
                for robot in panda_robots:
                    if robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            if not job_assigned:
                unassigned_jobs.append(robot_job)
        
        state.robot_ops_queue.extend(unassigned_jobs)
            