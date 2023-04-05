from archemist.core.state.robot import RobotTaskType, RobotState
from archemist.robots.kmriiwa_robot.state import (
    KukaLBRTask,
    KukaNAVTask,
    KukaLBRMaintenanceTask,
)
from archemist.robots.panda_robot.state import PandaRobotTask
from archemist.robots.yumi_robot.state import YuMiRobotTask
from archemist.core.state.state import State


class RobotScheduler:
    def __init__(self):
        pass

    def schedule(self, state: State):
        pass


class SimpleRobotScheduler(RobotScheduler):
    def __init__(self):
        super().__init__()

    def schedule(self, state: State):
        unassigned_jobs = []
        while state.robot_ops_queue:
            station_robot_job = state.robot_ops_queue.pop()
            job_assigned = False
            robot_job = station_robot_job.robot_op
            if isinstance(robot_job, KukaLBRMaintenanceTask):
                robot = state.get_robot("KukaLBRIIWA", 1)
                if robot.state == RobotState.IDLE:
                    robot.assign_op(station_robot_job)
                    job_assigned = True
            if isinstance(robot_job, KukaNAVTask) or isinstance(robot_job, KukaLBRTask):
                robot = state.get_robot(
                    "KukaLBRIIWA", 1
                )  # this can be replaced by querying a list with robots that are KUKA
                if robot.operational and robot.state == RobotState.IDLE:
                    robot.assign_op(station_robot_job)
                    job_assigned = True
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
        if next_station_name != "end":
            station = state.get_station(next_station_name, next_station_id)
            if len(station.assigned_batches) == 0:
                next_station_free = True
        else:
            next_station_free = True
        return next_station_free

    def schedule(self, state: State):
        unassigned_jobs = []
        while state.robot_ops_queue:
            robot_job = state.robot_ops_queue.pop()
            job_assigned = False
            if isinstance(robot_job, KukaLBRMaintenanceTask):
                kuka_robots = state.get_robots("KukaLBRIIWA")
                for robot in kuka_robots:
                    if robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            elif isinstance(robot_job, KukaNAVTask):
                kuka_robots = state.get_robots("KukaLBRIIWA")
                for robot in kuka_robots:
                    if robot.operational and robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            elif isinstance(robot_job, KukaLBRTask):
                kuka_robots = state.get_robots("KukaLBRIIWA")
                for robot in kuka_robots:
                    if robot.operational and robot.state == RobotState.IDLE:
                        if (
                            robot_job.task_type == RobotTaskType.LOAD_TO_ROBOT
                        ):  # load to robot if it has capacity and next station is free
                            if not robot.is_onboard_capacity_full():
                                if self._is_next_station_free(
                                    robot_job.related_batch_id, state
                                ):
                                    robot.assign_op(robot_job)
                                    job_assigned = True
                                    break
                        elif robot_job.task_type == RobotTaskType.UNLOAD_FROM_ROBOT:
                            if robot.is_batch_onboard(robot_job.related_batch_id):
                                if self._is_next_station_free(
                                    robot_job.related_batch_id, state
                                ):
                                    robot.assign_op(robot_job)
                                    job_assigned = True
                                    break
                        elif robot_job.task_type == RobotTaskType.MANIPULATION:
                            robot.assign_op(robot_job)
                            job_assigned = True
                            break
            elif isinstance(robot_job, YuMiRobotTask):
                yumi_robots = state.get_robots("YuMiRobot")
                for robot in yumi_robots:
                    if robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            elif isinstance(robot_job, PandaRobotTask):
                panda_robots = state.get_robots("PandaFranka")
                for robot in panda_robots:
                    if robot.state == RobotState.IDLE:
                        robot.assign_op(robot_job)
                        job_assigned = True
                        break
            if not job_assigned:
                unassigned_jobs.append(robot_job)

        state.robot_ops_queue.extend(unassigned_jobs)
