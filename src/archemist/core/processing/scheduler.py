from bson.objectid import ObjectId
from archemist.core.persistence.objects_getter import RobotsGetter, LotsGetter, StationsGetter, StateGetter
from archemist.core.state.batch import Batch
from archemist.core.state.robot import MobileRobotMode, MobileRobot, FixedRobot
from archemist.core.util.enums import RobotTaskType
from archemist.core.state.robot_op import (RobotOpDescriptor,
                                           RobotTaskOpDescriptor,
                                           RobotMaintenanceOpDescriptor,
                                           RobotWaitOpDescriptor,
                                           RobotNavOpDescriptor)

from typing import List, Type
from abc import ABC, abstractmethod

class RobotScheduler(ABC):
    def __init__(self, *args, **kwargs):
        pass
    
    @abstractmethod
    def schedule(self, robot_ops_queue: List[Type[RobotOpDescriptor]]):
        pass

class PriorityQueueRobotScheduler(RobotScheduler):
    def __init__(self, *args, **kwargs):
        robots = RobotsGetter.get_robots()
        self.robots_schedules = {robot.__class__.__name__: [] for robot in robots}
        super().__init__(*args, **kwargs)

    def schedule(self, robot_ops_queue: List[Type[RobotOpDescriptor]]):
       # clear robot schedules
       self.robots_schedules = {robot: [] for robot in self.robots_schedules}
       
       # add robot ops to the appropriate robot queues
       for op in robot_ops_queue:
           self.robots_schedules[op.target_robot].append(op)
       
       # sort robot queues according to priority
       for robot_name in self.robots_schedules:
            # group ops according to their lot and origin station
            robot_ops_list = sorted(self.robots_schedules[robot_name], key= lambda op: self._get_lot_and_requester_keys(op))
            # sort robot queus        
            robot = RobotsGetter.get_robot(robot_name)
            if isinstance(robot, MobileRobot):
                self.robot_free_slots_num = robot.free_batch_capacity
                robot_ops_with_priority_list = [(op, self._calculate_mobile_robot_priority(op, robot)) for op in robot_ops_list]
                self.robots_schedules[robot_name] = sorted(robot_ops_with_priority_list, key= lambda tup: tup[1], reverse=True)
            elif isinstance(robot, FixedRobot):
                robot_ops_with_priority_list = [(op, self._calculate_fixed_robot_priority(op, robot)) for op in robot_ops_list]
                self.robots_schedules[robot_name] = sorted(robot_ops_with_priority_list, key= lambda tup: tup[1], reverse=True)

            # add ops to the robot queue
            for index, op_priority_tup in enumerate(self.robots_schedules[robot_name]):
                op = op_priority_tup[0]
                priority = op_priority_tup[1]
                if index == 0:
                    if priority < 100:
                        break
                    first_requester = op.requested_by
                
                if priority >= 100 and op.requested_by ==  first_requester:
                    robot.add_op(op)
                    robot_ops_queue.remove(op)
                else:
                    break
                
                    

    
    def _calculate_mobile_robot_priority(self, op: Type[RobotOpDescriptor], robot: Type[MobileRobot]):
        priority = 0
        if isinstance(op, RobotMaintenanceOpDescriptor):
            priority += 200
        else:
            if isinstance(op, RobotTaskOpDescriptor):
                if op.task_type == RobotTaskType.UNLOAD_FROM_ROBOT:
                    if op.related_batch in robot.onboard_batches:
                        priority += 30
                    else:
                        priority -= 100
                elif op.task_type == RobotTaskType.LOAD_TO_ROBOT:
                    if (self.robot_free_slots_num > 0
                        and (op.related_lot in robot.consigned_lots or robot.free_lot_capacity > 0)
                        and self._is_next_station_free(op.related_batch)
                        and robot.operational_mode == MobileRobotMode.OPERATIONAL ):
                        priority += 20
                        self.robot_free_slots_num -= 1
                    else:
                        priority -= 100
                else:
                    priority += 10
            elif isinstance(op, RobotNavOpDescriptor):
                priority += 10
            elif isinstance(op, RobotWaitOpDescriptor):
                priority += 5
            
            if robot.attending_to is None:
                priority += 100
            elif robot.attending_to == op.requested_by:
                priority += 150
        
        return priority
    
    def _calculate_fixed_robot_priority(self, op: Type[RobotOpDescriptor], robot: Type[FixedRobot]):
        priority = 0
        if isinstance(op, RobotMaintenanceOpDescriptor):
            priority += 200
        elif isinstance(op, RobotWaitOpDescriptor):
            priority += 5
        else:
            priority += 10
        
        if robot.attending_to is None:
            priority += 100
        elif robot.attending_to == op.requested_by:
            priority += 150
        
        return priority
    
    def _is_next_station_free(self, batch: Batch) -> bool:
        is_free = False
        parent_lot = LotsGetter.get_containing_lot(batch)
        next_state = parent_lot.recipe.get_next_state_details(success=True)
        if next_state is None:
            output_site = StateGetter.get_output_state()
            if output_site.free_lot_capacity > 0:
                is_free = True    
        else:
            next_station = StationsGetter.get_station(next_state.station_id, next_state.station_type)
            if next_station.free_lot_capacity > 0:
                is_free = True
        
        return is_free
    
    def _get_lot_and_requester_keys(self, op: Type[RobotOpDescriptor]):
        if isinstance(op, RobotTaskOpDescriptor) and op.related_lot:
            lot_key = op.related_lot.object_id
        else:
            lot_key = ObjectId(b"no lot      ")
        requester_key = op.requested_by if op.requested_by else ObjectId(b"no requester")
        return (lot_key, requester_key)
            