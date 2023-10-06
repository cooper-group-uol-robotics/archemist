from time import sleep
from archemist.core.state.robot import Robot
from archemist.core.state.station import Station, OpState
from archemist.core.persistence.object_factory import StationFactory, RobotFactory, ProcessFactory
from archemist.core.util.enums import StationState, RobotState, OpResult, ProcessStatus
from typing import Tuple,Dict
from abc import ABC, abstractmethod

class OpHandler(ABC):
    @abstractmethod
    def initialise(self) -> bool:
        pass

    @abstractmethod
    def execute_op(self):
        pass
    
    @abstractmethod
    def is_op_execution_complete(self) -> bool:
        pass

    @abstractmethod
    def get_op_result(self) -> Tuple[OpResult,Dict]:
        pass

    @abstractmethod
    def shut_down(self):
        pass

class StationOpHandler(OpHandler):
    def __init__(self, station: Station):
        self._station = station

    def handle(self):
        if self._station.assigned_op_state == OpState.INVALID:
            self._station.update_assigned_op()
        elif self._station.assigned_op_state == OpState.ASSIGNED:
            self._station.set_assigned_op_to_execute()
            self.execute_op()
        elif self._station.assigned_op_state == OpState.EXECUTING:
            if self.is_op_execution_complete():
                op_successful, op_results = self.get_op_result()
                self._station.complete_assigned_op(op_successful, **op_results)
        elif self._station.assigned_op_state == OpState.TO_BE_REPEATED:
            self._station.set_assigned_op_to_execute()
            self.execute_op()
        elif self._station.assigned_op_state == OpState.TO_BE_SKIPPED:
            self._station.complete_assigned_op(OpResult.SKIPPED)
    def run(self):
        try:
            while True:
                self.handle()
                sleep(1)
        except Exception as err:
            print(err)
        finally:
            self.shut_down()

class RobotOpHandler(OpHandler):
    def __init__(self, robot: Robot):
        self._robot = robot

    def handle(self):
        if self._robot.assigned_op_state == OpState.ASSIGNED:
            self._robot.set_assigned_op_to_execute()
            self.execute_op()
        elif self._robot.assigned_op_state == OpState.EXECUTING:
            if self.is_op_execution_complete():
                op_successful = self.get_op_result()
                self._robot.complete_assigned_op(op_successful)
        elif self._robot.assigned_op_state == OpState.TO_BE_REPEATED:
            self._robot.set_assigned_op_to_execute()
            self.execute_op()
        elif self._robot.assigned_op_state == OpState.TO_BE_SKIPPED:
            self._robot.complete_assigned_op(OpResult.SKIPPED)

class StationProcessHandler:
    def __init__(self, station: Station):
        self._station = station
        self._lots_seen = []
        for _, proc in self._station.running_procs_slots.items():
            if proc is not None:
                self._lots_seen.append(proc.lot)

    def _handle_assigned_lots(self):
        assigned_lots = self._station.assigned_lots
        newly_added_lots = [lot for lot in assigned_lots if lot not in self._lots_seen]
        for lot in newly_added_lots:
            state_details = lot.recipe.current_state_details
            proc_dict = state_details.station_process
            station_module_path = self._station.module_path
            proc = ProcessFactory.create_from_dict(dict(proc_dict), lot, station_module_path)
            self._station.add_process(proc)
            self._lots_seen.append(lot)

    def _handle_processes(self):
        for slot, proc in self._station.running_procs_slots.items():
            if proc is None:
                if self._station.queued_procs:
                    new_proc = self._station.queued_procs.pop(left=True)
                    new_proc.processing_slot = slot
                    self._station.running_procs_slots[slot] = new_proc
            else:
                if proc.m_state != "final_state":
                    proc.tick()
                else:
                    self._station.running_procs_slots[slot] = None
                    lot = proc.lot
                    if lot in self._station.assigned_lots:
                        self._station.finish_processing_lot(lot)
                    self._lots_seen.remove(lot)
                    self._station.procs_history.append(proc)

    def _update_seen_lots(self):
        for proc in self._station.queued_procs:
            lot = proc.lot
            if lot not in self._lots_seen:
                self._lots_seen.append(lot)

    def _handle_proc_requests(self):
        for slot, proc in self._station.running_procs_slots.items():
            if proc:
                if proc.status == ProcessStatus.REQUESTING_ROBOT_OPS:
                    for robot_op in proc.req_robot_ops:
                        self._station.requested_robot_ops.append(robot_op)
                    proc.switch_to_waiting()
                
                if proc.status == ProcessStatus.REQUESTING_STATION_OPS:
                    for station_op in proc.req_station_ops:
                        self._station.add_station_op(station_op)
                    proc.switch_to_waiting()
                
                if proc.status == ProcessStatus.REQUESTING_STATION_PROCS:
                    for req_station_proc in proc.req_station_procs:
                        self._station.request_external_process(req_station_proc)
                    proc.switch_to_waiting()

    def handle(self):
        self._update_seen_lots()
        self._handle_assigned_lots()
        self._handle_processes()
        self._handle_proc_requests()

class StationHandler:
    def __init__(self, station: Station, use_sim: bool):
        self._station = station
        self._station_op_handler: StationOpHandler = StationFactory.create_op_handler(station, use_sim)
        self._station_process_handler = StationProcessHandler(station)

    def initialise(self):
        init_successful = self._station_op_handler.initialise()
        if init_successful:
            print(f"{self._station} handler was successfully initialised")
            self._station.state = StationState.ACTIVE
        else:
            print(f"{self._station} handler was unsuccessfully initialised")
            self._station.state = StationState.ERROR
        
    def tick(self):
        if self._station.state == StationState.ACTIVE:
            self._station_op_handler.handle()
            self._station_process_handler.handle()

    def run(self):
        try:
            while True:
                self.tick()
                sleep(1)
        except Exception as err:
            print(err)
        finally:
            self._station_op_handler.shut_down()
            
class RobotHandler:
    def __init__(self, robot: Robot, use_sim: bool):
        self._robot = robot
        self._robot_op_handler: RobotOpHandler = RobotFactory.create_op_handler(robot, use_sim)

    def initialise(self):
        init_successful = self._robot_op_handler.initialise()
        if init_successful:
            print(f"{self._robot} handler was successfully initialised")
            self._robot.state = RobotState.ACTIVE
        else:
            print(f"{self._robot} handler was unsuccessfully initialised")
            self._robot.state = RobotState.ERROR

    def tick(self):
        if self._robot.state == RobotState.ACTIVE:
            self._robot_op_handler.handle()

    def run(self):
        try:
            while True:
                self.tick()
                sleep(1)
        except Exception as err:
            print(err)
        finally:
            self._robot_op_handler.shut_down()

class SimRobotOpHandler(RobotOpHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)

    def initialise(self) -> bool:
        print(f"[{self.__class__.__name__}] is initialised")
        return True

    def execute_op(self):
        robot_op = self._robot.assigned_op
        print(f'[{self.__class__.__name__}] executing {str(robot_op)}')

    def is_op_execution_complete(self):
            return True

    def get_op_result(self) -> OpResult:
        return OpResult.SUCCEEDED
    
    def shut_down(self):
         print(f"[{self.__class__.__name__}] is shutting down")

class SimStationOpHandler(StationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def initialise(self) -> bool:
        print(f"[{self.__class__.__name__}] is initialised")
        return True

    def execute_op(self):
        station_op = self._station.assigned_op
        print(f'[{self.__class__.__name__}] executing {station_op}')

    def is_op_execution_complete(self):
            return True

    def get_op_result(self) -> Tuple[OpResult, Dict]:
        return OpResult.SUCCEEDED, {}
    
    def shut_down(self):
         print(f"[{self.__class__.__name__}] is shutting down")