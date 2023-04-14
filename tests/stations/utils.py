from typing import Union, Dict, Any
from unittest import TestCase
from archemist.core.state.station import Station
from archemist.core.state.station_process import StationProcess
from archemist.core.state.robot_op import RobotOpDescriptor, RobotTaskOpDescriptor
from archemist.core.state.station_op import StationOpDescriptor

class ProcessTestingMixin(object):

    def assert_process_transition(self: TestCase, process: StationProcess, new_state: str):
        process.process_state_transitions()
        self.assertEqual(process.data.status['state'], new_state)

    def assert_robot_op(self: TestCase,
                                    robot_op: Union[RobotOpDescriptor,RobotTaskOpDescriptor],
                                    expected_type: str, expecte_fields: Dict[str,Any]):
        self.assertEqual(robot_op.__class__.__name__, expected_type)
        for field_name, field_value in expecte_fields.items():
            op_field_value = getattr(robot_op, field_name)
            self.assertEqual(op_field_value, field_value)

    def complete_robot_op(self, station: Station,
                        robot_op: Union[RobotOpDescriptor,RobotTaskOpDescriptor]):
        robot_op.add_start_timestamp()
        robot_op.complete_op("test-bot", True)
        station.complete_robot_op_request(robot_op)

    def assert_station_op(self: TestCase, station_op: StationOpDescriptor,
                                    expected_type: str, 
                                    expecte_fields: Dict[str,Any]={}):
        self.assertEqual(station_op.__class__.__name__, expected_type)
        for field_name, field_value in expecte_fields.items():
            op_field_value = getattr(station_op, field_name)
            self.assertEqual(op_field_value, field_value)

    def complete_station_op(self, station: Station, **kwargs):
        station.update_assigned_op()
        station.complete_assigned_station_op(True, **kwargs)

