from typing import Type, List
from unittest import TestCase
from archemist.core.state.station_process import StationProcess
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.util.enums import OpOutcome, ProcessStatus
from bson.objectid import ObjectId


def test_req_robot_ops(test_case: TestCase,
                       process: Type[StationProcess],
                       expected_op_types: List[type],
                       ):
    test_case.assertEqual(process.status, ProcessStatus.REQUESTING_ROBOT_OPS)
    process.switch_to_waiting()
    test_case.assertEqual(len(process.req_robot_ops), len(expected_op_types))
    for index, robot_op in enumerate(process.req_robot_ops):
        test_case.assertIsInstance(robot_op, expected_op_types[index])
        robot_op.complete_op(ObjectId(), OpOutcome.SUCCEEDED)


def test_req_station_op(test_case: TestCase,
                        process: Type[StationProcess],
                        expected_op: type
                        ):
    test_case.assertEqual(process.status, ProcessStatus.REQUESTING_STATION_OPS)
    process.switch_to_waiting()
    station_op = process.req_station_ops[0]
    test_case.assertIsInstance(station_op, expected_op)
    dummy_results = [StationOpResult.from_args(station_op.object_id)]
    station_op.complete_op(OpOutcome.SUCCEEDED, dummy_results)


def test_req_station_proc(test_case: TestCase,
                          process: Type[StationProcess],
                          expected_proc: type
                          ):
    test_case.assertEqual(process.status, ProcessStatus.REQUESTING_STATION_PROCS)
    process.switch_to_waiting()
    station_proc = process.req_station_procs[0]
    test_case.assertIsInstance(station_proc, expected_proc)
    station_proc._model_proxy.status = ProcessStatus.FINISHED
