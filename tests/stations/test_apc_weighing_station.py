import unittest

from mongoengine import connect
from archemist.stations.apc_weighing_station.state import (
    ApcWeighingStation, 
    ApcWeighingVOpenDoorOp, 
    ApcWeighingVCloseDoorOp, 
    ApcBalanceOpenDoorOp, 
    ApcBalanceCloseDoorOp,
    ApcTareOp,
    ApcWeighingOp,
    ApcWeighResult
)
from archemist.stations.apc_weighing_station.process import ApcWeighingStationProcess
from archemist.stations.apc_weighing_station.handler import SimApcWeighingStationHandler
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import StationState, ProcessStatus, OpOutcome
from archemist.core.state.robot_op import RobotTaskOp, RobotNavOp, RobotWaitOp
from .testing_utils import test_req_robot_ops, test_req_station_op


class WeighingStationTest(unittest.TestCase):

    def setUp(self -> None):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'WeighingStation',
            'id': 35,
            'location': {'coordinates': [1,7], 'descriptor': "WeighingStation"},
            'total_lot_capacity': 1,
            'handler': 'Op',
            'properties': None,
            'materials': None
        }

        self.station = ApcWeighingStation.from_dict(self.station_doc)

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        
        station = ApcWeighingStation.from_dict(self.station_doc)
        self.assertIsNotNone(station)

        self.assertFalse(station.balance_doors_open)
        self.assertFalse(station.vertical_doors_open)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        station.add_lot(lot)

        # test WeighingVOpenDoorOp
        t_op = ApcWeighingVOpenDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(station.vertical_doors_open)

        # test WeighingVCloseDoorOp
        t_op = ApcWeighingVCloseDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test BalanceOpenDoorOp
        t_op = ApcBalanceOpenDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test BalanceCloseDoorOp
        t_op = ApcBalanceCloseDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test TareOp
        t_op = ApcTareOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test WeighingOp
        t_op = ApcWeighingOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test WeighingOpResult
        t_result_op = ApcWeighResult.from_args(
            origin_op=t_op.object_id,
            reading_value=42.1)
        self.assertIsNotNone(t_result_op.object_id)
        self.assertEqual(t_result_op.reading_value, 42.1)
        self.assertEqual(t_result_op.unit, "g")

    def test_process(self):

        # construct batches
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        
        # add lot to station
        self.station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "weigh_op",
                    "op": "WeighOp",
                    "parameters": {}
                }
            ]
        process = ApcWeighingStationProcess.from_args()
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.m_state, 'prep_state')
        self.assertEqual(process.status, ProcessStatus.RUNNING)

        # navigate_to_weighing_station
        process.tick()
        self.assertEqual(process.m_state, 'navigate_to_weighing_station')
        test_req_robot_ops(self, process, [RobotNavOp, RobotWaitOp])

        # open_fh_door_vertical
        process.tick()
        self.assertEqual(process.m_state, 'open_fh_door_vertical')
        test_req_station_op(self, process, ApcWeighingVOpenDoorOp)

        # tare
        process.tick()
        self.assertEqual(process.m_state, 'tare')
        test_req_station_op(self, process, ApcTareOp)

        # open_balance_door
        process.tick()
        self.assertEqual(process.m_state, 'open_balance_door')
        test_req_station_op(self, process, ApcBalanceOpenDoorOp)

        # load_funnel
        process.tick()
        self.assertEqual(process.m_state, 'load_funnel')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # close_balance_door
        process.tick()
        self.assertEqual(process.m_state, 'close_balance_door')
        test_req_station_op(self, process, ApcBalanceCloseDoorOp)

        # weigh
        process.tick()
        self.assertEqual(process.m_state, 'weigh')
        test_req_station_op(self, process, ApcWeighingOp)

        # unload_funnel
        process.tick()
        self.assertEqual(process.m_state, 'unload_funnel')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # close_fh_door_vertical
        process.tick()
        self.assertEqual(process.m_state, 'close_fh_door_vertical')
        test_req_station_op(self, process, ApcWeighingVCloseDoorOp)

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_sim_handler(self): 
        batch_1 = Batch.from_args(3)
        lot = Lot.from_args([batch_1])

        # add batches to station
        self.station.add_lot(lot)

        # construct handler
        handler = SimApcWeighingStationHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct weigh result op
        current_op = ApcWeighingOp.from_args(target_sample=batch_1.samples[0])
        t_op = ApcWeighResult.from_args(origin_op=current_op.object_id, reading_value=6.0)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsInstance(op_results, ApcWeighResult)


if __name__ == '__main__':
    unittest.main()
