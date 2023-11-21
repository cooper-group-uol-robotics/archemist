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
from testing_utils import test_req_robot_ops, test_req_station_op


class APCWeighingStationTest(unittest.TestCase):

    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(
            db=self._db_name, alias='archemist_state', host='mongodb://localhost:27017',
            uuidrepresentation='standard'
        )

        self.station_doc = {
            'type': 'ApcWeighingStation',
            'id': 35,
            'location': {'coordinates': [1,7], 'descriptor': "ApcWeighingStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': None,
            'materials': None
        }

        self.station = ApcWeighingStation.from_dict(self.station_doc)

    def tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        
        # test station is constructed properly
        self.assertIsNotNone(self.station)

        self.assertFalse(self.station.balance_doors_open)
        self.assertFalse(self.station.vertical_doors_open)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        self.station.add_lot(lot)

        # test WeighingVOpenDoorOp
        t_op = ApcWeighingVOpenDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(self.station.vertical_doors_open)

        # test WeighingVCloseDoorOp
        t_op = ApcWeighingVCloseDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(self.station.vertical_doors_open)

        # test BalanceOpenDoorOp
        t_op = ApcBalanceOpenDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(self.station.balance_doors_open)

        # test BalanceCloseDoorOp
        t_op = ApcBalanceCloseDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(self.station.balance_doors_open)

        # test TareOp
        t_op = ApcTareOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test WeighingOp
        t_op = ApcWeighingOp.from_args(target_sample=batch.samples[0])
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
        process = ApcWeighingStationProcess.from_args(lot=lot)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.m_state, 'prep_state')
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertFalse(process.data['is_weighing_complete'])

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
        test_req_robot_ops(self, process, [RobotTaskOp, RobotWaitOp])

        # close_balance_door
        process.tick()
        self.assertEqual(process.m_state, 'close_balance_door')
        test_req_station_op(self, process, ApcBalanceCloseDoorOp)

        # weigh
        process.tick()
        self.assertEqual(process.m_state, 'weigh')
        test_req_station_op(self, process, ApcWeighingOp)
        self.assertTrue(process.data['is_weighing_complete'])

        # open_balance_door
        process.tick()
        self.assertEqual(process.m_state, 'open_balance_door')
        test_req_station_op(self, process, ApcBalanceOpenDoorOp)

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
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])

        # add batches to station
        self.station.add_lot(lot)

        # construct handler
        handler = SimApcWeighingStationHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct weigh result op
        t_op = ApcWeighingOp.from_args(target_sample=batch.samples[0])
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsInstance(op_results[0], ApcWeighResult)
        self.assertEqual(op_results[0].reading_value, 42)


if __name__ == '__main__':
    unittest.main()
