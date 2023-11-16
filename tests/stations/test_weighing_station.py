import unittest

from mongoengine import connect
from archemist.stations.weighing_station.state import (
    WeighingStation, 
    WeighingVOpenDoorOp, 
    WeighingVCloseDoorOp, 
    BalanceOpenDoorOp, 
    BalanceCloseDoorOp,
    LoadFunnelOp,
    UnloadFunnelOp,
    TareOp,
    WeighingOp,
    WeighResult
)
from archemist.stations.weighing_station.process import WeighingStationProcess
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import StationState, ProcessStatus, OpOutcome
from archemist.core.state.robot_op import RobotTaskOp, RobotNavOp, RobotWaitOp
from .testing_utils import test_req_robot_ops, test_req_station_op


class WeighingStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'WeighingStation',
            'id': 26,
            'location': {'coordinates': [1,7], 'descriptor': "WeighingStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': None,
            'materials': None
        }

        self.station = WeighingStation.from_dict(self.station_doc)

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_station_state(self):
        
        station = WeighingStation.from_dict(self.station_doc)
        self.assertIsNotNone(station)

        self.assertFalse(station.balance_doors_open)
        self.assertFalse(station.vertical_doors_open)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        station.add_lot(lot)

        # test WeighingVOpenDoorOp
        t_op = WeighingVOpenDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(station.vertical_doors_open)

        # test WeighingVCloseDoorOp
        t_op = WeighingVCloseDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test BalanceOpenDoorOp
        t_op = BalanceOpenDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test BalanceCloseDoorOp
        t_op = BalanceCloseDoorOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test TareOp
        t_op = TareOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test WeighingOp
        t_op = WeighingOp.from_args()
        self.assertIsNotNone(t_op.object_id)

        # test WeighingOpResult
        t_result_op = WeighResult.from_args(
            origin_op=t_op.object_id,
            reading_value=42.1)
        self.assertIsNotNone(t_result_op.object_id)
        self.assertEqual(t_result_op.reading_value, 42.1)
        self.assertEqual(t_result_op.unit, "g")

    def test_station_process(self):

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
        process = WeighingStationProcess.from_args()
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)

        # navigate_to_weighing_station
        process.tick()
        self.assertEqual(process.m_state, 'navigate_to_weighing_station')
        test_req_robot_ops(self, process, [RobotNavOp, RobotWaitOp])

        # open_fh_door_vertical
        process.tick()
        self.assertEqual(process.m_state, 'open_fh_door_vertical')
        test_req_station_op(self, process, WeighingVOpenDoorOp)

        # update_open_fh_door_vertical
        process.tick()
        self.assertEqual(process.m_state, 'update_open_fh_door_vertical')

        # tare
        process.tick()
        self.assertEqual(process.m_state, 'tare')
        test_req_station_op(self, process, TareOp)

        # open_balance_door
        process.tick()
        self.assertEqual(process.m_state, 'open_balance_door')
        test_req_station_op(self, process, BalanceOpenDoorOp)

        # update_open_balance_door
        process.tick()
        self.assertEqual(process.m_state, 'update_open_balance_door')

        # load_funnel
        process.tick()
        self.assertEqual(process.m_state, 'load_funnel')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # update_load_funnel
        process.tick()
        self.assertEqual(process.m_state, 'update_load_funnel')

        # close_balance_door
        process.tick()
        self.assertEqual(process.m_state, 'close_balance_door')
        test_req_station_op(self, process, BalanceCloseDoorOp)

        # update_close_balance_door
        process.tick()
        self.assertEqual(process.m_state, 'update_close_balance_door')

        # weigh
        process.tick()
        self.assertEqual(process.m_state, 'weigh')
        test_req_station_op(self, process, WeighingOp)

        # unload_funnel
        process.tick()
        self.assertEqual(process.m_state, 'unload_funnel')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # update_unload_funnel
        process.tick()
        self.assertEqual(process.m_state, 'update_unload_funnel')

        # close_fh_door_vertical
        process.tick()
        self.assertEqual(process.m_state, 'close_fh_door_vertical')
        test_req_station_op(self, process, WeighingVCloseDoorOp)

        # update_close_fh_door_vertical
        process.tick()
        self.assertEqual(process.m_state, 'update_close_fh_door_vertical')

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)


if __name__ == '__main__':
    unittest.main()




# add handler tests