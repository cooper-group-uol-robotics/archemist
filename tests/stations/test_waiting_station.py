import unittest

from mongoengine import connect

from archemist.stations.waiting_station.state import WaitingStation, WaitOp
from archemist.stations.waiting_station.process import WaitingStationProcess
from archemist.core.state.robot_op import DropBatchOpDescriptor, CollectBatchOpDescriptor
from archemist.core.state.station_op_result import ProcessOpResult
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import StationState, ProcessStatus, OpOutcome
from .testing_utils import test_req_robot_ops

class WaitingStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'WaitingStation',
            'id': 23,
            'location': {'coordinates': [1,7], 'descriptor': "WaitingStation"},
            'total_lot_capacity': 1,
            'handler': 'WaitingStationHandler',
        }

        self.station = WaitingStation.from_dict(self.station_doc)

    def tearDown(self):
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()
    
    def test_state(self):
        # test station is constructed properly
        self.assertIsNotNone(self.station)
        self.assertEqual(self.station.state, StationState.INACTIVE)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        self.station.add_lot(lot)
        
        # test station op construction
        t_op = WaitOp.from_args(target_lot=lot, duration=12, time_unit="minute")
        self.assertEqual(t_op.duration, 12)
        self.assertEqual(t_op.time_unit, "minute")

    def test_waiting_station_process(self):
        # construct batches        
        batch_1 = Batch.from_args(3, self.station.location)
        batch_2 = Batch.from_args(3, self.station.location)
        lot = Lot.from_args([batch_1, batch_2])
        self.station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "wait_op",
                    "op": "WaitOp",
                    "parameters": {
                        "duration": 5,
                        "time_unit": "second"
                    }
                }
            ]

        # create station process
        process = WaitingStationProcess.from_args(lot=lot,
                                                  operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)


        # load_lot
        process.tick()
        self.assertEqual(process.m_state, 'load_lot')
        test_req_robot_ops(self, process, [DropBatchOpDescriptor]*2)

        # waiting_process
        process.tick()
        self.assertEqual(process.m_state, 'waiting_process')
        wait_op = process.req_station_ops[0]
        parameters = {"duration": wait_op.duration, "time_unit": wait_op.time_unit}
        wait_op.complete_op(OpOutcome.SUCCEEDED, [ProcessOpResult.from_args(origin_op=wait_op.object_id, parameters=parameters)])

        # unload_lot
        process.tick()
        self.assertEqual(process.m_state, 'unload_lot')
        test_req_robot_ops(self, process, [CollectBatchOpDescriptor]*2)

        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

if __name__ == '__main__':
    unittest.main()