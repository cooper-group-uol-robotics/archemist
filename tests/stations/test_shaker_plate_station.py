import unittest

from mongoengine import connect

from archemist.stations.shaker_plate_station.state import ShakerPlateStation, ShakerPlateOp
from archemist.stations.shaker_plate_station.process import PXRDWorkflowShakingProcess
from archemist.stations.shaker_plate_station.handler import SimShakePlateHandler
from archemist.core.state.robot_op import RobotTaskOpDescriptor, CollectBatchOpDescriptor
from archemist.core.state.lot import Lot, Batch
from archemist.core.util.enums import StationState, OpOutcome, ProcessStatus
from .testing_utils import test_req_robot_ops, test_req_station_op

class ShakerPlateStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'ShakerPlateStation',
            'id': 20,
            'location': {'coordinates': [1,7], 'descriptor': "ShakerPlateStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'materials': None,
            'parameters': None
        }

        self.station = ShakerPlateStation.from_dict(self.station_doc)

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

        # test station specific members
        self.assertFalse(self.station.is_shaking)

        # test ShakerPlateOp
        t_op = ShakerPlateOp.from_args(target_batch=batch_1,
                                       duration=3,
                                       time_unit="minute")
        self.assertIsNotNone(t_op.object_id)
        self.assertEqual(t_op.duration, 3)
        self.assertEqual(t_op.time_unit, "minute")
        
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertTrue(self.station.is_shaking)
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(self.station.is_shaking)
                

    def test_yumi_shaker_plate_process(self):
        # construct batches
        batch_1 = Batch.from_args(2)
        batch_2 = Batch.from_args(2)
        lot = Lot.from_args([batch_1, batch_2])
        
        # add lot to station
        self.station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "shake_samples",
                    "op": "ShakerPlateOp",
                    "parameters": {
                        "duration": 3,
                        "time_unit": "minute"
                    }
                }
            ]
        process = PXRDWorkflowShakingProcess.from_args(lot=lot, operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')

        # load_shaker_plate
        process.tick()
        self.assertEqual(process.m_state, 'load_shaker_plate')
        test_req_robot_ops(self, process, [RobotTaskOpDescriptor])

        # shake
        process.tick()
        self.assertEqual(process.m_state, 'shake')
        test_req_station_op(self, process, ShakerPlateOp)

        # unload_shaker_plate
        process.tick()
        self.assertEqual(process.m_state, 'unload_shaker_plate')
        test_req_robot_ops(self, process, [RobotTaskOpDescriptor])

        # unscrew_caps
        process.tick()
        self.assertEqual(process.m_state, 'unscrew_caps')
        test_req_robot_ops(self, process, [RobotTaskOpDescriptor])
        
        # pick_lot
        process.tick()
        self.assertEqual(process.m_state, 'pick_lot')
        test_req_robot_ops(self, process, [CollectBatchOpDescriptor]*2)

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
        handler = SimShakePlateHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct op
        t_op = ShakerPlateOp.from_args(target_batch=batch_1,
                                       duration=3,
                                       time_unit="minute")
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        # get op 
        parameters = {}
        parameters["duration"]  = t_op.duration
        parameters["time_unit"]  = t_op.time_unit
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertDictEqual(op_results[0].parameters, parameters)

if __name__ == '__main__':
    unittest.main()