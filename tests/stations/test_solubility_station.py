import unittest

from mongoengine import connect

from archemist.core.state.lot import Lot, Batch
from archemist.stations.solubility_station.state import (SolubilityStation,
                                                         CheckSolubilityOp,
                                                         SolubilityState,
                                                         SolubilityOpResult)
from archemist.stations.solubility_station.process import SolubilityStationProcess
from archemist.stations.solubility_station.handler import SimSolubilityStationHandler
from archemist.core.util.enums import ProcessStatus, OpOutcome, StationState
from archemist.core.state.robot_op import RobotTaskOpDescriptor
from .testing_utils import test_req_robot_ops, test_req_station_op

class SolubilityStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        station_dict = {
            'type': 'SolubilityStation',
            'id': 25,
            'location': {'coordinates': [1,7], 'descriptor': "SolubilityStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'parameters':{}
        }

        self.station = SolubilityStation.from_dict(station_dict)

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


        # test CheckSolubilityOp
        t_op = CheckSolubilityOp.from_args(lot.batches[0].samples[0])
        self.assertIsNotNone(t_op.object_id)

        # test SolubilityOpResult
        op_result = SolubilityOpResult.from_args(origin_op=t_op.object_id,
                                                 solubility_state=SolubilityState.DISSOLVED,
                                                 result_filename="test123.png")
        self.assertEqual(op_result.solubility_state, SolubilityState.DISSOLVED)
        self.assertEqual(op_result.result_filename, "test123.png")
    
    def test_solubility_station_process(self):
        num_samples = 3
        batch_1 = Batch.from_args(num_samples)
        batch_2 = Batch.from_args(num_samples)
        lot = Lot.from_args([batch_1, batch_2])
        
        # add batches to station
        self.station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "check_solubility",
                    "op": "CheckSolubilityOp",
                    "parameters": None
                }
            ]
        process = SolubilityStationProcess.from_args(lot=lot,
                                            operations=operations)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.m_state, 'prep_state')
        self.assertEqual(process.data['batch_index'], 0)
        self.assertEqual(process.data['sample_index'], 0)

        for i in range(lot.num_batches):
            for j in range(num_samples):
                # load_sample
                process.tick()
                self.assertEqual(process.m_state, 'load_sample')
                test_req_robot_ops(self, process, [RobotTaskOpDescriptor])

                # station_process
                process.tick()
                self.assertEqual(process.m_state, 'station_process')
                test_req_station_op(self, process, CheckSolubilityOp)

                # unload_sample
                process.tick()
                self.assertEqual(process.m_state, 'unload_sample')
                test_req_robot_ops(self, process, [RobotTaskOpDescriptor])

                # update_sample_index
                process.tick()
                self.assertEqual(process.m_state, 'update_sample_index')
                self.assertEqual(process.data['batch_index'], i)
                self.assertEqual(process.data['sample_index'], j+1)
            
            # update_batch_index
            process.tick()
            self.assertEqual(process.m_state, 'update_batch_index')
            self.assertEqual(process.data['batch_index'], i+1)
            self.assertEqual(process.data['sample_index'], 0)

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
        handler = SimSolubilityStationHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct analyse op
        t_op = CheckSolubilityOp.from_args(target_sample=batch_1.samples[0])
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], SolubilityOpResult))
        self.assertIsNotNone(op_results[0].result_filename)
        self.assertIn(op_results[0].solubility_state, [SolubilityState.DISSOLVED, SolubilityState.UNDISSOLVED])
        self.station.complete_assigned_op(outcome, op_results)

if __name__ == '__main__':
    unittest.main()