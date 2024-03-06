import unittest

from mongoengine import connect
from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.core.state.robot_op import RobotTaskOp
from archemist.stations.waters_lcms_station.state import (WatersLCMSStation,
                                                          LCMSSampleAnalysisOp,
                                                          LCMSAnalysisResult,
                                                          LCMSInsertRackOp,
                                                          LCMSEjectRackOp,
                                                          LCMSAnalysisStatus)
from archemist.stations.waters_lcms_station.handler import SimWatersLCMSStationHandler
from archemist.stations.waters_lcms_station.process import APCLCMSAnalysisProcess
from archemist.core.util.enums import StationState, OpOutcome, ProcessStatus
from .testing_utils import test_req_robot_ops, test_req_station_op


class WatersLCMSStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        station_dict = {
            'type': 'WatersLCMSStation',
            'id': 20,
            'location': {'coordinates': [1, 7], 'descriptor': "WatersLCMSStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': None
        }

        self.station = WatersLCMSStation.from_dict(station_dict)

    def tearDown(self):
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        self.assertIsNotNone(self.station)
        self.assertEqual(self.station.state, StationState.INACTIVE)

        # test station specific members
        self.assertEqual(self.station.analysis_status, LCMSAnalysisStatus.INVALID)
        self.assertFalse(self.station.batch_inserted)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        self.station.add_lot(lot)

        # test LCMSInsertBatchOp
        t_op = LCMSInsertRackOp.from_args()
        self.assertIsNotNone(t_op)

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(self.station.batch_inserted)

        # test LCMSSampleAnalysisOp
        t_op = LCMSSampleAnalysisOp.from_args(batch_1.samples[0])
        self.assertIsNotNone(t_op)

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertEqual(self.station.analysis_status, LCMSAnalysisStatus.RUNNING_ANALYSIS)

        # test LCMSAnalysisResult
        t_result = LCMSAnalysisResult.from_args(origin_op=t_op.object_id,
                                                concentration=0.58,
                                                result_filename="file.xml")
        self.assertIsNotNone(t_result)
        self.assertEqual(t_result.concentration, 0.58)
        self.assertEqual(t_result.result_filename, "file.xml")

        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, [t_result])
        self.assertEqual(self.station.analysis_status, LCMSAnalysisStatus.ANALYSIS_COMPLETE)

        # test LCMSEjectBatchOp
        t_op = LCMSEjectRackOp.from_args()
        self.assertIsNotNone(t_op)

        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(self.station.batch_inserted)

    def test_apc_analysis_process(self):
        num_samples = 1
        batch = Batch.from_args(num_samples)
        lot = Lot.from_args([batch])

        # add batches to station
        self.station.add_lot(lot)

        # create station process
        process = APCLCMSAnalysisProcess.from_args(lot=lot,
                                                   target_batch_index=0,
                                                   target_sample_index=0)
        process.lot_slot = 0

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.m_state, 'prep_state')

        # place_vial
        process.tick()
        self.assertEqual(process.m_state, 'place_vial')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # insert_rack
        process.tick()
        self.assertEqual(process.m_state, 'insert_rack')
        test_req_station_op(self, process, LCMSInsertRackOp)

        # run_analysis
        process.tick()
        self.assertEqual(process.m_state, 'run_analysis')
        test_req_station_op(self, process, LCMSSampleAnalysisOp)

        # eject_rack
        process.tick()
        self.assertEqual(process.m_state, 'eject_rack')
        test_req_station_op(self, process, LCMSEjectRackOp)

        # dispose_vial
        process.tick()
        self.assertEqual(process.m_state, 'dispose_vial')
        test_req_robot_ops(self, process, [RobotTaskOp])

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
        handler = SimWatersLCMSStationHandler(self.station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct analyse op
        t_op = LCMSSampleAnalysisOp.from_args(batch_1.samples[0])
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()

        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], LCMSAnalysisResult))
        self.assertIsNotNone(op_results[0].concentration)
        self.assertIsNotNone(op_results[0].result_filename)
        self.station.complete_assigned_op(outcome, op_results)

        # construct analyse op
        t_op = LCMSInsertRackOp.from_args()
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()

        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsNone(op_results)


if __name__ == '__main__':
    unittest.main()
