import unittest

from mongoengine import connect
from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.stations.waters_lcms_station.state import (WatersLCMSStation,
                                                          LCMSBayOccupiedOp,
                                                          LCMSBayFreedOp,
                                                          LCMSAnalysisOp,
                                                          LCMSAnalysisResult, 
                                                          LCMSInsertBatchOp, 
                                                          LCMSEjectBatchOp,
                                                          LCMSAutoLoaderStatus,
                                                          LCMSAnalysisStatus)
from archemist.stations.waters_lcms_station.handler import SimWatersLCMSStationHandler
from archemist.core.util.enums import StationState, OpOutcome

class WatersLCMSStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')


        station_dict = {
            'type': 'WatersLCMSStation',
            'id': 20,
            'location': {'coordinates': [1,7], 'descriptor': "WatersLCMSStation"},
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
        self.assertEqual(self.station.auto_loader_status, LCMSAutoLoaderStatus.BAY_FREE)

        # construct lot and add it to station
        batch_1 = Batch.from_args(2)
        lot = Lot.from_args([batch_1])
        self.station.add_lot(lot)
        
        # test LCMSBayOccupiedOp
        t_op = LCMSBayOccupiedOp.from_args(bay_index=1)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.bay_index, 1)
        
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(self.station.auto_loader_status, LCMSAutoLoaderStatus.BAY_OCCUPIED)

        # test LCMSInsertBatchOp
        t_op = LCMSInsertBatchOp.from_args(target_batch=batch_1, bay_index=1)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.bay_index, 1)
        
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(self.station.auto_loader_status, LCMSAutoLoaderStatus.BAY_UNAVAILABLE)

        # test LCMSAnalysisOp
        t_op = LCMSAnalysisOp.from_args()
        self.assertIsNotNone(t_op)
        
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.assertEqual(self.station.analysis_status, LCMSAnalysisStatus.RUNNING_ANALYSIS)

        # test LCMSAnalysisResult
        t_result = LCMSAnalysisResult.from_args(origin_op=t_op.object_id, result_filename="file.xml")
        self.assertIsNotNone(t_result)
        self.assertEqual(t_result.result_filename, "file.xml")

        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, [t_result])
        self.assertEqual(self.station.analysis_status, LCMSAnalysisStatus.ANALYSIS_COMPLETE)

        # test LCMSEjectBatchOp
        t_op = LCMSEjectBatchOp.from_args(target_batch=batch_1, bay_index=1)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.bay_index, 1)
        
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(self.station.auto_loader_status, LCMSAutoLoaderStatus.BAY_OCCUPIED)

        # test LCMSBayFreedOp
        t_op = LCMSBayFreedOp.from_args(bay_index=1)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.bay_index, 1)
        
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(self.station.auto_loader_status, LCMSAutoLoaderStatus.BAY_FREE)

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
        t_op = LCMSAnalysisOp.from_args()
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], LCMSAnalysisResult))
        self.assertIsNotNone(op_results[0].result_filename)
        self.station.complete_assigned_op(outcome, op_results)

        # construct analyse op
        t_op = LCMSInsertBatchOp.from_args(target_batch=batch_1, bay_index=1)
        self.station.add_station_op(t_op)
        self.station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsNone(op_results)

        

if __name__ == '__main__':
    unittest.main()