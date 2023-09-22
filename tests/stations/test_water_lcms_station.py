import unittest

import mongoengine

from archemist.stations.waters_lcms_station.state import (WatersLCMSStation,
                                                          LCMSStatus,
                                                          LCMSInsertBatchOpDescriptor, 
                                                          LCMSExtractBatchOpDescriptor, 
                                                          LCMSAnalysisOpDescriptor)
from archemist.core.util.enums import StationState

class WatersLCMSStationTest(unittest.TestCase):
    def setUp(self):
        self.station_doc = {
            'type': 'WatersLCMSStation',
            'id': 20,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': '',
                'args': {}
            },
            'parameters':{}
        }

        self.station = WatersLCMSStation.from_dict(self.station_doc,[],[])

    def tearDown(self):
        self.station.model.delete()
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        self.assertIsNone(self.station.status)
        
        # test LCMSInsertBatchOpDescriptor
        t_op = LCMSInsertBatchOpDescriptor.from_args(used_rack_index=1)
        self.assertFalse(t_op.has_result)
        self.assertEqual(t_op.used_rack_index, 1)
        
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_station_op(True)
        self.assertEqual(self.station.status, LCMSStatus.BATCH_LOADED)

        # test LCMSAnalysisOpDescriptor
        t_op = LCMSAnalysisOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.assertEqual(self.station.status, LCMSStatus.RUNNING_ANALYSIS)
        self.station.complete_assigned_station_op(True)
        self.assertEqual(self.station.status, LCMSStatus.ANALYSIS_COMPLETE)

        # test LCMSExtractBatchOpDescriptor
        t_op = LCMSExtractBatchOpDescriptor.from_args(used_rack_index=1)
        self.assertFalse(t_op.has_result)
        self.assertEqual(t_op.used_rack_index, 1)
        
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_station_op(True)
        self.assertEqual(self.station.status, LCMSStatus.BATCH_READY_FOR_COLLECTION)

        ret_t_op = self.station.completed_station_ops[str(t_op.uuid)]
        self.assertTrue(ret_t_op.has_result)
        self.assertTrue(ret_t_op.was_successful)

if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()