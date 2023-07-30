import unittest

import mongoengine

from archemist.stations.fisher_balance_station.state import FisherWeightingStation, FisherWeightOpDescriptor
from archemist.core.util.enums import StationState

class FisherBalanceStationTest(unittest.TestCase):
    def setUp(self):
        self.station_doc = {
            'type': 'FisherWeightingStation',
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

        self.station = FisherWeightingStation.from_dict(self.station_doc,[],[])
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        
        # test station op construction
        t_op = FisherWeightOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_station_op(True, weight=1.2)

        ret_t_op = self.station.completed_station_ops[str(t_op.uuid)]
        self.assertTrue(ret_t_op.has_result)
        self.assertTrue(ret_t_op.was_successful)
        self.assertEqual(ret_t_op.weight, 1.2)

if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()