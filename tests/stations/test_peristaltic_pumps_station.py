import unittest
from datetime import datetime

import mongoengine

from archemist.stations.peristaltic_pumps_station.state import PeristalticLiquidDispensing, PeristalticPumpOpDescriptor
from archemist.core.state.material import Liquid
from archemist.core.util.enums import StationState

class PeristalticLiquidDispensingTest(unittest.TestCase):
    def setUp(self):
        self.station_doc = {
            'type': 'PeristalticLiquidDispensing',
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
            'parameters':{
                'liquid_pump_map': {'water': 'pUmP1'}
            }
        }
        liquid_dict = {
            'name': 'water',
            'id': 1235,
            'amount_stored': 400,
            'unit': 'ml',
            'density': 997,
            'pump_id': 'pUmP1',
            'expiry_date': datetime.fromisoformat('2025-02-11')
        }
        
        self.liquids_list = []
        self.liquids_list.append(Liquid.from_dict(liquid_dict))
        self.station = PeristalticLiquidDispensing.from_dict(station_dict=self.station_doc, 
                        liquids=self.liquids_list, solids=[])
        
    def tearDown(self):
        self.station.model.delete()
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        self.assertEqual(self.station.get_liquid(pump_id='pUmP1').name, self.liquids_list[0].name)
        self.assertEqual(self.station.get_pump_id('water'), 'pUmP1')

        # test PeristalticPumpOpDescriptor
        t_op = PeristalticPumpOpDescriptor.from_args(liquid_name='water',dispense_volume=100)
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.station.complete_assigned_station_op(True, actual_dispensed_volume=99)
        ret_op = self.station.completed_station_ops.get(str(t_op.uuid))
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(ret_op.actual_dispensed_volume, 99)
        self.assertEqual(self.liquids_list[0].volume, 0.4-0.099)

if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()