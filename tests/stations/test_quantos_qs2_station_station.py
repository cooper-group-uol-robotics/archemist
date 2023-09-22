import unittest
from datetime import datetime

import mongoengine

from archemist.stations.quantos_qs2_station.state import (QuantosSolidDispenserQS2,
                                                          QuantosStatus,
                                                          QuantosCatridge,
                                                          OpenDoorOpDescriptor,
                                                          CloseDoorOpDescriptor,
                                                          MoveCarouselOpDescriptor,
                                                          QuantosDispenseOpDescriptor)
from archemist.core.state.material import Solid
from archemist.core.util.enums import StationState

class QuantosSolidDispenserQS2Test(unittest.TestCase):
    def setUp(self):
        self.station_doc = {
            'type': 'QuantosSolidDispenserQS2',
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
                'catridges': [{'id': 31, 'hotel_index': 1, 'remaining_dosages': 100}]
            }
        }
        solid_dict = {
            'name': 'salt',
            'id': 1235,
            'amount_stored': 2,
            'unit': 'g',
            'dispense_src': 'quantos',
            'cartridge_id': 31,
            'expiry_date': datetime.fromisoformat('2025-02-11')
        }
        
        self.solids_list = []
        self.solids_list.append(Solid.from_dict(solid_dict))
        self.station = QuantosSolidDispenserQS2.from_dict(station_dict=self.station_doc, 
                        liquids=[], solids=self.solids_list)
        
    def tearDown(self):
        self.station.model.delete()
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        
        # test station specific methods
        self.assertEqual(self.station.carousel_pos,1)
        self.station.carousel_pos = 2
        self.assertEqual(self.station.carousel_pos,2)

        self.assertIsNone(self.station.status)
        self.station.status = QuantosStatus.DOORS_OPEN
        self.assertEqual(self.station.status, QuantosStatus.DOORS_OPEN)
        self.station.status = QuantosStatus.DOORS_CLOSED
        self.assertEqual(self.station.status, QuantosStatus.DOORS_CLOSED)

        catridge_id = self.station.get_cartridge_id("salt")
        self.assertEqual(catridge_id, 31)
        self.assertIsNone(self.station.current_catridge)
        self.station.load_catridge(31)
        current_catridge = self.station.current_catridge
        self.assertTrue(isinstance(current_catridge, QuantosCatridge))
        self.assertEqual(current_catridge.hotel_index, 1)
        self.assertEqual(current_catridge.id, 31)
        self.assertEqual(current_catridge.remaining_dosages, 100)
        self.assertEqual(current_catridge.associated_solid.name, self.solids_list[0].name)
        self.assertEqual(current_catridge.associated_solid.mass, 2)
        self.assertEqual(current_catridge.associated_solid.mass, self.solids_list[0].mass)

        # test OpenDoorOpDescriptor
        t_op = OpenDoorOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.station.complete_assigned_station_op(True)
        ret_op = self.station.completed_station_ops.get(str(t_op.uuid))
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(self.station.status, QuantosStatus.DOORS_OPEN)

        # test CloseDoorOpDescriptor
        t_op = CloseDoorOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.station.complete_assigned_station_op(True)
        ret_op = self.station.completed_station_ops.get(str(t_op.uuid))
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(self.station.status, QuantosStatus.DOORS_CLOSED)

        # test MoveCarouselOpDescriptor
        t_op = MoveCarouselOpDescriptor.from_args(carousel_pos=12)
        self.assertFalse(t_op.has_result)
        self.assertEqual(t_op.carousel_pos, 12)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.station.complete_assigned_station_op(True)
        ret_op = self.station.completed_station_ops.get(str(t_op.uuid))
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(self.station.carousel_pos, 12)

        # test QuantosDispenseOpDescriptor
        t_op = QuantosDispenseOpDescriptor.from_args(solid_name='salt',dispense_mass=1.2)
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.station.complete_assigned_station_op(True, actual_dispensed_mass=1.193)
        ret_op = self.station.completed_station_ops.get(str(t_op.uuid))
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(ret_op.actual_dispensed_mass, 1.193)
        current_catridge = self.station.current_catridge
        self.assertEqual(current_catridge.remaining_dosages, 99)
        self.assertEqual(current_catridge.associated_solid.mass, 2 - 1.193)
        self.assertEqual(current_catridge.associated_solid.mass, self.solids_list[0].mass)

        self.station.unload_current_catridge()
        self.assertIsNone(self.station.current_catridge)

if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()