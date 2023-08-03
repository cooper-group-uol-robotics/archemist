import unittest
from datetime import datetime

import mongoengine

from archemist.stations.quantos_qb1_station.state import (QuantosSolidDispenserQB1,
                                                          QuantosStatus,
                                                          QuantosCartridge,
                                                          OpenDoorOpDescriptor,
                                                          CloseDoorOpDescriptor,
                                                          MoveCarouselOpDescriptor,
                                                          DispenseOpDescriptor)


from archemist.stations.quantos_qb1_station.process import QuantosQb1StationProcess

from archemist.core.state.material import Solid
from archemist.core.util.enums import StationState
from archemist.core.state.station import StationProcessData
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.location import Location


from utils import ProcessTestingMixin

class QuantosSolidDispenserQB1Test(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'QuantosSolidDispenserQB1',
            'id': 20,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'QuantosQb1StationProcess',
                'args': {"solid_name": "caffeine",
                          "target_mass": 10,
                          "tolerance": 10}
            },
            'parameters':{
                'cartridges': [{'id': 31, 'hotel_index': 1, 'remaining_dosages': 100, 'remaining_quantity':100}]
            }
        }
        solid_dict = {
            'name': 'salt',
            'id': 31,
            'amount_stored': 2000,
            'unit': 'g',
            'dispense_src': 'quantos',
            'cartridge_id': 31,
            'expiry_date': datetime.fromisoformat('2025-02-11')
        }
        
        self.solids_list = []
        self.solids_list.append(Solid.from_dict(solid_dict))
        self.station = QuantosSolidDispenserQB1.from_dict(station_dict=self.station_doc, 
                        solids=self.solids_list, liquids = [])
        
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

        cartridge_id = self.station.get_cartridge_id("salt")
        self.assertEqual(cartridge_id, 31)
        self.assertIsNone(self.station.current_cartridge)
        self.station.load_cartridge(31)
        current_cartridge = self.station.current_cartridge
        self.assertTrue(isinstance(current_cartridge, QuantosCartridge))
        self.assertEqual(current_cartridge.hotel_index, 1)
        self.assertEqual(current_cartridge.id, 31)
        self.assertEqual(current_cartridge.remaining_dosages, 100)
        self.assertEqual(current_cartridge.remaining_quantity,100)
        self.assertEqual(current_cartridge.associated_solid.name, self.solids_list[0].name)
        self.assertEqual(current_cartridge.associated_solid.mass, 2000)
        self.assertEqual(current_cartridge.associated_solid.mass, self.solids_list[0].mass)

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

        # test DispenseOpDescriptorModel
        t_op = DispenseOpDescriptor.from_args(solid_name='salt',target_mass =1.2, tolerance = 10)
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
        current_cartridge = self.station.current_cartridge
        self.assertEqual(current_cartridge.remaining_dosages, 99)
        self.assertEqual(current_cartridge.associated_solid.mass, 2000 - 1.193)
        self.assertEqual(current_cartridge.associated_solid.mass, self.solids_list[0].mass)

        self.station.unload_current_cartridge()
        self.assertIsNone(self.station.current_cartridge)

    def test_proccess(self):
        recipe_dict =  {
            "general":
             {
            "name": "quantos_rih_test_recipe",
            "id": 1},

            "materials":
            {"solids": 
                [{
                    "name": "salt",
                    "id": 31


                }]

            },

            "process" :
            [
                {
                    "state_name": "dispense_solid",
                    "station":
                    {
                        "type": "QauntosSolidDispenserQB1",
                        "id": 20,
                        "operation":
                        {
                            "type": "DispenseOpDescriptor", 
                            "properties":
                            {
                                "solid_name": "salt",
                                "target_mass": 10,
                                "tolerance": 10
                            }

                        }
                    
                    },
                    "transitions": 
                    {
                        'on_success': 'end_state',
                        'on_fail': 'end_state'
                    },
                },
            ]
            }

        batch_1 = Batch.from_arguments(31,1,Location(1,3,'table_frame')) #not sure if this is correctly set up
        recipe = Recipe.from_dict(recipe_dict)
        batch_1.attach_recipe(recipe)
        
        # add batches to station
        self.station.add_batch(batch_1)

        # create station process
        process_data = StationProcessData.from_args([batch_1])
        process = QuantosQb1StationProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')

        # open_door
        self.assert_process_transition(process, 'open_quantos_door')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'OpenDoorOpDescriptor')
        self.complete_station_op(station=self.station)


        # load_vial
        self.assert_process_transition(process, 'load_vial')
        robot_op = self.station.get_requested_robot_ops()[0]
        
        self.assert_robot_op(robot_op, 'PandaRobotTask',
                        {'name': 'PresentVial',
                         'params': [] }) #no idea about these values
        self.complete_robot_op(self.station, robot_op)

        # load_cartridge
        self.assert_process_transition(process, 'load_cartridge')
        
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'PandaRobotTask',
                              {'name': 'PresentCartridge',
                               'params': []})
                               
        self.complete_robot_op(self.station, robot_op)

        #close_quantos_door
        self.assert_process_transition(process, 'close_quantos_door')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'CloseDoorOpDescriptor')
        self.complete_station_op(station=self.station)

        #quantos_dispense
        self.assert_process_transition(process, 'quantos_dispense')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'DispenseOpDescriptor')
        self.complete_station_op(self.station, actual_dispensed_mass= 10)

        #open_quantos_door
        self.assert_process_transition(process, 'open_quantos_door')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'OpenDoorOpDescriptor')
        self.complete_station_op(self.station)


        #unload_vial
        self.assert_process_transition(process, 'unload_vial')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'PandaRobotTask',
                        {'name': 'ReturnVial',
                         'params': [] })
        self.complete_robot_op(self.station, robot_op)

        
        #unload_cartridge
        self.assert_process_transition(process,'unload_cartridge')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'PandaRobotTask',
                        {'name': 'ReturnCartridge',
                         'params': [] })
        self.complete_robot_op(self.station, robot_op)
          
        # final_state
        self.assertFalse(self.station.has_processed_batch())
        self.assert_process_transition(process, 'final_state')
        self.assertTrue(self.station.has_processed_batch())



if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main(verbosity=4)
    unittest.main()