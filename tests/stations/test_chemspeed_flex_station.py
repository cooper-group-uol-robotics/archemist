import unittest
from typing import List
from datetime import datetime
from mongoengine import connect
from archemist.core.state.material import Liquid
from archemist.core.state.station import StationProcessData
from archemist.stations.chemspeed_flex_station.state import (ChemSpeedFlexStation, 
                                                             ChemSpeedStatus,
                                                             CSOpenDoorOpDescriptor,
                                                             CSCloseDoorOpDescriptor,
                                                             CSCSVJobOpDescriptor, 
                                                             CSProcessingOpDescriptor)
from archemist.stations.chemspeed_flex_station.process import ChemSpeedCSVProcess
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import StationState
from archemist.core.util.location import Location
from utils import ProcessTestingMixin

class ChemspeedFlexStationTest(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'ChemSpeedFlexStation',
            'id': 22,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'ChemSpeedRackSm',
                'args': {}
            },
            'parameters':
            {
                'used_liquids': ['water']
            }
        }
        liquid_dict = {
            'name': 'water',
            'id': 1235,
            'amount_stored': 400,
            'unit': 'ml',
            'density': 997,
            'pump_id': 'chemspeed',
            'expiry_date': datetime.fromisoformat('2025-02-11')
        }
        liquids_list = []
        liquids_list.append(Liquid.from_dict(liquid_dict))

        self.station = ChemSpeedFlexStation.from_dict(self.station_doc,liquids_list,[])

    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)

        # test station specific members
        self.assertIsNone(self.station.status)
        self.assertEqual(self.station.used_liquids_names, ['water'])
        liq = self.station.get_liquid('water')
        
        # test CSOpenDoorOpDescriptor
        t_op = CSOpenDoorOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.station.complete_assigned_station_op(True)
        self.assertEqual(self.station.status, ChemSpeedStatus.DOORS_OPEN)
        self.assertIsNotNone(self.station.completed_station_ops.get(str(t_op.uuid)))

        # test CSCloseDoorOpDescriptor
        t_op = CSCloseDoorOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.station.complete_assigned_station_op(True)
        self.assertEqual(self.station.status, ChemSpeedStatus.DOORS_CLOSED)
        self.assertIsNotNone(self.station.completed_station_ops.get(str(t_op.uuid)))

        # test CSProcessingOpDescriptor
        t_op = CSProcessingOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.assertEqual(self.station.status, ChemSpeedStatus.DOORS_CLOSED)
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.assertEqual(self.station.status, ChemSpeedStatus.RUNNING_JOB)
        self.station.complete_assigned_station_op(True)
        self.assertEqual(self.station.status, ChemSpeedStatus.JOB_COMPLETE)
        self.assertIsNotNone(self.station.completed_station_ops.get(str(t_op.uuid)))

        # test CSProcessingOpDescriptor
        t_op = CSCSVJobOpDescriptor.from_args(dispense_info={'water':[10,20]})
        self.assertEqual(t_op.dispense_info, {'water':[10,20]})
        self.assertEqual(t_op.get_csv_string(),r"10.0\n20.0\n")
        self.assertFalse(t_op.has_result)
        self.assertFalse(t_op.result_file)

        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.assertEqual(self.station.status, ChemSpeedStatus.JOB_COMPLETE)
        self.station.update_assigned_op()
        self.assertTrue(self.station.has_assigned_station_op())
        self.assertEqual(self.station.status, ChemSpeedStatus.RUNNING_JOB)
        self.assertEqual(liq.volume, 0.4)
        self.station.complete_assigned_station_op(True, result_file="some_file.txt")
        self.assertEqual(self.station.status, ChemSpeedStatus.JOB_COMPLETE)
        complete_op = self.station.completed_station_ops.get(str(t_op.uuid))
        self.assertEqual(complete_op.result_file, "some_file.txt")
        self.assertEqual(liq.volume, 0.37)

    def test_chemspeed_csv_dispense_process(self):
        # construct batches
        recipe_doc = {
            'general': 
            {
                'name': 'chemspeed_test_recipe',
                'id': 111
            },
            'process':
            [
                {
                    'state_name': 'dispense_liquid',
                    'station':
                    {
                        'type': 'ChemSpeedFlexStation',
                        'id': 23,
                        'operation':
                            {
                                'type': 'CSCSVJobOpDescriptor',
                                'properties': 
                                {
                                    'dispense_info':
                                    {
                                        'water': [10,20]
                                    }
                                }
                            }
                    },
                    'transitions':
                    {
                        'on_success': 'end_state',
                        'on_fail': 'end_state'
                    },
                },
            ]
        }

        batch_1 = Batch.from_arguments(31,2,Location(1,3,'table_frame'))
        recipe = Recipe.from_dict(recipe_doc)
        batch_1.attach_recipe(recipe)
        batch_2 = Batch.from_arguments(32,2,Location(1,3,'table_frame'))
        batch_2.attach_recipe(recipe)
        
        # add batches to station
        self.station.add_batch(batch_1)
        self.station.add_batch(batch_2)

        # create station process
        process_data = StationProcessData.from_args([batch_1, batch_2])
        process = ChemSpeedCSVProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')
        self.assertEqual(process.data.status['batch_index'], 0)

        # disable_auto_functions
        self.assert_process_transition(process, 'disable_auto_functions')
        req_robot_ops = self.station.get_requested_robot_ops()
        self.assertEqual(len(req_robot_ops),1)
        robot_op = req_robot_ops[0]
        self.assert_robot_op(robot_op, 'KukaLBRMaintenanceTask',
                                     {'name':'DiableAutoFunctions',
                                      'params':['False']})
        self.complete_robot_op(self.station, robot_op)

        # navigate_to_chemspeed
        self.assert_process_transition(process, 'navigate_to_chemspeed')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaNAVTask',
                                     {'target_location':Location(26,1,''),
                                      'fine_localisation':False})
        self.complete_robot_op(self.station, robot_op)

        # open_chemspeed_door
        self.assert_process_transition(process, 'open_chemspeed_door')
        req_station_ops = process.data.req_station_ops
        station_op = req_station_ops[0]
        self.assertEqual(len(req_station_ops),1)
        self.assert_station_op(station_op, 'CSOpenDoorOpDescriptor')
        self.complete_station_op(self.station)
        self.assertEqual(self.station.status, ChemSpeedStatus.DOORS_OPEN)

        for i in range(self.station_doc['process_batch_capacity']):
            # load_batch
            self.assert_process_transition(process, 'load_batch')
            robot_op = self.station.get_requested_robot_ops()[0]
            self.assert_robot_op(robot_op, 'KukaLBRTask',
                                {'name': 'LoadChemSpeed',
                                'params':['True', str(i+1)]})
            self.complete_robot_op(self.station, robot_op)

            # added_batch_update
            self.assert_process_transition(process, 'added_batch_update')
            self.assertEqual(process.data.status['batch_index'], i+1)

        # close_chemspeed_door
        self.assert_process_transition(process, 'close_chemspeed_door')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'CSCloseDoorOpDescriptor')
        self.complete_station_op(self.station)
        self.assertEqual(self.station.status, ChemSpeedStatus.DOORS_CLOSED)

        # enable_auto_functions
        self.assert_process_transition(process, 'enable_auto_functions')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRMaintenanceTask',
                                     {'name':'EnableAutoFunctions',
                                      'params':['False']})
        self.complete_robot_op(self.station, robot_op)

        # retreat_from_chemspeed
        self.assert_process_transition(process, 'retreat_from_chemspeed')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaNAVTask',
                                     {'target_location':Location(26,1,''),
                                      'fine_localisation':False})
        self.complete_robot_op(self.station, robot_op)

        # chemspeed_process
        self.assert_process_transition(process, 'chemspeed_process')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'CSCSVJobOpDescriptor', 
                          {'dispense_info': {'water': [10,20,10,20]}})
        self.complete_station_op(self.station)
        self.assertEqual(self.station.status, ChemSpeedStatus.JOB_COMPLETE)

        # disable_auto_functions
        self.assert_process_transition(process, 'disable_auto_functions')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRMaintenanceTask',
                                     {'name':'DiableAutoFunctions',
                                      'params':['False']})
        self.complete_robot_op(self.station, robot_op)

        # navigate_to_chemspeed
        self.assert_process_transition(process, 'navigate_to_chemspeed')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaNAVTask',
                                     {'target_location':Location(26,1,''),
                                      'fine_localisation':False})
        self.complete_robot_op(self.station, robot_op)

        # open_chemspeed_door
        self.assert_process_transition(process, 'open_chemspeed_door')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'CSOpenDoorOpDescriptor')
        self.complete_station_op(self.station)
        self.assertEqual(self.station.status, ChemSpeedStatus.DOORS_OPEN)

        for i in range(self.station_doc['process_batch_capacity'], 0, -1):
            # unload_batch
            self.assert_process_transition(process, 'unload_batch')
            robot_op = self.station.get_requested_robot_ops()[0]
            self.assert_robot_op(robot_op, 'KukaLBRTask',
                                {'name': 'UnloadChemSpeed',
                                'params':['False', str(i)]})
            self.complete_robot_op(self.station, robot_op)

            # removed_batch_update
            self.assert_process_transition(process, 'removed_batch_update')
            self.assertEqual(process.data.status['batch_index'], i-1)

        # close_chemspeed_door
        self.assert_process_transition(process, 'close_chemspeed_door')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'CSCloseDoorOpDescriptor')
        self.complete_station_op(self.station)
        self.assertEqual(self.station.status, ChemSpeedStatus.DOORS_CLOSED)

        # enable_auto_functions
        self.assert_process_transition(process, 'enable_auto_functions')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRMaintenanceTask',
                                     {'name':'EnableAutoFunctions',
                                      'params':['False']})
        self.complete_robot_op(self.station, robot_op)

        # final_state
        self.assertFalse(self.station.has_processed_batch())
        self.assert_process_transition(process, 'final_state')
        self.assertTrue(self.station.has_processed_batch())


if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()
