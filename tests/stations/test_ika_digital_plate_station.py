import unittest

import mongoengine

from archemist.core.state.station import StationProcessData
from archemist.stations.ika_digital_plate_station.state import (IkaPlateDigital, IKAMode, 
                                                                IKAStirringOpDescriptor,
                                                                IKAHeatingOpDescriptor,
                                                                IKAHeatingStirringOpDescriptor)
from archemist.stations.ika_digital_plate_station.process import CrystalBotWorkflowProcess
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import StationState
from archemist.core.util.location import Location
from utils import ProcessTestingMixin

class IkaPlateDigitalStationTest(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'IkaPlateDigital',
            'id': 20,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 1,
            'process_batch_capacity': 1,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'IKAStirPlateSm',
                'args': {}
            },
            'parameters':{}
        }

        self.station = IkaPlateDigital.from_dict(self.station_doc,[],[])
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        self.assertIsNone(self.station.mode)
        
        self.assertIsNone(self.station.current_temperature)
        self.station.current_temperature = 20
        self.assertEqual(self.station.current_temperature, 20)
        
        self.assertIsNone(self.station.target_temperature)

        self.assertIsNone(self.station.current_stirring_speed)
        self.station.current_stirring_speed = 120
        self.assertEqual(self.station.current_stirring_speed, 120)
        
        self.assertIsNone(self.station.target_stirring_speed)
        self.assertIsNone(self.station.target_duration)
        
        self.assertIsNone(self.station.external_temperature)
        self.station.external_temperature = 25
        self.assertEqual(self.station.external_temperature, 25)

        self.assertIsNone(self.station.viscosity_trend)
        self.station.viscosity_trend = 0.8
        self.assertEqual(self.station.viscosity_trend, 0.8)

        # test IKAStirringOpDescriptor
        t_op = IKAStirringOpDescriptor.from_args(stirring_speed=500, duration=10)
        self.assertFalse(t_op.has_result)
        
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.assertTrue(self.station.mode, IKAMode.STIRRING)
        self.assertTrue(self.station.target_stirring_speed, 500)
        self.assertTrue(self.station.target_duration, 10)
        
        self.station.complete_assigned_station_op(True)
        ret_op = self.station.completed_station_ops[str(t_op.uuid)]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertIsNone(self.station.mode)
        self.assertIsNone(self.station.target_stirring_speed)
        self.assertIsNone(self.station.target_duration)

        # test IKAHeatingOpDescriptor
        t_op = IKAHeatingOpDescriptor.from_args(temperature=140, duration=10)
        self.assertFalse(t_op.has_result)
        
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.assertTrue(self.station.mode, IKAMode.HEATING)
        self.assertTrue(self.station.target_temperature, 140)
        self.assertTrue(self.station.target_duration, 10)
        
        self.station.complete_assigned_station_op(True)
        ret_op = self.station.completed_station_ops[str(t_op.uuid)]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertIsNone(self.station.mode)
        self.assertIsNone(self.station.target_temperature)
        self.assertIsNone(self.station.target_duration)

        # test IKAHeatingStirringOpDescriptor
        t_op = IKAHeatingStirringOpDescriptor.from_args(temperature=140, stirring_speed=500, duration=10)
        self.assertFalse(t_op.has_result)
        
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.assertTrue(self.station.mode, IKAMode.HEATINGSTIRRING)
        self.assertTrue(self.station.target_temperature, 140)
        self.assertTrue(self.station.target_stirring_speed, 500)
        self.assertTrue(self.station.target_duration, 10)
        
        self.station.complete_assigned_station_op(True)
        ret_op = self.station.completed_station_ops[str(t_op.uuid)]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertIsNone(self.station.mode)
        self.assertIsNone(self.station.target_stirring_speed)
        self.assertIsNone(self.station.target_temperature)
        self.assertIsNone(self.station.target_duration)

    def test_crystal_workflow_process(self):
        # construct batches
        recipe_doc = {
            'general': 
            {
                'name': 'ika_plate_test_recipe',
                'id': 112
            },
            'process':
            [
                {
                    'state_name': 'stir_samples',
                    'station':
                    {
                        'type': 'IkaPlateDigital',
                        'id': 20,
                        'operation':
                            {
                                'type': 'IKAStirringOpDescriptor',
                                'properties': 
                                    {
                                        'stirring_speed': 1200,
                                        'duration': 10
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
        
        # add batches to station
        self.station.add_batch(batch_1)

        # create station process
        process_data = StationProcessData.from_args([batch_1])
        process = CrystalBotWorkflowProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')
        self.assertEqual(process.data.status['batch_index'], 0)

        # disabe_auto_functions_state
        self.assert_process_transition(process, 'disable_auto_functions')
        req_robot_ops = self.station.get_requested_robot_ops()
        self.assertEqual(len(req_robot_ops),1)
        robot_op = req_robot_ops[0]
        self.assert_robot_op(robot_op, 'KukaLBRMaintenanceTask',
                        {'name':'DiableAutoFunctions',
                        'params':['False']})
        self.complete_robot_op(self.station, robot_op)

        # place_8_well_rack
        self.assert_process_transition(process, 'place_8_well_rack')
        self.assertEqual(process.data.status['batch_index'], 0)
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                        {'name': 'LoadEightWRackYumiStation',
                        'params':['True']})
        self.complete_robot_op(self.station, robot_op)

        # place_pxrd_rack
        self.assert_process_transition(process, 'place_pxrd_rack')
        self.assertEqual(process.data.status['batch_index'], 0)
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                        {'name': 'LoadPXRDRackYumiStation',
                        'params':['False']})
        self.complete_robot_op(self.station, robot_op)

        # added_batch_update
        self.assert_process_transition(process, 'added_batch_update')

        # load_stir_plate
        self.assert_process_transition(process, 'load_stir_plate')
        self.assertEqual(process.data.status['batch_index'], 0)
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'YuMiRobotTask',
                        {'name': 'loadIKAPlate'})
        self.complete_robot_op(self.station, robot_op)

        # stir
        self.assert_process_transition(process, 'stir')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'IKAStirringOpDescriptor',
                               {'target_stirring_speed': 1200,
                                'target_duration': 10})
        self.complete_station_op(self.station)

        # enable_auto_functions_state
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
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()