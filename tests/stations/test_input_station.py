import unittest

import mongoengine

from archemist.core.state.station import StationProcessData
from archemist.stations.input_station.state import InputStation, InputStationPickupOp
from archemist.stations.input_station.process import InputStationProcess, CrystalWorkflowInputStationProcess
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import StationState
from archemist.core.util.location import Location
from utils import ProcessTestingMixin

class InputStationTest(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'InputStation',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'InputStationSm',
                'args': {}
            },
            'parameters':{}
        }

        self.station = InputStation.from_dict(self.station_doc,[],[])

    def tearDown(self):
        self.station.model.delete()
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        
        # test station op construction
        t_op = InputStationPickupOp.from_args()
        self.assertFalse(t_op.has_result)

    def test_input_station_process(self):
        # construct batches
        recipe_doc = {
            'general': 
            {
                'name': 'input_test_recipe',
                'id': 111
            },
            'process':
            [
                {
                    'state_name': 'pick_batch',
                    'station':
                    {
                        'type': 'InputStation',
                        'id': 23,
                        'operation':
                            {
                                'type': 'InputStationPickupOp',
                                'properties': {}
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
        process = InputStationProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')
        self.assertEqual(process.data.status['batch_index'], 0)

        for i in range(self.station_doc['process_batch_capacity']):
            # pickup_batch
            self.assert_process_transition(process, 'pickup_batch')
            self.assertEqual(process.data.status['batch_index'], i)

            # process robot_op
            req_robot_ops = self.station.get_requested_robot_ops()
            self.assertEqual(len(req_robot_ops),1)
            robot_op = req_robot_ops[0]
            self.assert_robot_op(robot_op, 'KukaLBRTask',
                            {'name': 'PickupInputRack',
                            'params':['False', str(i+1)]})
            self.complete_robot_op(self.station, robot_op)

            # removed_batch_update
            self.assert_process_transition(process, 'removed_batch_update')
            self.assertEqual(process.data.status['batch_index'], i+1)

        # final_state
        self.assertFalse(self.station.has_processed_batch())
        self.assert_process_transition(process, 'final_state')
        self.assertTrue(self.station.has_processed_batch())
        # empty processed batches 
        while self.station.has_processed_batch():
            _ = self.station.get_processed_batch()

    def test_crystalworkflow_input_station_process(self):
        # construct batches
        recipe_doc = {
            'general': 
            {
                'name': 'input_test_recipe',
                'id': 111
            },
            'process':
            [
                {
                    'state_name': 'pick_batch',
                    'station':
                    {
                        'type': 'InputStation',
                        'id': 23,
                        'operation':
                            {
                                'type': 'InputStationPickupOp',
                                'properties': {}
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
        process = CrystalWorkflowInputStationProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')
        self.assertEqual(process.data.status['batch_index'], 0)

        for i in range(self.station_doc['process_batch_capacity']):
            # pickup_batch
            self.assert_process_transition(process, 'pickup_batch')
            self.assertEqual(process.data.status['batch_index'], i)
            robot_op = self.station.get_requested_robot_ops()[0]
            self.assert_robot_op(robot_op, 'KukaLBRTask',
                            {'name': 'PickupEightWRack',
                            'params':['False', str(i+1)]})
            self.complete_robot_op(self.station, robot_op)

            # pickup_pxrd_plate
            self.assert_process_transition(process, 'pickup_pxrd_plate')
            self.assertEqual(process.data.status['batch_index'], i)
            robot_op = self.station.get_requested_robot_ops()[0]
            self.assert_robot_op(robot_op, 'KukaLBRTask',
                            {'name': 'PickupPXRDRack',
                            'params':['False']})
            self.complete_robot_op(self.station, robot_op)

            # removed_batch_update
            self.assert_process_transition(process, 'removed_batch_update')
            self.assertEqual(process.data.status['batch_index'], i+1)

        # final_state
        self.assertFalse(self.station.has_processed_batch())
        self.assert_process_transition(process, 'final_state')
        self.assertTrue(self.station.has_processed_batch())

if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()