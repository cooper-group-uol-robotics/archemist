import unittest

import mongoengine

from archemist.core.state.station import StationProcessData
from archemist.stations.output_station.state import OutputStation, OutputStationPlaceOp
from archemist.stations.output_station.process import OutputStationProcess
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import StationState
from archemist.core.util.location import Location
from utils import ProcessTestingMixin

class InputStationTest(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'OutputStation',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'OutputStationProcess',
                'args': {}
            },
            'parameters':{}
        }

        self.station = OutputStation.from_dict(self.station_doc,[],[])
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        
        # test station op construction
        t_op = OutputStationPlaceOp.from_args()
        self.assertFalse(t_op.has_result)

    def test_output_station_process(self):
        # construct batches
        recipe_doc = {
            'general': 
            {
                'name': 'output_station_test_recipe',
                'id': 111
            },
            'process':
            [
                {
                    'state_name': 'place_batch',
                    'station':
                    {
                        'type': 'OutputStation',
                        'id': 23,
                        'operation':
                            {
                                'type': 'OutputStationPlaceOp',
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
        process = OutputStationProcess(self.station, process_data)

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

        for i in range(self.station_doc['process_batch_capacity']):
            # place_batch
            self.assert_process_transition(process, 'place_batch')
            self.assertEqual(process.data.status['batch_index'], i)
            robot_op = self.station.get_requested_robot_ops()[0]
            self.assert_robot_op(robot_op, 'KukaLBRTask',
                            {'name': 'PlaceRack',
                            'params':['False', str(i+1)]})
            self.complete_robot_op(self.station, robot_op)

            # added_batch_update
            self.assert_process_transition(process, 'added_batch_update')
            self.assertEqual(process.data.status['batch_index'], i+1)

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