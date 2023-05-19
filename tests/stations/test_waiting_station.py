import unittest
import time
from datetime import timedelta

import mongoengine

from archemist.core.util.enums import TimeUnit
from archemist.core.state.station import StationProcessData
from archemist.stations.waiting_station.state import WaitingStation, WaitingOpDescriptor
from archemist.stations.waiting_station.process import WaitingStationProcess
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import StationState
from archemist.core.util.location import Location
from utils import ProcessTestingMixin

class WaitingStationTest(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'WaitingStation',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'WaitingStationSm',
                'args': {}
            },
            'parameters':{}
        }

        self.station = WaitingStation.from_dict(self.station_doc,[],[])

    def tearDown(self):
        self.station.model.delete()
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        
        # test station op construction
        t_op = WaitingOpDescriptor.from_args(duration=12, time_unit="min")
        self.assertFalse(t_op.has_result)
        self.assertEqual(t_op.duration, 12)
        self.assertEqual(t_op.time_unit, TimeUnit.MINUTES)

    def test_waiting_station_process(self):
        # construct batches
        WAITING_DURATION = 3
        
        recipe_doc = {
            'general': 
            {
                'name': 'waiting_station_test_recipe',
                'id': 111
            },
            'process':
            [
                {
                    'state_name': 'age_samples',
                    'station':
                    {
                        'type': 'WaitingStation',
                        'id': 23,
                        'operation':
                            {
                                'type': 'WaitingOpDescriptor',
                                'properties': 
                                {
                                    'duration': WAITING_DURATION,
                                    'time_unit': 'sec'
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
        process = WaitingStationProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')
        self.assertEqual(process.data.status['batch_index'], 0)
        self.assertEqual(process.data.status['timer_expiry_datetime'], "")
        self.assertIsNone(process.data.status['stored_op'])


        for i in range(self.station_doc['process_batch_capacity']):
            # load_batch
            self.assert_process_transition(process, 'load_batch')
            robot_op = self.station.get_requested_robot_ops()[0]
            self.assert_robot_op(robot_op, 'KukaLBRTask',
                                {'name': 'LoadWaitingStation',
                                'params':['True', str(i+1)]})
            self.complete_robot_op(self.station, robot_op)

            # added_batch_update
            self.assert_process_transition(process, 'added_batch_update')
            self.assertEqual(process.data.status['batch_index'], i+1)

        # waiting_process
        self.assert_process_transition(process, 'waiting_process')
        for i in range(WAITING_DURATION - 1):
            self.assert_process_transition(process, 'waiting_process')
            time.sleep(1)
        time.sleep(1)

        for i in range(self.station_doc['process_batch_capacity'], 0, -1):
            # unload_batch
            self.assert_process_transition(process, 'unload_batch')
            robot_op = self.station.get_requested_robot_ops()[0]
            self.assert_robot_op(robot_op, 'KukaLBRTask',
                                {'name': 'UnloadWaitingStation',
                                'params':['False', str(i)]})
            self.complete_robot_op(self.station, robot_op)

            # removed_batch_update
            self.assert_process_transition(process, 'removed_batch_update')
            self.assertEqual(process.data.status['batch_index'], i-1)

        # final_state
        self.assertFalse(self.station.has_processed_batch())
        self.assert_process_transition(process, 'final_state')
        self.assertTrue(self.station.has_processed_batch())

if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()