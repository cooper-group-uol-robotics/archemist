import unittest

import mongoengine

from archemist.core.state.station import StationState, StationProcessData
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.stations.lightbox_station.state import LightBoxStation, SampleColorOpDescriptor
from archemist.stations.lightbox_station.process import LightBoxProcess
from archemist.core.util.location import Location
from utils import ProcessTestingMixin

class LightBoxStationTest(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'LightBoxStation',
            'id': 25,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'process_state_machine': 
            {
                'type': 'LightBoxProcess',
                'args': {}
            },
            'parameters':{}
        }

        self.station = LightBoxStation.from_dict(self.station_doc, [], [])

    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)

        # test SampleColorOpDescriptor
        t_op = SampleColorOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.station.complete_assigned_station_op(True, result_filename='file.test', 
                                                  red_intensity=128, green_intensity=253, 
                                                  blue_intensity=11)
        ret_t_op = self.station.completed_station_ops[str(t_op.uuid)]
        self.assertTrue(ret_t_op.has_result)
        self.assertTrue(ret_t_op.was_successful)
        self.assertEqual(ret_t_op.result_filename, 'file.test')
        self.assertEqual(ret_t_op.red_intensity, 128)
        self.assertEqual(ret_t_op.green_intensity, 253)
        self.assertEqual(ret_t_op.blue_intensity, 11)
    
    def test_lightbox_process(self):
        # construct batches
        recipe_doc = {
            'general': 
            {
                'name': 'lightbox_test_recipe',
                'id': 111
            },
            'process':
            [
                {
                    'state_name': 'analyse_samples',
                    'station':
                    {
                        'type': 'LightBoxStation',
                        'id': 111,
                        'operation':
                            {
                                'type': 'SampleColorOpDescriptor',
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

        num_samples = 3
        batch_1 = Batch.from_arguments(31,num_samples,Location(1,3,'table_frame'))
        recipe = Recipe.from_dict(recipe_doc)
        batch_1.attach_recipe(recipe)
        batch_2 = Batch.from_arguments(32,num_samples,Location(1,3,'table_frame'))
        batch_2.attach_recipe(recipe)
        
        # add batches to station
        self.station.add_batch(batch_1)
        self.station.add_batch(batch_2)

        # create station process
        process_data = StationProcessData.from_args([batch_1, batch_2])
        process = LightBoxProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')
        self.assertEqual(process.data.status['batch_index'], 0)
        self.assertEqual(process.data.status['sample_index'], 0)

        # disable_auto_functions
        self.assert_process_transition(process, 'disable_auto_functions')
        req_robot_ops = self.station.get_requested_robot_ops()
        self.assertEqual(len(req_robot_ops),1)
        robot_op = req_robot_ops[0]
        self.assert_robot_op(robot_op, 'KukaLBRMaintenanceTask',
                                     {'name':'DiableAutoFunctions',
                                      'params':['False']})
        self.complete_robot_op(self.station, robot_op)

        for i in range(self.station_doc['process_batch_capacity']):
            for j in range(num_samples):
                # load_sample
                self.assert_process_transition(process, 'load_sample')
                robot_op = self.station.get_requested_robot_ops()[0]
                self.assert_robot_op(robot_op, 'KukaLBRTask',
                                        {'name': 'PresentVial',
                                        'params':['False', str(i+1),str(j+1), 'False']})
                self.complete_robot_op(self.station, robot_op)

                # station_process
                self.assert_process_transition(process, 'station_process')
                req_station_ops = process.data.req_station_ops
                station_op = req_station_ops[0]
                self.assertEqual(len(req_station_ops),1)
                self.assert_station_op(station_op, 'SampleColorOpDescriptor')
                self.complete_station_op(self.station)

                 # unload_sample
                self.assert_process_transition(process, 'unload_sample')
                robot_op = self.station.get_requested_robot_ops()[0]
                self.assert_robot_op(robot_op, 'KukaLBRTask',
                                        {'name': 'ReturnVial',
                                        'params':['False', str(i+1),str(j+1), 'False']})
                self.complete_robot_op(self.station, robot_op)

                # update_sample_index
                self.assert_process_transition(process, 'update_sample_index')
                self.assertEqual(process.data.status['batch_index'], i)
                self.assertEqual(process.data.status['sample_index'], j+1)
            
            # update_batch_index
            self.assert_process_transition(process, 'update_batch_index')
            self.assertEqual(process.data.status['batch_index'], i+1)
            self.assertEqual(process.data.status['sample_index'], 0)
        
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
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()