import unittest

import mongoengine

from archemist.core.state.station import StationProcessData
from archemist.stations.shaker_plate_station.state import (ShakerPlateStation,ShakeOpDescriptor,
                                                           ShakerStatus)
from archemist.stations.shaker_plate_station.process import YumiShakerPlateProcess
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import StationState
from archemist.core.util.location import Location
from utils import ProcessTestingMixin

class ShakerPlateStationTest(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'ShakerPlateStation',
            'id': 20,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 1,
            'process_batch_capacity': 1,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'YumiShakerPlateProcess',
                'args': {}
            },
            'parameters':{}
        }

        self.station = ShakerPlateStation.from_dict(self.station_doc,[],[])

    def tearDown(self):
        self.station.model.delete()
    
    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)
        self.assertEqual(self.station.status, ShakerStatus.NOT_SHAKING)

        # test ShakeOpDescriptor
        t_op = ShakeOpDescriptor.from_args(duration=10)
        self.assertFalse(t_op.has_result)
        self.assertEqual(t_op.duration, 10)
        
        self.station.assign_station_op(t_op)
        self.station.update_assigned_op()
        self.assertEqual(self.station.status, ShakerStatus.SHAKING)
        
        self.station.complete_assigned_station_op(True)
        self.assertEqual(self.station.status, ShakerStatus.NOT_SHAKING)
        ret_op = self.station.completed_station_ops[str(t_op.uuid)]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)

    def test_yumi_shaker_plate_process(self):
        # construct batches
        recipe_doc = {
            'general': 
            {
                'name': 'shaker_plate_test_recipe',
                'id': 112
            },
            'process':
            [
                {
                    'state_name': 'shake_samples',
                    'station':
                    {
                        'type': 'ShakerPlateStation',
                        'id': 20,
                        'operation':
                            {
                                'type': 'ShakeOpDescriptor',
                                'properties': 
                                    {
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
        process = YumiShakerPlateProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')
        self.assertEqual(process.data.status['batch_index'], 0)

        # load_shaker_plate
        self.assert_process_transition(process, 'load_shaker_plate')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'YuMiRobotTask',
                        {'name': 'invertAndLoadShakerPlate'})
        self.complete_robot_op(self.station, robot_op)

        # shake
        self.assert_process_transition(process, 'shake')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'ShakeOpDescriptor',
                               {'duration': 10})
        self.complete_station_op(self.station)

        # unload_shaker_plate
        self.assert_process_transition(process, 'unload_shaker_plate')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'YuMiRobotTask',
                        {'name': 'invertAndLoadWellPlate'})
        self.complete_robot_op(self.station, robot_op)

        # unscrew_caps
        self.assert_process_transition(process, 'unscrew_caps')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'YuMiRobotTask',
                        {'name': 'unscrewCaps'})
        self.complete_robot_op(self.station, robot_op)
        
        # pick_pxrd_rack
        self.assert_process_transition(process, 'pick_pxrd_rack')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                        {'name': 'UnloadPXRDRackYumiStation',
                        'params':['True']})
        self.complete_robot_op(self.station, robot_op)

        # pick_eightw_rack
        self.assert_process_transition(process, 'pick_eightw_rack')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                        {'name': 'UnloadEightWRackYumiStation',
                        'params':['False']})
        self.complete_robot_op(self.station, robot_op)

        # removed_batch_update
        self.assert_process_transition(process, 'removed_batch_update')

        # final_state
        self.assertFalse(self.station.has_processed_batch())
        self.assert_process_transition(process, 'final_state')
        self.assertTrue(self.station.has_processed_batch())

if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()