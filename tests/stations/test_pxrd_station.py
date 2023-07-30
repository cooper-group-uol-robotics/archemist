import unittest
from typing import List
from datetime import datetime

import mongoengine

from archemist.core.state.station import StationProcessData
from archemist.stations.pxrd_station.state import PXRDStation, PXRDStatus, PXRDAnalysisOpDescriptor
from archemist.stations.pxrd_station.process import PXRDProcess
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.enums import StationState
from archemist.core.util.location import Location
from utils import ProcessTestingMixin

class PXRDStationTest(unittest.TestCase, ProcessTestingMixin):
    def setUp(self):
        self.station_doc = {
            'type': 'PXRDStation',
            'id': 22,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 1,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 1,
            'process_state_machine': 
            {
                'type': 'PXRDProcess',
                'args': {}
            },
            'parameters': {}
        }

        self.station = PXRDStation.from_dict(self.station_doc,[],[])

    def tearDown(self):
        self.station.model.delete()

    def test_state(self):
        # test station is constructed properly
        self.assertEqual(self.station.id, self.station_doc['id'])
        self.assertEqual(self.station.state, StationState.INACTIVE)

        # test station specific members
        self.assertIsNone(self.station.status)
        self.station.status = PXRDStatus.DOORS_OPEN
        self.assertEqual(self.station.status, PXRDStatus.DOORS_OPEN)
        self.station.status = PXRDStatus.DOORS_CLOSED
        self.assertEqual(self.station.status, PXRDStatus.DOORS_CLOSED)

        
        # test PXRDAnalysisOpDescriptor
        t_op = PXRDAnalysisOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        self.station.assign_station_op(t_op)
        self.assertFalse(self.station.has_assigned_station_op())
        self.station.update_assigned_op()
        self.assertEqual(self.station.status, PXRDStatus.RUNNING_JOB)
        self.assertTrue(self.station.has_assigned_station_op())
        self.station.complete_assigned_station_op(True, result_file='result.file')
        self.assertEqual(self.station.status, PXRDStatus.JOB_COMPLETE)
        complete_op = self.station.completed_station_ops.get(str(t_op.uuid))
        self.assertEqual(complete_op.result_file, "result.file")

        # reset station status in prep for process test
        self.station.status = None

    def test_pxrd_process(self):
        # construct batches
        recipe_doc = {
            'general': 
            {
                'name': 'pxrd_test_recipe',
                'id': 111
            },
            'process':
            [
                {
                    'state_name': 'analyse',
                    'station':
                    {
                        'type': 'PXRDStation',
                        'id': 22,
                        'operation':
                            {
                                'type': 'PXRDAnalysisOpDescriptor',
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
        
        # add batches to station
        self.station.add_batch(batch_1)

        # create station process
        process_data = StationProcessData.from_args([batch_1])
        process = PXRDProcess(self.station, process_data)

        # assert initial state
        self.assertEqual(process.data.status['state'], 'init_state')
        self.assertEqual(len(process.data.req_robot_ops),0)
        self.assertEqual(len(process.data.robot_ops_history),0)
        self.assertEqual(len(process.data.req_station_ops),0)
        self.assertEqual(len(process.data.station_ops_history),0)

        # prep_state
        self.assert_process_transition(process, 'prep_state')
        self.assertEqual(process.data.status['batch_index'], 0)
        self.assertFalse(process.data.status['operation_complete'])

        # open_pxrd_door
        self.assert_process_transition(process, 'open_pxrd_door')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                                {'name': 'OpenDoors',
                                'params':['True']})
        self.complete_robot_op(self.station, robot_op)

        # load_pxrd
        self.assert_process_transition(process, 'load_pxrd')
        self.assertEqual(self.station.status, PXRDStatus.DOORS_OPEN)
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                            {'name': 'LoadPXRD',
                            'params':['True']})
        self.complete_robot_op(self.station, robot_op)

        # added_batch_update
        self.assert_process_transition(process, 'added_batch_update')

        # close_pxrd_door
        self.assert_process_transition(process, 'close_pxrd_door')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                                {'name': 'CloseDoors',
                                'params':['True']})
        self.complete_robot_op(self.station, robot_op)

        # pxrd_process
        self.assert_process_transition(process, 'pxrd_process')
        station_op = process.data.req_station_ops[0]
        self.assert_station_op(station_op, 'PXRDAnalysisOpDescriptor')
        self.complete_station_op(self.station)
        self.assertEqual(self.station.status, PXRDStatus.JOB_COMPLETE)

         # open_pxrd_door
        self.assert_process_transition(process, 'open_pxrd_door')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                                {'name': 'OpenDoors',
                                'params':['True']})
        self.complete_robot_op(self.station, robot_op)

        # unload_pxrd
        self.assert_process_transition(process, 'unload_pxrd')
        self.assertEqual(self.station.status, PXRDStatus.DOORS_OPEN)
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                            {'name': 'UnloadPXRD',
                            'params':['True']})
        self.complete_robot_op(self.station, robot_op)

        # removed_batch_update
        self.assert_process_transition(process, 'removed_batch_update')

        # close_pxrd_door
        self.assert_process_transition(process, 'close_pxrd_door')
        robot_op = self.station.get_requested_robot_ops()[0]
        self.assert_robot_op(robot_op, 'KukaLBRTask',
                                {'name': 'CloseDoors',
                                'params':['True']})
        self.complete_robot_op(self.station, robot_op)

        # final_state
        self.assertFalse(self.station.has_processed_batch())
        self.assert_process_transition(process, 'final_state')
        self.assertTrue(self.station.has_processed_batch())


if __name__ == '__main__':
    mongoengine.connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()
