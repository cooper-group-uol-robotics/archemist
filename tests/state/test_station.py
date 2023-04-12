import unittest
from archemist.core.state.robot import RobotTaskOpDescriptor, RobotTaskType
from mongoengine import connect
from archemist.core.state.station import Station, StationState, OpState, StationProcessData
from archemist.core.state.station_op import StationOpDescriptor, StationOpDescriptorModel
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.location import Location
import yaml
import uuid

class StationTest(unittest.TestCase):

    def setUp(self):
        station_dict = {
            'type': 'TestStation',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
        }
        self.station: Station = Station.from_dict(station_dict=station_dict, liquids=[], solids=[])
        self.station_object_id = None

    def test_general_members(self):
        self.assertEqual(self.station.id, 23)
        self.station_object_id = self.station.object_id
        self.assertEqual(self.station.state, StationState.INACTIVE)
        self.assertEqual(self.station.batch_capacity, 2)
        self.assertEqual(self.station.selected_handler_dict,
                         {
                            'type': 'GenericStationHandler',
                            'module': 'archemist.core.state'
                         })
        self.assertEqual(self.station.location, Location(1,7,''))

    def test_batch_members(self):
        # assert empty members
        self.assertFalse(self.station.assigned_batches)
        self.assertFalse(self.station.processed_batches)

        # batches creation
        recipe_doc = dict()
        with open('resources/testing_recipe.yaml') as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        batch1 = Batch.from_arguments(31,2,Location(1,3,'table_frame'))
        recipe1 = Recipe.from_dict(recipe_doc)
        batch1.attach_recipe(recipe1)
        batch2 = Batch.from_arguments(32,2,Location(1,3,'table_frame'))
        recipe2 = Recipe.from_dict(recipe_doc)
        batch2.attach_recipe(recipe2)
        batch3 = Batch.from_arguments(33,2,Location(1,3,'table_frame'))
        recipe3 = Recipe.from_dict(recipe_doc)
        batch3.attach_recipe(recipe3)

        # batch assignment
        self.assertTrue(self.station.has_free_batch_capacity())
        self.station.add_batch(batch1)
        self.station.add_batch(batch2)
        self.assertFalse(self.station.has_free_batch_capacity())

        asigned_batches = self.station.assigned_batches
        self.assertEqual(len(asigned_batches),2)
        self.assertEqual(asigned_batches[0].id, batch1.id)
        self.assertEqual(asigned_batches[1].location, batch2.location)

        # all batch processing
        self.station.process_assigned_batches()
        self.assertFalse(self.station.assigned_batches)
        self.assertTrue(self.station.has_processed_batch())
        self.assertEqual(len(self.station.processed_batches),2)

        procssed_batch1 = self.station.get_processed_batch()
        self.assertTrue(procssed_batch1 is not None)
        self.assertEqual(procssed_batch1.id, batch1.id)
        self.assertEqual(procssed_batch1.location, batch1.location)

        procssed_batch2 = self.station.get_processed_batch()
        self.assertTrue(procssed_batch2 is not None)
        self.assertEqual(procssed_batch2.id, batch2.id)
        self.assertEqual(procssed_batch2.location, batch2.location)
        self.assertFalse(self.station.processed_batches)
        self.assertTrue(self.station.get_processed_batch() is None)

        # single batch processing
        self.station.add_batch(batch3)
        self.assertTrue(self.station.has_free_batch_capacity())
        self.station.process_assinged_batch(batch3)
        self.assertEqual(len(self.station.processed_batches),1)
        rocssed_batch3 = self.station.get_processed_batch()
        self.assertTrue(rocssed_batch3 is not None)
        self.assertEqual(rocssed_batch3.id, batch3.id)

    def test_robot_ops_members(self):
        # assert empty members
        self.assertEqual(len(self.station.completed_robot_ops), 0)
        self.assertFalse(self.station.has_requested_robot_ops())
        self.assertEqual(len(self.station.get_requested_robot_ops()), 0)

        # op creation
        robot_op1 = RobotTaskOpDescriptor.from_args('test_task1', params=['False','1'])
        robot_op2 = RobotTaskOpDescriptor.from_args('test_task2', params=['False','2'])
        
        # op assignment
        self.station.request_robot_op(robot_op1, current_batch_id=31)
        self.station.request_robot_op(robot_op2, current_batch_id=32)
        self.assertTrue(self.station.has_requested_robot_ops())
        
        ret_robot_jobs = self.station.get_requested_robot_ops()
        self.assertFalse(self.station.has_requested_robot_ops())
        self.assertEqual(len(ret_robot_jobs), 2)
        self.assertEqual(robot_op1.name, ret_robot_jobs[0].name)
        self.assertEqual(robot_op1.params, ret_robot_jobs[0].params)
        self.assertEqual(ret_robot_jobs[0].origin_station, self.station.object_id)
        self.assertEqual(ret_robot_jobs[0].related_batch_id, 31)
        self.assertEqual(ret_robot_jobs[0].task_type, RobotTaskType.MANIPULATION)
        self.assertEqual(robot_op2.name, ret_robot_jobs[1].name)
        self.assertEqual(robot_op2.params, ret_robot_jobs[1].params)

        self.station.complete_robot_op_request(robot_op1)
        self.assertEqual(len(self.station.completed_robot_ops), 1)
        self.assertTrue(str(robot_op1.uuid) in 
                        self.station.completed_robot_ops.keys())
        self.station.complete_robot_op_request(robot_op2)
        self.assertEqual(len(self.station.completed_robot_ops), 2)
        self.assertTrue(str(robot_op2.uuid) in 
                        self.station.completed_robot_ops.keys())
        
    def test_station_ops_members(self):
        # assert empty members
        self.assertFalse(self.station.has_queued_ops())
        self.assertFalse(self.station.has_assigned_station_op())
        self.assertEqual(self.station.assigned_op_state, OpState.INVALID)
        self.assertEqual(len(self.station.completed_station_ops), 0)

        # op creation
        op_model_1 = StationOpDescriptorModel(uuid=uuid.uuid4(), associated_station='TestStation',
                                              _type='StationOpDescriptor', _module='archemist.core.state.station_op')
        station_op1 = StationOpDescriptor(op_model_1)
        op_model_2 = StationOpDescriptorModel(uuid=uuid.uuid4(), associated_station='TestStation',
                                              _type='StationOpDescriptor', _module='archemist.core.state.station_op')
        station_op2 = StationOpDescriptor(op_model_2)

        # op assignment
        self.station.assign_station_op(station_op1)
        self.station.assign_station_op(station_op2)
        
        self.assertTrue(self.station.has_queued_ops())
        self.assertFalse(self.station.has_assigned_station_op())
        self.assertEqual(self.station.assigned_op_state, OpState.INVALID)

        # process station op1

        self.station.update_assigned_op()
        self.assertTrue(self.station.has_queued_ops())
        self.assertTrue(self.station.has_assigned_station_op())
        self.assertEqual(self.station.assigned_op_state, OpState.ASSIGNED)

        ret_op = self.station.get_assigned_station_op()
        self.assertEqual(ret_op.uuid, station_op1.uuid)
        self.assertIsNone(ret_op.start_timestamp)
        
        self.station.assigned_op_state = OpState.EXECUTING
        self.assertEqual(self.station.assigned_op_state, OpState.EXECUTING)
        
        # complete station op1

        self.station.add_timestamp_to_assigned_op()
        self.station.complete_assigned_station_op(True)
        self.assertFalse(self.station.has_assigned_station_op())
        self.assertEqual(self.station.assigned_op_state, OpState.INVALID)
        self.assertEqual(len(self.station.completed_station_ops), 1)
        
        complete_op = self.station.completed_station_ops[str(station_op1.uuid)]
        self.assertIsNotNone(complete_op.start_timestamp)
        self.assertTrue(complete_op.has_result)
        self.assertTrue(complete_op.was_successful)

        # process station op 2

        self.station.update_assigned_op()
        self.assertFalse(self.station.has_queued_ops())
        self.assertTrue(self.station.has_assigned_station_op())
        self.assertEqual(self.station.assigned_op_state, OpState.ASSIGNED)

        # test repeat and skip op
        self.station.repeat_assigned_op()
        self.assertEqual(self.station.assigned_op_state, OpState.TO_BE_REPEATED)

        self.station.skip_assigned_op()
        self.assertEqual(self.station.assigned_op_state, OpState.TO_BE_SKIPPED)

        # complete station op 2
        self.station.add_timestamp_to_assigned_op()
        self.station.complete_assigned_station_op(True)
        self.assertFalse(self.station.has_assigned_station_op())
        self.assertEqual(self.station.assigned_op_state, OpState.INVALID)
        self.assertEqual(len(self.station.completed_station_ops), 2)
        complete_op = self.station.completed_station_ops[str(station_op2.uuid)]
        self.assertIsNotNone(complete_op.start_timestamp)
        self.assertTrue(complete_op.has_result)
        self.assertTrue(complete_op.was_successful)
        
    def test_process_members(self):
        # assert empty members
        self.assertEqual(len(self.station.get_all_processes_data()),0)

        # process parameters

        process_dict = { 'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}}
        self.assertEqual(self.station.process_sm_dict, 
                         process_dict)
        self.assertEqual(self.station.process_batch_capacity, 2)

        # create batches

        recipe_doc = dict()
        with open('resources/testing_recipe.yaml') as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        batch1 = Batch.from_arguments(31,2,Location(1,3,'table_frame'))
        recipe1 = Recipe.from_dict(recipe_doc)
        batch1.attach_recipe(recipe1)
        batch2 = Batch.from_arguments(32,2,Location(1,3,'table_frame'))
        recipe2 = Recipe.from_dict(recipe_doc)
        batch2.attach_recipe(recipe2)
        batch3 = Batch.from_arguments(33,2,Location(1,3,'table_frame'))
        recipe3 = Recipe.from_dict(recipe_doc)
        batch3.attach_recipe(recipe3)
        batch4 = Batch.from_arguments(34,2,Location(1,3,'table_frame'))
        recipe4 = Recipe.from_dict(recipe_doc)
        batch4.attach_recipe(recipe4)

        # construct process data objects
        process_data_1 = StationProcessData.from_args([batch1, batch2])
        process_data_2= StationProcessData.from_args([batch3, batch4])

        # add prcess data
        self.station.set_process_data(process_data_1)
        self.assertEqual(len(self.station.get_all_processes_data()),1)
        self.station.set_process_data(process_data_2)
        self.assertEqual(len(self.station.get_all_processes_data()),2)

        # get process data 1
        ret_process_data = self.station.get_process_data(process_data_1.uuid)
        self.assertListEqual(process_data_1.batches, ret_process_data.batches)

        # delete process data 1
        self.station.delete_process_data(process_data_1.uuid)
        self.assertEqual(len(self.station.get_all_processes_data()),1)
        self.assertIsNone(self.station.get_process_data(process_data_1.uuid))

        # get process data 2
        ret_process_data = self.station.get_process_data(process_data_2.uuid)
        self.assertListEqual(process_data_2.batches, ret_process_data.batches)

        # delete process data 2
        self.station.delete_process_data(process_data_2.uuid)
        self.assertEqual(len(self.station.get_all_processes_data()),0)
        self.assertIsNone(self.station.get_process_data(process_data_2.uuid))
    

if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()