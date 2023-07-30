import unittest
import yaml
import uuid
from mongoengine import connect
from transitions import State
from archemist.core.state.station_process import StationProcess
from archemist.core.state.station import Station, StationProcessData
from archemist.core.state.station_op import StationOpDescriptor, StationOpDescriptorModel
from archemist.core.state.robot_op import RobotTaskOpDescriptor, RobotTaskType
from archemist.core.state.batch import Batch
from archemist.core.state.recipe import Recipe
from archemist.core.util.location import Location

class StationProcessTest(unittest.TestCase):
    
    class TestStationProcess(StationProcess):   
        def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
            
            ''' States '''
            states = [ State(name='init_state'),
                State(name='prep_state', on_enter='initialise_process_data'),
                State(name='load_sample', on_enter=['request_load_sample']),
                State(name='processed_sample_update', on_enter=['update_sample_processing']),
                State(name='load_batch', on_enter=['request_load_batch']),
                State(name='added_batch_update', on_enter=['update_batch_addition']),
                State(name='station_process', on_enter=['request_operation']),
                State(name='unload_sample', on_enter=['request_unload_sample']),
                State(name='unload_batch', on_enter=['request_unload_batch']),
                State(name='removed_batch_update', on_enter=['update_batch_removal']),
                State(name='final_state', on_enter=['finalize_batch_processing'])]
            
            ''' Transitions '''
            transitions = [
                {'source':'init_state','dest':'prep_state'},
                {'source':'prep_state','dest':'load_batch'},
                {'source':'load_batch','dest':'added_batch_update', 'conditions':'are_req_robot_ops_completed'},
                {'source':'added_batch_update','dest':'load_sample'},
                {'source':'load_sample','dest':'station_process', 'conditions':'are_req_robot_ops_completed'},
                {'source':'station_process','dest':'unload_sample', 'conditions':'are_req_station_ops_completed', 'before':'process_sample'},
                {'source':'unload_sample','dest':'processed_sample_update', 'conditions':'are_req_robot_ops_completed'},
                {'source':'processed_sample_update','dest':'load_sample', 'unless':'are_all_samples_processed'},
                {'source':'processed_sample_update','dest':'unload_batch', 'conditions':'are_all_samples_processed'},
                {'source':'unload_batch','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
                {'source':'removed_batch_update','dest':'load_batch', 'unless':'are_all_batches_processed'},
                {'source':'removed_batch_update','dest':'final_state', 'conditions':'are_all_batches_processed'}]

            super().__init__(station, process_data, states, transitions, **kwargs)

        ''' state callbacks '''
        def initialise_process_data(self):
            self._process_data.status['batch_index'] = 0
            self._process_data.status['sample_index'] = 0

        def request_load_sample(self):
            sample_index = self._process_data.status['sample_index']
            batch_index = self._process_data.status['batch_index']
            robot_job = (RobotTaskOpDescriptor.from_args(name="LoadSample", params=[sample_index]))
            current_batch_id = self._process_data.batches[batch_index].id
            self.request_robot_op(robot_job,current_batch_id)

        def update_sample_processing(self):
            self._process_data.status['sample_index'] += 1

        def request_load_batch(self):
            batch_index = self._process_data.status['batch_index']
            robot_job = (RobotTaskOpDescriptor.from_args(name="LoadBatch", 
                                                         type=RobotTaskType.UNLOAD_FROM_ROBOT,
                                                         location=self._station.location, params=[batch_index]))
            current_batch_id = self._process_data.batches[batch_index].id
            self.request_robot_op(robot_job,current_batch_id)

        def update_batch_addition(self):
            batch_index = self._process_data.status['batch_index']
            self._update_batch_loc_to_station(batch_index)

        def request_operation(self):
            op_model = StationOpDescriptorModel(uuid=uuid.uuid4(), associated_station='TestStation',
                                              _type='StationOpDescriptor', _module='archemist.core.state.station_op')
            station_op = StationOpDescriptor(op_model)
            self.request_station_op(station_op)

        def request_unload_batch(self):
            batch_index = self._process_data.status['batch_index']
            robot_job = (RobotTaskOpDescriptor.from_args(name="UnloadBatch", 
                                                         type=RobotTaskType.LOAD_TO_ROBOT,
                                                         location=self._station.location, params=[batch_index]))
            current_batch_id = self._process_data.batches[batch_index].id
            self.request_robot_op(robot_job,current_batch_id)

        def request_unload_sample(self):
            sample_index = self._process_data.status['sample_index']
            batch_index = self._process_data.status['batch_index']
            robot_job = (RobotTaskOpDescriptor.from_args(name="UnloadSample", params=[sample_index]))
            current_batch_id = self._process_data.batches[batch_index].id
            self.request_robot_op(robot_job,current_batch_id)

        def update_batch_removal(self):
            batch_index = self._process_data.status['batch_index']
            self._update_batch_loc_to_robot(batch_index)
            self._process_data.status['batch_index'] += 1
            self._process_data.status['sample_index'] = 0

        def finalize_batch_processing(self):
            self._station.process_assigned_batches()

        '''transition callbacks '''
        def process_sample(self):
            batch_index = self._process_data.status['batch_index']
            last_operation_op_uuid = self._process_data.station_ops_history[-1]
            last_operation_op = self._station.completed_station_ops[last_operation_op_uuid]
            self._process_data.batches[batch_index].add_station_op_to_current_sample(last_operation_op)
            self._process_data.batches[batch_index].process_current_sample()

        def are_all_samples_processed(self):
            sample_index = self._process_data.status['sample_index']
            batch_index = self._process_data.status['batch_index']
            return sample_index == self._process_data.batches[batch_index].num_samples
        
        def are_all_batches_processed(self):
            batch_index = self._process_data.status['batch_index']
            return batch_index == len(self._process_data.batches)
    
    def setUp(self):

        # construct station
        station_dict = {
            'type': 'TestStation',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 1,
            'handler': 'GenericStationHandler',
            'process_batch_capacity': 1,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
        }
        self.station = Station.from_dict(station_dict=station_dict, liquids=[], solids=[])

        # create batches and add them to station
        recipe_doc = dict()
        with open('resources/testing_recipe.yaml') as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        batch1 = Batch.from_arguments(31,2,Location(1,3,'table_frame'))
        recipe1 = Recipe.from_dict(recipe_doc)
        batch1.attach_recipe(recipe1)
        self.station.add_batch(batch1)
        
        # create station process
        process_data = StationProcessData.from_args([batch1])
        self.process = self.TestStationProcess(self.station, process_data)

    def test_process(self):
        # assert initial state
        self.assertEqual(self.process.data.status['state'], 'init_state')
        self.assertEqual(len(self.process.data.req_robot_ops),0)
        self.assertEqual(len(self.process.data.robot_ops_history),0)
        self.assertEqual(len(self.process.data.req_station_ops),0)
        self.assertEqual(len(self.process.data.station_ops_history),0)
        
        # state transitions
        self.process.process_state_transitions()
        self.assertEqual(self.process.data.status['state'], 'prep_state')

        # transition to load_batch
        self.process.process_state_transitions()
        self.assertEqual(self.process.data.status['state'], 'load_batch')
        req_robot_ops = self.process.data.req_robot_ops
        robot_op = req_robot_ops[0]
        self.assertEqual(len(req_robot_ops),1)
        self.assertEqual(robot_op.name, 'LoadBatch')
        self.assertEqual(robot_op.params, ['0'])

        # test no transition
        self.process.process_state_transitions()
        self.assertEqual(self.process.data.status['state'], 'load_batch')

        # complete robot op
        robot_op.add_start_timestamp()
        robot_op.complete_op("test_robot", True)
        self.station.complete_robot_op_request(robot_op)

        # transition to added_batch_update
        self.process.process_state_transitions()
        self.assertEqual(self.process.data.status['state'], 'added_batch_update')
        self.assertEqual(len(self.process.data.req_robot_ops), 0)

        for i in range(2):
            # transition to load sample
            self.assertEqual(self.process.data.status['sample_index'], i)
            self.process.process_state_transitions()
            self.assertEqual(self.process.data.status['state'], 'load_sample')
            req_robot_ops = self.process.data.req_robot_ops
            robot_op = req_robot_ops[0]
            self.assertEqual(len(req_robot_ops),1)
            self.assertEqual(robot_op.name, 'LoadSample')
            self.assertEqual(robot_op.params, [str(i)])

            robot_op.add_start_timestamp()
            robot_op.complete_op("test_robot", True)
            self.station.complete_robot_op_request(robot_op)

            # transition to added_batch_update
            self.process.process_state_transitions()
            self.assertEqual(self.process.data.status['state'], 'station_process')
            self.assertEqual(len(self.process.data.req_robot_ops), 0)
            req_station_ops = self.process.data.req_station_ops
            station_op = req_station_ops[0]
            self.assertEqual(len(req_station_ops),1)
            self.assertTrue(isinstance(station_op, StationOpDescriptor))
            self.station.update_assigned_op()
            self.station.complete_assigned_station_op(True)

            # transition to unload_sample
            self.process.process_state_transitions()
            self.assertEqual(self.process.data.status['state'], 'unload_sample')
            self.assertEqual(len(self.process.data.req_station_ops), 0)
            req_robot_ops = self.process.data.req_robot_ops
            robot_op = req_robot_ops[0]
            self.assertEqual(len(req_robot_ops),1)
            self.assertEqual(robot_op.name, 'UnloadSample')
            self.assertEqual(robot_op.params, [str(i)])

            robot_op.add_start_timestamp()
            robot_op.complete_op("test_robot", True)
            self.station.complete_robot_op_request(robot_op)

            # transition to unload_sample
            self.process.process_state_transitions()
            self.assertEqual(self.process.data.status['state'], 'processed_sample_update')
            self.assertEqual(len(self.process.data.req_robot_ops), 0)

        # transition to unload_batch
        self.process.process_state_transitions()
        self.assertEqual(self.process.data.status['state'], 'unload_batch')
        
        req_robot_ops = self.process.data.req_robot_ops
        robot_op = req_robot_ops[0]
        self.assertEqual(len(req_robot_ops),1)
        self.assertEqual(robot_op.name, 'UnloadBatch')
        self.assertEqual(robot_op.params, ['0'])

        robot_op.add_start_timestamp()
        robot_op.complete_op("test_robot", True)
        self.station.complete_robot_op_request(robot_op)

        # transition to removed_batch_update
        self.process.process_state_transitions()
        self.assertEqual(self.process.data.status['state'], 'removed_batch_update')
        self.assertEqual(len(self.process.data.req_robot_ops), 0)
        self.assertEqual(self.process.data.status['sample_index'], 0)
        self.assertEqual(self.process.data.status['batch_index'], 1)

        # transition to final_state
        self.assertFalse(self.station.has_processed_batch())
        self.process.process_state_transitions()
        self.assertEqual(self.process.data.status['state'], 'final_state')
        self.assertTrue(self.station.has_processed_batch())


if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()
        