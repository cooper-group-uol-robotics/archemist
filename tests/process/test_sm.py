import unittest
from archemist.util import Location
from archemist.state.material import Liquid
from archemist.state.stations.ika_place_rct_digital import IkaPlateRCTDigital
from archemist.persistence.object_factory import StationFactory
from archemist.state.batch import Batch
from archemist.state.station import StationState
from datetime import datetime
from mongoengine import connect
import yaml

class SMTestWithBatchMode(unittest.TestCase):
    def test_sm_progress(self):
        station_dict = {
            'type': 'IkaPlateRCTDigital',
            'id': 2,
            'location': {'node_id': 2, 'graph_id': 7},
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {
                    'batch_mode': False, 
                    'batch_load_task': 'LoadBatch',
                    'batch_unload_task': 'UnLoadBatch',
                    'sample_load_task': 'LoadSample',
                    'sample_unload_task': 'UnLoadSample'
                    }
            },
            'batch_capacity': 1,
            'parameters':{}
        }
        liquid_dict = {
            'name': 'water',
            'id': 1235,
            'amount_stored': 400,
            'unit': 'ml',
            'density': 997,
            'pump_id': 'pUmP1',
            'expiry_date': datetime.fromisoformat('2025-02-11')
        }
        liquids_list = []
        liquids_list.append(Liquid.from_dict(liquid_dict))
        t_station = IkaPlateRCTDigital.from_dict(station_dict=station_dict, 
                        liquids=liquids_list, solids=[])

        t_sm = StationFactory.create_state_machine(t_station)
        
        self.assertEqual(t_station.state, StationState.IDLE)
        self.assertEqual(t_station.assigned_batches,[])
        
        self.assertEqual(t_sm.state, 'init_state')
        self.assertFalse(t_sm.process_state_transitions())

        recipe_doc = dict()
        with open('/home/gilgamish/robot_chemist_ws/src/archemist/tests/state/resources/testing_recipe.yaml') as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        
        bat = Batch.from_arguments(31,2,Location(2,7,'table_frame'))
        bat.attach_recipe(recipe_doc)
        t_station.add_batch(bat)
        bat.recipe.advance_state(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertEqual(len(t_station.assigned_batches), 1)
        self.assertEqual(t_sm.state, 'init_state')
        
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'load_batch')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_requested_robot_op())
        
        job = t_station.get_requested_robot_op()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'LoadBatch')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'added_batch_update')
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'load_sample')

        self.assertTrue(t_station.has_requested_robot_op())

        job = t_station.get_requested_robot_op()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'LoadSample')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)
        
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'station_process')
        self.assertEqual(t_station.state, StationState.OP_ASSIGNED)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_assigned_station_op(True)
        t_station.set_to_processing()
        
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'unload_sample')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_requested_robot_op())
        
        job = t_station.get_requested_robot_op()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'UnLoadSample')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'added_sample_update')
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'load_sample')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_requested_robot_op())
        
        job = t_station.get_requested_robot_op()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'LoadSample')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'station_process')
        self.assertEqual(t_station.state, StationState.OP_ASSIGNED)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_assigned_station_op(True)
        t_station.set_to_processing()
        
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'unload_sample')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_requested_robot_op())
        
        job = t_station.get_requested_robot_op()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'UnLoadSample')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'added_sample_update')
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'unload_batch')

        job = t_station.get_requested_robot_op()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'UnLoadBatch')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'removed_batch_update')
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'init_state')

if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()


