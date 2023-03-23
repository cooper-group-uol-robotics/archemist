import unittest
from archemist.core.util import Location
from archemist.core.state.material import Liquid
from archemist.stations.weighing_station.state import WeighingStation
from archemist.core.state.recipe import Recipe
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.state.batch import Batch
from archemist.core.state.station import StationState
from datetime import datetime
from mongoengine import connect
import yaml

class SMTestWithBatchMode(unittest.TestCase):
    def test_sm_progress(self):
        station_dict = {
            'type': 'WeighingStation',
            'id': 2,
            'location': {'node_id': 2, 'graph_id': 7},
            'process_state_machine': 
            {
                'type': 'WeighingSM',
                'args': {
                    }
            },
            'batch_capacity': 1,
            'handler': 'GenericStationHandler',
            'parameters':{}
        }


        t_station = WeighingStation.from_dict(station_dict=station_dict, 
                        liquids=[], solids=[])

        t_sm = StationFactory.create_state_machine(t_station)
        
        self.assertEqual(t_station.state, StationState.IDLE)
        self.assertEqual(t_station.assigned_batches,[])
        
        self.assertEqual(t_sm.state, 'init_state')
        self.assertFalse(t_sm.process_state_transitions())

        recipe_doc = dict()  
        with open('/home/satheesh/ARChemeist_ws/archemist/tests/state/resources/testing_recipe_weighing.yaml') as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        
        bat = Batch.from_arguments(1,1,Location(2,7,'table_frame'))
        recipe = Recipe.from_dict(recipe_doc)
        bat.attach_recipe(recipe)
        t_station.add_batch(bat)
        bat.recipe.advance_state(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertEqual(len(t_station.assigned_batches), 1)
        self.assertEqual(t_sm.state, 'init_state')
        
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'disable_auto_functions')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_requested_robot_op())

        job = t_station.get_requested_robot_op()

    
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'DiableAutoFunctions')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())

        self.assertEqual(t_sm.state, 'load_sample')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_requested_robot_op())
        
        job = t_station.get_requested_robot_op()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'PresentVial')
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
        self.assertEqual(job.name, 'ReturnVial')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'update_batch_index')
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'enable_auto_functions')
       
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_station.has_requested_robot_op())

        job = t_station.get_requested_robot_op()

    
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.name, 'EnableAutoFunctions')
        self.assertFalse(t_sm.process_state_transitions())
        t_station.complete_robot_op_request(job)
        self.assertFalse(t_station.has_requested_robot_op())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'init_state')

if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()


