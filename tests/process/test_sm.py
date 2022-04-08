import unittest
from archemist.util import Location
from archemist.state.material import Liquid
from archemist.state.stations import IkaPlateRCTDigital
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.state.batch import Batch
from archemist.state.station import StationState
from datetime import date
import yaml

class SMTestWithBatchMode(unittest.TestCase):
    def test_sm_progress(self):
        station_dict = {
            'class': 'IkaPlateRCTDigital',
            'id': 2,
            'location': {'node_id': 2, 'graph_id': 7},
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{}
        }
        liquid_dict = {
            'name': 'water',
            'id': 1235,
            'amount_stored': 400,
            'unit': 'ml',
            'density': 997,
            'pump_id': 'pUmP1',
            'expiry_date': date.fromisoformat('2025-02-11')
        }
        liquids_list = []
        liquids_list.append(Liquid('test', liquid_dict))
        t_station = IkaPlateRCTDigital.from_dict(db_name='test', station_dict=station_dict, 
                        liquids=liquids_list, solids=[])

        t_sm = ObjectConstructor.construct_process_sm_for_station(t_station)
        
        self.assertEqual(t_station.state, StationState.IDLE)
        self.assertTrue(t_station.assigned_batch is None)
        self.assertEqual(t_station.loaded_samples, 0)
        
        self.assertEqual(t_sm.state, 'init_state')
        self.assertFalse(t_sm.process_state_transitions())

        recipe_doc = dict()
        with open('/home/gilgamish/robot_chemist_ws/src/archemist/tests/state/resources/testing_recipe.yaml') as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        
        bat = Batch.from_arguments('test',31,recipe_doc,2,Location(2,7,'table_frame'))
        t_station.add_batch(bat)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.assigned_batch is not None)
        self.assertEqual(t_station.loaded_samples, 0)
        self.assertEqual(t_sm.state, 'init_state')
        
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'load_sample')

        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_robot_job())
        self.assertEqual(t_station.loaded_samples, 1)
        
        job = t_station.get_robot_job()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.sample_index, 1)
        self.assertFalse(t_sm.process_state_transitions())
        t_station.finish_robot_job(job)
        self.assertFalse(t_station.has_robot_job())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'load_sample')

        self.assertTrue(t_station.has_robot_job())
        self.assertEqual(t_station.loaded_samples, 2)

        job = t_station.get_robot_job()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.sample_index, 2)
        self.assertFalse(t_sm.process_state_transitions())
        t_station.finish_robot_job(job)
        self.assertFalse(t_station.has_robot_job())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'station_process')
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        self.assertFalse(t_station.has_robot_job())
        self.assertFalse(t_sm.process_state_transitions())
        t_station.finish_station_operation()
        
        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'unload_sample')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_robot_job())
        
        job = t_station.get_robot_job()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.sample_index, 2)
        self.assertFalse(t_sm.process_state_transitions())
        t_station.finish_robot_job(job)
        self.assertFalse(t_station.has_robot_job())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'unload_sample')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.has_robot_job())
        
        job = t_station.get_robot_job()
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.sample_index, 1)
        self.assertFalse(t_sm.process_state_transitions())
        t_station.finish_robot_job(job)
        self.assertFalse(t_station.has_robot_job())
        self.assertEqual(t_station.state, StationState.PROCESSING)

        self.assertTrue(t_sm.process_state_transitions())
        self.assertEqual(t_sm.state, 'init_state')
        self.assertEqual(t_station.state, StationState.PROCESSING_COMPLETE)
        self.assertEqual(t_station.loaded_samples, 0)
        self.assertTrue(t_station.has_processed_batch())
        self.assertTrue(t_station.assigned_batch is None)

        print("all done here")

if __name__ == '__main__':
    unittest.main()


