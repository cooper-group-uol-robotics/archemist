from pickle import NONE
import unittest
from archemist.util import Location
from archemist.state.state import State
from archemist.persistence.persistenceManager import Parser
from archemist.state.batch import Batch
from archemist.state.station import StationState

class SMTestWithBatchMode(unittest.TestCase):
    def test_sm_progress(self):
        parser = Parser()
        state = State()
        state.initializeState(True)
        s1 = state.stations[0]
        sm1 = parser.create_process_sm(s1.__class__.__name__)
        sm1.set_station(s1)
        self.assertEqual(s1.state, StationState.IDLE)
        self.assertTrue(s1.assigned_batch is None)
        self.assertEqual(s1.loaded_samples, 0)
        
        self.assertEqual(sm1.state, 'init_state')
        self.assertFalse(sm1.process_state_transitions())

        
        recipe = parser.loadRecipeYaml()
        bat = Batch(12, recipe, 2, Location(2,7,''))
        s1.add_batch(bat)
        self.assertEqual(s1.state, StationState.PROCESSING)
        self.assertTrue(s1.assigned_batch is not None)
        self.assertEqual(s1.loaded_samples, 0)
        self.assertEqual(sm1.state, 'init_state')
        
        self.assertTrue(sm1.process_state_transitions())
        self.assertEqual(sm1.state, 'load_sample')

        self.assertEqual(s1.state, StationState.PROCESSING)
        self.assertTrue(s1.has_robot_job())
        self.assertEqual(s1.loaded_samples, 1)
        
        job = s1.get_robot_job()
        self.assertEqual(s1.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.sample_index, 1)
        self.assertFalse(sm1.process_state_transitions())
        s1.finish_robot_job()
        self.assertFalse(s1.has_robot_job())
        self.assertEqual(s1.state, StationState.PROCESSING)

        self.assertTrue(sm1.process_state_transitions())
        self.assertEqual(sm1.state, 'load_sample')

        self.assertTrue(s1.has_robot_job())
        self.assertEqual(s1.loaded_samples, 2)

        job = s1.get_robot_job()
        self.assertEqual(s1.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.sample_index, 2)
        self.assertFalse(sm1.process_state_transitions())
        s1.finish_robot_job()
        self.assertFalse(s1.has_robot_job())
        self.assertEqual(s1.state, StationState.PROCESSING)

        self.assertTrue(sm1.process_state_transitions())
        self.assertEqual(sm1.state, 'station_process')
        self.assertEqual(s1.state, StationState.WAITING_ON_OPERATION)
        self.assertFalse(s1.has_robot_job())
        self.assertFalse(sm1.process_state_transitions())
        s1.state = StationState.OPERATION_COMPLETE
        
        self.assertTrue(sm1.process_state_transitions())
        self.assertEqual(sm1.state, 'unload_sample')
        self.assertEqual(s1.state, StationState.PROCESSING)
        self.assertTrue(s1.has_robot_job())
        
        job = s1.get_robot_job()
        self.assertEqual(s1.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.sample_index, 2)
        self.assertFalse(sm1.process_state_transitions())
        s1.finish_robot_job()
        self.assertFalse(s1.has_robot_job())
        self.assertEqual(s1.state, StationState.PROCESSING)

        self.assertTrue(sm1.process_state_transitions())
        self.assertEqual(sm1.state, 'unload_sample')
        self.assertEqual(s1.state, StationState.PROCESSING)
        self.assertTrue(s1.has_robot_job())
        
        job = s1.get_robot_job()
        self.assertEqual(s1.state, StationState.WAITING_ON_ROBOT)
        self.assertEqual(job.sample_index, 1)
        self.assertFalse(sm1.process_state_transitions())
        s1.finish_robot_job()
        self.assertFalse(s1.has_robot_job())
        self.assertEqual(s1.state, StationState.PROCESSING)

        self.assertTrue(sm1.process_state_transitions())
        self.assertEqual(sm1.state, 'init_state')
        self.assertEqual(s1.state, StationState.PROCESSING_COMPLETE)
        self.assertEqual(s1.loaded_samples, 0)
        self.assertTrue(s1.has_processed_batch())
        self.assertTrue(s1.assigned_batch is None)

        print("all done here")

if __name__ == '__main__':
    unittest.main()


