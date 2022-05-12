import unittest
from archemist.exceptions.exception import StationAssignedRackError, StationNoOutcomeError, StationUnAssignedRackError
from archemist.state.material import Liquid
from archemist.state.robot import MoveSampleOp, RobotOutputDescriptor

from archemist.state.station import Station, StationState
from archemist.state.stations.peristaltic_liquid_dispensing import PeristalticLiquidDispensing, PeristalticPumpOpDescriptor, PeristalticPumpOutputDescriptor
from archemist.state.batch import Batch
from archemist.util.location import Location
import yaml
from datetime import date

class StationTest(unittest.TestCase):

    def test_station(self):
        station_dict = {
            'class': 'PeristalticLiquidDispensing',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':
            {
                'liquid_pump_map': {'water': 'pUmP1'}
            }
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
        t_station = PeristalticLiquidDispensing.from_dict(db_name='test', station_dict=station_dict, 
                        liquids=liquids_list, solids=[])

        # general properties
        self.assertEqual(t_station.id, 23)
        self.assertEqual(t_station.state, StationState.IDLE)
        self.assertEqual(t_station.operational, True)
        self.assertEqual(t_station.location, Location(1,7,''))
        self.assertEqual(t_station.process_sm_dict, station_dict['process_state_machine'])

        # Loaded samples
        self.assertEqual(t_station.loaded_samples, 0)
        t_station.load_sample()
        t_station.load_sample()
        self.assertEqual(t_station.loaded_samples, 2)
        t_station.unload_sample()
        t_station.unload_sample()
        self.assertEqual(t_station.loaded_samples, 0)

        # Batch
        self.assertTrue(t_station.assigned_batch is None)
        
        recipe_doc = dict()
        with open('resources/testing_recipe.yaml') as fs:
            recipe_doc = yaml.load(fs, Loader=yaml.SafeLoader)
        batch = Batch.from_arguments('test',31,2,Location(1,3,'table_frame'))
        batch.attach_recipe(recipe_doc)
        t_station.add_batch(batch)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        with self.assertRaises(StationAssignedRackError):
            t_station.add_batch(batch)
        asigned_batch = t_station.assigned_batch
        self.assertTrue(asigned_batch is not None)
        self.assertEqual(asigned_batch.id, batch.id)
        self.assertEqual(asigned_batch.location, batch.location)
        self.assertFalse(t_station.has_processed_batch())

        t_station.process_assigned_batch()
        self.assertTrue(t_station.assigned_batch is None)
        self.assertTrue(t_station.has_processed_batch())
        self.assertEqual(t_station.state, StationState.PROCESSING_COMPLETE)

        procssed_batch = t_station.get_processed_batch()
        self.assertTrue(procssed_batch is not None)
        self.assertEqual(procssed_batch.id, batch.id)
        self.assertEqual(procssed_batch.location, batch.location)
        self.assertEqual(t_station.state, StationState.IDLE)
        self.assertTrue(t_station.get_processed_batch() is None)

        # Robot Job
        self.assertFalse(t_station.has_robot_job())
        self.assertTrue(t_station.get_robot_job() is None)

        robot_op = MoveSampleOp(1, Location(1,7,'frame_A'), Location(1,7,'frame_B'), RobotOutputDescriptor())
        t_station.set_robot_job(robot_op)
        self.assertTrue(t_station.has_robot_job())
        ret_robot_job, station_obj_id = t_station.get_robot_job()
        self.assertTrue(ret_robot_job is not None)
        self.assertEqual(robot_op.sample_index, ret_robot_job.sample_index)
        self.assertEqual(robot_op.start_location, ret_robot_job.start_location)
        self.assertEqual(robot_op.target_location, ret_robot_job.target_location)
        self.assertEqual(t_station.state, StationState.WAITING_ON_ROBOT)

        t_station.finish_robot_job(robot_op)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertFalse(t_station.has_robot_job())

        # Station op
        self.assertEqual(len(t_station.station_op_history), 0)
        self.assertFalse(t_station.has_station_op())
        self.assertEqual(t_station.state,StationState.PROCESSING)
        station_op1 = PeristalticPumpOpDescriptor({'liquid': 'water', 'volume': 0.01},PeristalticPumpOutputDescriptor())
        t_station.set_station_op(station_op1)
        self.assertTrue(t_station.has_station_op())
        self.assertEqual(t_station.state,StationState.WAITING_ON_OPERATION)
        
        current_op = t_station.get_station_op()
        self.assertEqual(current_op.liquid_name, station_op1.liquid_name)
        self.assertEqual(current_op.dispense_volume, station_op1.dispense_volume)
        t_station.finish_station_op(current_op)
        self.assertFalse(t_station.has_station_op())
        self.assertEqual(t_station.state,StationState.PROCESSING)
        station_history = t_station.station_op_history
        self.assertEqual(len(station_history), 1)
        

        station_op2 = PeristalticPumpOpDescriptor({'liquid': 'water', 'volume': 0.005},PeristalticPumpOutputDescriptor())
        t_station.set_station_op(station_op2)
        self.assertTrue(t_station.has_station_op())
        t_station.finish_station_op(station_op2)

        station_history = t_station.station_op_history
        self.assertEqual(len(station_history), 2)
        self.assertEqual(station_history[-1].liquid_name, station_op2.liquid_name)
        self.assertEqual(station_history[-1].dispense_volume, station_op2.dispense_volume)

    def test_specific_station(self):
        station_dict = {
            'class': 'PeristalticLiquidDispensing',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':
            {
                'liquid_pump_map': {'water': 'pUmP1'}
            }
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
        t_station = PeristalticLiquidDispensing.from_dict(db_name='test', station_dict=station_dict, 
                        liquids=liquids_list, solids=[])

        liquid = t_station.get_liquid('pUmP1')
        self.assertEqual(liquid.name, 'water')
        self.assertEqual(liquid.id, 1235)
        self.assertEqual(liquid.volume, 0.4)

        pump_id = t_station.get_pump_id('water')
        self.assertEqual(pump_id, 'pUmP1')
        t_station.add_liquid('water', 0.05)
        self.assertEqual(t_station.get_liquid('pUmP1').volume, 0.40005)
        t_station.dispense_liquid('water', 0.1)
        self.assertEqual(t_station.get_liquid('pUmP1').volume, 0.39995)
        



if __name__ == '__main__':
    unittest.main()