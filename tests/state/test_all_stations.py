import unittest
from archemist.state.material import Solid,Liquid
from archemist.state.stations.fisher_weighing_station import FisherWeightingStation,FisherWeightOpDescriptor
from archemist.state.stations.ika_place_rct_digital import IKAMode,IkaPlateRCTDigital, IKAStirringOpDescriptor, IKAHeatingOpDescriptor,IKAHeatingStirringOpDescriptor
from archemist.state.stations.chemspeed_flex_station import ChemSpeedStatus,ChemSpeedFlexStation, CSOpenDoorOpDescriptor, CSCloseDoorOpDescriptor, CSCSVJobOpDescriptor
from archemist.state.stations.input_station import InputStation, InputStationPickupOp
from archemist.state.stations.output_station import OutputStation,OutputStationPlaceOp
from archemist.state.stations.light_box_station import LightBoxStation, SampleColorOpDescriptor
from archemist.state.stations.solid_dispensing_quantos_QS2 import QuantosSolidDispenserQS2, QuantosDispenseOpDescriptor
from archemist.state.stations.soluibility_station import SolubilityStation, SolubilityOpDescriptor, TurbidityState
from archemist.state.stations.peristaltic_liquid_dispensing import PeristalticLiquidDispensing, PeristalticPumpOpDescriptor
from archemist.state.station import StationState
from datetime import datetime
from mongoengine import connect

class AllStationsTest(unittest.TestCase):
    def test_fisher_weighing_station(self):
        # construct station
        station_doc = {
            'type': 'FisherWeightingStation',
            'id': 20,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{}
        }
        t_station = FisherWeightingStation.from_dict(station_doc,[],[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        
        # construct station ops
        t_op = FisherWeightOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        self.assertIsNone(t_op.weight)
        
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True, weight=1.2)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(ret_op.weight, 1.2)

    def test_ika_plate_station(self):
        # construct station
        station_doc = {
            'type': 'IkaPlateRCTDigital',
            'id': 21,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{}
        }
        t_station = IkaPlateRCTDigital.from_dict(station_doc,[],[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        
        # construct station ops
        t_op = IKAStirringOpDescriptor.from_args(stirring_speed=500, duration=10)
        self.assertFalse(t_op.has_result)
        
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        self.assertTrue(t_station.mode, IKAMode.STIRRING)
        self.assertTrue(t_station.target_stirring_speed, 500)
        self.assertTrue(t_station.target_duration, 10)
        t_station.finish_station_op(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertIsNone(t_station.mode)
        self.assertIsNone(t_station.target_stirring_speed)
        self.assertIsNone(t_station.target_duration)

        t_op = IKAHeatingOpDescriptor.from_args(temperature=140, duration=10)
        self.assertFalse(t_op.has_result)
        
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        self.assertTrue(t_station.mode, IKAMode.HEATING)
        self.assertTrue(t_station.target_temperature, 140)
        self.assertTrue(t_station.target_duration, 10)
        t_station.finish_station_op(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[1]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertIsNone(t_station.mode)
        self.assertIsNone(t_station.target_temperature)
        self.assertIsNone(t_station.target_duration)

        t_op = IKAHeatingStirringOpDescriptor.from_args(temperature=140, stirring_speed=500, duration=10)
        self.assertFalse(t_op.has_result)
        
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        self.assertTrue(t_station.mode, IKAMode.HEATINGSTIRRING)
        self.assertTrue(t_station.target_temperature, 140)
        self.assertTrue(t_station.target_stirring_speed, 500)
        self.assertTrue(t_station.target_duration, 10)
        t_station.finish_station_op(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[2]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertIsNone(t_station.mode)
        self.assertIsNone(t_station.target_stirring_speed)
        self.assertIsNone(t_station.target_temperature)
        self.assertIsNone(t_station.target_duration)

    def test_chemspeed_station(self):
        # construct station
        station_doc = {
            'type': 'ChemSpeedFlexStation',
            'id': 22,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{}
        }
        t_station = ChemSpeedFlexStation.from_dict(station_doc,[],[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        self.assertIsNone(t_station.status)
        
        # construct station ops
        t_op = CSOpenDoorOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.status, ChemSpeedStatus.DOORS_OPEN)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)

        t_op = CSCloseDoorOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.status, ChemSpeedStatus.DOORS_CLOSED)
        ret_op = t_station.station_op_history[1]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)

        t_op = CSCSVJobOpDescriptor.from_args(csv_string='1,2,3\n4,5,6\n')
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        self.assertTrue(t_station.status, ChemSpeedStatus.RUNNING_JOB)
        t_station.finish_station_op(True, result_file='file.test')
        self.assertEqual(t_station.state, StationState.PROCESSING)
        self.assertTrue(t_station.status, ChemSpeedStatus.JOB_COMPLETE)
        ret_op = t_station.station_op_history[2]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(ret_op.csv_string, '1,2,3\n4,5,6\n')
        self.assertEqual(ret_op.result_file, 'file.test')

    def test_input_station(self):
        # construct station
        station_doc = {
            'type': 'InputStation',
            'id': 23,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{}
        }
        t_station = InputStation.from_dict(station_doc,[],[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        
        # construct station ops
        t_op = InputStationPickupOp.from_args()
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)

    def test_output_station(self):
        # construct station
        station_doc = {
            'type': 'OutputStation',
            'id': 24,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{}
        }
        t_station = OutputStation.from_dict(station_doc,[],[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        
        # construct station ops
        t_op = OutputStationPlaceOp.from_args()
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)

    def test_light_box_station(self):
        # construct station
        station_doc = {
            'type': 'LightBoxStation',
            'id': 25,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{}
        }
        t_station = LightBoxStation.from_dict(station_doc,[],[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        
        # construct station ops
        t_op = SampleColorOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True, result_filename='file.test', red_intensity=255, 
                                    green_intensity=125, blue_intensity=0)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(ret_op.result_filename, 'file.test')
        self.assertEqual(ret_op.red_intensity, 255)
        self.assertEqual(ret_op.green_intensity, 125)
        self.assertEqual(ret_op.blue_intensity, 0)

    def test_quantos_station(self):
        # construct station
        station_doc = {
            'type': 'QuantosSolidDispenserQS2',
            'id': 26,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{
                'catridges': [{'id': 31, 'hotel_index': 1, 'remaining_dosages': 100}]
            }
        }
        solid_dict = {
            'name': 'salt',
            'id': 1235,
            'amount_stored': 2,
            'unit': 'g',
            'dispense_src': 'quantos',
            'cartridge_id': 31,
            'expiry_date': datetime.fromisoformat('2025-02-11')
        }
        solids_list = []
        solids_list.append(Solid.from_dict(solid_dict))
        t_station = QuantosSolidDispenserQS2.from_dict(station_doc,solids=solids_list,liquids=[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        self.assertIsNone(t_station.current_catridge)

        t_station.load_catridge(31)
        current_catridge = t_station.current_catridge
        self.assertEqual(current_catridge.hotel_index, 1)
        self.assertEqual(current_catridge.id, 31)
        self.assertEqual(current_catridge.remaining_dosages, 100)
        self.assertEqual(current_catridge.associated_solid.name, solids_list[0].name)
        self.assertEqual(current_catridge.associated_solid.mass, 2)
        self.assertEqual(current_catridge.associated_solid.mass, solids_list[0].mass)

        # construct station ops
        t_op = QuantosDispenseOpDescriptor.from_args(solid_name='salt',dispense_mass=1.2)
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True, actual_dispensed_mass=1.193)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(ret_op.actual_dispensed_mass, 1.193)
        current_catridge = t_station.current_catridge
        self.assertEqual(current_catridge.remaining_dosages, 99)
        self.assertEqual(current_catridge.associated_solid.mass, 2 - 1.193)
        self.assertEqual(current_catridge.associated_solid.mass, solids_list[0].mass)

        t_station.unload_current_catridge()
        self.assertIsNone(t_station.current_catridge)

    def test_peristaltic_station(self):
        # construct station
        station_doc = {
            'type': 'PeristalticLiquidDispensing',
            'id': 22,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
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
            'expiry_date': datetime.fromisoformat('2025-02-11')
        }
        liquids_list = []
        liquids_list.append(Liquid.from_dict(liquid_dict))
        t_station = PeristalticLiquidDispensing.from_dict(station_dict=station_doc, 
                        liquids=liquids_list, solids=[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        self.assertEqual(t_station.get_liquid(pump_id='pUmP1').name, liquids_list[0].name)
        self.assertEqual(t_station.get_pump_id('water'), 'pUmP1')

        # construct station ops
        t_op = PeristalticPumpOpDescriptor.from_args(liquid_name='water',dispense_volume=100)
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True, actual_dispensed_volume=199)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(ret_op.actual_dispensed_volume, 199)
        self.assertEqual(liquids_list[0].volume, 0.4-0.199)

    def test_soluibility_station(self):
        # construct station
        station_doc = {
            'type': 'SolubilityStation',
            'id': 26,
            'location': {'node_id': 1, 'graph_id': 7},
            'batch_capacity': 2,
            'process_state_machine': 
            {
                'type': 'StationLoadingSm',
                'args': {'batch_mode': True, 'load_frame': '/liquidStation/loadFrame'}
            },
            'parameters':{
                'catridges': [{'id': 31, 'hotel_index': 1, 'remaining_dosages': 100}]
            }
        }
        t_station = SolubilityStation.from_dict(station_doc,[],[])
        self.assertEqual(t_station.id, station_doc['id'])
        self.assertEqual(t_station.state, StationState.IDLE)
        
        # construct station ops
        t_op = SolubilityOpDescriptor.from_args()
        self.assertFalse(t_op.has_result)
        t_station.set_station_op(t_op)
        self.assertEqual(t_station.state, StationState.WAITING_ON_OPERATION)
        t_station.finish_station_op(True, turbidity_state=TurbidityState.DISSOLVED)
        self.assertEqual(t_station.state, StationState.PROCESSING)
        ret_op = t_station.station_op_history[0]
        self.assertTrue(ret_op.has_result)
        self.assertTrue(ret_op.was_successful)
        self.assertEqual(ret_op.turbidity_state, TurbidityState.DISSOLVED)

if __name__ == '__main__':
    connect(db='archemist_test', host='mongodb://localhost:27017', alias='archemist_state')
    unittest.main()





