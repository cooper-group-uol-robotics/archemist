import unittest

from mongoengine import connect
from bson.objectid import ObjectId
from time import sleep

from archemist.stations.mt_synthesis_station.state import (MTSynthesisStation,
                                                           SynthesisCartridge,
                                                           MTSynthDispenseSolidOp,
                                                           MTSynthDispenseLiquidOp,
                                                           MTSynthStartLiquidDispensingOp,
                                                           MTSynthStopLiquidDispensingOp,
                                                           MTSynthAddWashLiquidOp,
                                                           MTSynthSampleOp,
                                                           MTSynthStopReactionOp,
                                                           MTSynthOpenReactionValveOp,
                                                           MTSynthCloseReactionValveOp,
                                                           MTSynthTimedOpenReactionValveOp,
                                                           MTSynthFilterOp,
                                                           MTSynthDryOp,
                                                           MTSynthDrainOp,
                                                           OptiMaxMode,
                                                           MTSynthHeatStirOp)
from archemist.stations.mt_synthesis_station.process import MTAPCSynthProcess, MTAPCCleanProcess, MTAPCFilterProcess
from archemist.stations.mt_synthesis_station.handler import SimMTSynthesisStationHandler
from archemist.core.state.robot_op import RobotTaskOp
from archemist.core.util.enums import StationState
from archemist.core.state.station_op_result import MaterialOpResult
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import OpOutcome, ProcessStatus
from datetime import date
from .testing_utils import test_req_robot_ops, test_req_station_op, test_req_station_proc

class MTSynthesisStationTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
            'type': 'MTSynthesisStation',
            'id': 21,
            'location': {'coordinates': [1,7], 'descriptor': "MTSynthesisStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': {
                'cartridges': [
                    {'associated_solid': "NaCl", 'hotel_index': 1},
                    {'associated_solid': "NaCl", 'hotel_index': 2}
                    ],
                'num_sampling_vials': 12
            },
            'materials':
            {
                'liquids':
                [{
                    'name': 'H2O',
                    'amount': 400,
                    'unit': 'mL',
                    'density': 997,
                    'density_unit': "kg/m3",
                    "details": {"dispense_method": "diaphragm_pump"},
                    'expiry_date': date.fromisoformat('2025-02-11')
                },
                {
                    'name': 'C4O3H6',
                    'amount': 400,
                    'unit': 'mL',
                    'density': 997,
                    'density_unit': "kg/m3",
                    "details": {"dispense_method": "syringe_pump", "in_port": 1, "out_port": 2},
                    'expiry_date': date.fromisoformat('2025-02-11')
                }],
                'solids': [{
                    'name': 'NaCl',
                    'amount': 100,
                    'unit': 'mg',
                    'expiry_date': date.fromisoformat('2025-02-11'),
                    'details': None
                }]
            }
        }

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_cartrdige(self):
        cartridge_dict = {'associated_solid': "NaCl", 'hotel_index': 1}
        cartridge = SynthesisCartridge.from_dict(cartridge_dict)
        self.assertIsNotNone(cartridge)
        self.assertEqual(cartridge.associated_solid, "NaCl")
        self.assertEqual(cartridge.hotel_index, 1)
        self.assertFalse(cartridge.depleted)
        cartridge.depleted = True
        self.assertTrue(cartridge.depleted)

    def test_station_state(self):
        
        station = MTSynthesisStation.from_dict(self.station_doc)
        # test station is constructed properly
        self.assertIsNotNone(station)
        self.assertEqual(station.state, StationState.INACTIVE)

        # test station specific methods
        self.assertIsNone(station.optimax_mode)
        self.assertEqual(len(station.cartridges), 2)
        self.assertEqual(station.cartridges[0].associated_solid, "NaCl")

        self.assertIsNone(station.loaded_cartridge)
        self.assertFalse(station.window_open)
        station.window_open = True
        self.assertTrue(station.window_open)
        self.assertFalse(station.optimax_valve_open)
        self.assertEqual(station.num_sampling_vials, 12)
        self.assertIsNone(station.set_reaction_temperature)
        self.assertIsNone(station.set_stirring_speed)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        station.add_lot(lot)

        # test load cartridge
        station.load_cartridge(0)
        self.assertEqual(station.loaded_cartridge.hotel_index, 1)
        self.assertFalse(station.loaded_cartridge.depleted)

        # test MTSynthDispenseSolidOp
        t_op = MTSynthDispenseSolidOp.from_args(target_sample=batch.samples[0],
                                                solid_name="NaCl",
                                                dispense_mass=15,
                                                dispense_unit="mg")
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(station.solids_dict["NaCl"].mass, 85)
        self.assertTrue(station.loaded_cartridge.depleted)

        # test unload_cartridge 
        station.unload_cartridge()
        self.assertIsNone(station.loaded_cartridge)

        # test MTSynthDispenseLiquidOp
        t_op = MTSynthDispenseLiquidOp.from_args(target_sample=batch.samples[0],
                                                 liquid_name="H2O",
                                                 dispense_volume=50,
                                                 dispense_unit="mL")
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(station.liquids_dict["H2O"].volume, 350)

        # test MTSynthStartLiquidDispensingOp
        t_op = MTSynthStartLiquidDispensingOp.from_args(liquid_name="H2O",
                                                        dispense_rate=1.5,
                                                        rate_unit="mL/minute",
                                                        max_dispense_volume=10,
                                                        dispense_unit="mL")
        self.assertEqual(t_op.dispense_rate, 1.5)
        self.assertEqual(t_op.rate_unit, "mL/minute")
        self.assertEqual(t_op.max_dispense_volume, 10)
        self.assertEqual(t_op.dispense_unit, "mL")
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)

        # test MTSynthStopLiquidDispensingOp
        t_op = MTSynthStopLiquidDispensingOp.from_args(target_sample=batch.samples[0],
                                                 liquid_name="H2O")
        self.assertIsNone(t_op.dispensed_volume)
        self.assertIsNone(t_op.dispense_unit)
        station.add_station_op(t_op)
        station.update_assigned_op()

        op_result = MaterialOpResult.from_args(origin_op=t_op.object_id,
                                               material_names=["H2O"],
                                               amounts=[5],
                                               units=["mL"])

        station.complete_assigned_op(OpOutcome.SUCCEEDED, [op_result])
        self.assertEqual(t_op.dispensed_volume, 5)
        self.assertEqual(t_op.dispense_unit, "mL")
        self.assertEqual(station.liquids_dict["H2O"].volume, 345)

        # test MTSynthAddWashLiquidOp
        t_op = MTSynthAddWashLiquidOp.from_args(liquid_name="H2O",
                                                 dispense_volume=50,
                                                 dispense_unit="mL")
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(station.liquids_dict["H2O"].volume, 295)

        # test MTSynthHeatStirOp heating and stirring
        t_op = MTSynthHeatStirOp.from_args(target_sample=batch.samples[0],
                                            target_temperature=100,
                                            target_stirring_speed=50,
                                            wait_duration=3,
                                            time_unit="minute")
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_temperature, 100)
        self.assertEqual(t_op.target_stirring_speed, 50)
        self.assertEqual(t_op.wait_duration, 3)
        self.assertEqual(t_op.time_unit, "minute")

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING_STIRRING)
        self.assertEqual(station.set_reaction_temperature, 100)
        self.assertEqual(station.set_stirring_speed, 50)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)
        self.assertEqual(station.set_reaction_temperature, 100)
        self.assertEqual(station.set_stirring_speed, 50)

        # test MTSynthHeatStirOp heating
        t_op = MTSynthHeatStirOp.from_args(target_sample=batch.samples[0],
                                            target_temperature=103,
                                            target_stirring_speed=None,
                                            wait_duration=None,
                                            time_unit=None)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_temperature, 103)
        self.assertIsNone(t_op.target_stirring_speed)
        self.assertIsNone(t_op.wait_duration)
        self.assertIsNone(t_op.time_unit)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING)
        self.assertEqual(station.set_reaction_temperature, 103)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)

        # test MTSynthHeatStirOp stirring
        t_op = MTSynthHeatStirOp.from_args(target_sample=batch.samples[0],
                                            target_temperature=None,
                                            target_stirring_speed=55,
                                            wait_duration=None,
                                            time_unit=None)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_stirring_speed, 55)
        self.assertIsNone(t_op.target_temperature)
        self.assertIsNone(t_op.wait_duration)
        self.assertIsNone(t_op.time_unit)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.STIRRING)
        self.assertEqual(station.set_stirring_speed, 55)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)

        # test MTSynthSampleOp heating
        t_op = MTSynthSampleOp.from_args(target_sample=batch.samples[0],
                                            target_temperature=99,
                                            target_stirring_speed=None)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_temperature, 99)
        self.assertIsNone(t_op.target_stirring_speed)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING)
        self.assertEqual(station.set_reaction_temperature, 99)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(station.num_sampling_vials, 11)
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING)

        # test MTSynthStopReactionOp
        t_op = MTSynthStopReactionOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)

        # test MTSynthOpenReactionValveOp
        t_op = MTSynthOpenReactionValveOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(station.optimax_valve_open)

        # test MTSynthCloseReactionValveOp
        t_op = MTSynthCloseReactionValveOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(station.optimax_valve_open)

        # test MTSynthTimedOpenReactionValveOp
        t_op = MTSynthTimedOpenReactionValveOp.from_args(duration=1.5,
                                                         time_unit="second")
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertTrue(station.optimax_valve_open)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(station.optimax_valve_open)

        # test MTSynthFilterOp
        t_op = MTSynthFilterOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)

        # test MTSynthDryOp
        t_op = MTSynthDryOp.from_args(duration=5,
                                      time_unit="minute")
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.duration, 5)
        self.assertEqual(t_op.time_unit, "minute")

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)

        # test MTSynthDrainOp
        t_op = MTSynthDrainOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)

    def test_synthesis_process(self):

        # construct station
        station = MTSynthesisStation.from_dict(self.station_doc)
        
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        
        # add batches to station
        station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "add_liquid_op_1",
                    "op": "MTSynthDispenseLiquidOp",
                    "parameters": {
                        "liquid_name": "H2O",
                        "dispense_volume": 50,
                        "dispense_unit": "mL"                    
                    }
                },
                {
                    "name": "add_liquid_op_2",
                    "op": "MTSynthStartLiquidDispensingOp",
                    "parameters": {
                        "liquid_name": "C4O3H6",
                        "dispense_rate": 5,
                        "rate_unit": "mL/minute",
                        "max_dispense_volume": 50,
                        "dispense_unit": "mL"                    
                    }
                },
                {
                    "name": "add_solid_op",
                    "op": "MTSynthDispenseSolidOp",
                    "parameters": {
                        "solid_name": "NaCl",
                        "dispense_mass": 15,
                        "dispense_unit": "mg"                    
                    }
                },
                {
                    "name": "reaction_op",
                    "op": "MTSynthHeatStirOp",
                    "parameters": {
                        "target_temperature": 100,
                        "target_stirring_speed": 50,
                        "wait_duration": None,
                    }
                },
                {
                    "name": "heat_stir_discharge",
                    "op": "MTSynthHeatStirOp",
                    "parameters": {
                        "target_temperature": 100,
                        "target_stirring_speed": 50,
                        "wait_duration": None,
                    }
                },
                {
                "name": "discharge_product",
                "op": "MTSynthTimedOpenReactionValveOp",
                "parameters": {
                    "duration": 1,
                    "time_unit": "second"                    
                }
                },
                {
                    "name": "wash_product",
                    "op": "MTSynthAddWashLiquidOp",
                    "parameters": {
                        "liquid_name": "H2O",
                        "dispense_volume": 20,
                        "dispense_unit": "mL"                    
                    }
                },
                {
                    "name": "dry_product",
                    "op": "MTSynthDryOp",
                    "parameters": {
                        "duration": 5,
                        "time_unit": "minute"                    
                    }
                }]

        process = MTAPCSynthProcess.from_args(lot=lot,
                                              target_product_concentration=0.8,
                                              num_discharge_cycles=3,
                                              num_wash_cycles=3,
                                              operations=operations)
        process.lot_slot = 0
        process.assigned_to = station.object_id
        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')

        # add_liquid
        process.tick()
        self.assertEqual(process.m_state, 'add_liquid_1')
        test_req_station_op(self, process, MTSynthDispenseLiquidOp)

        # open_sash
        process.tick()
        self.assertEqual(process.m_state, 'open_sash')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # update_open_sash
        process.tick()
        self.assertEqual(process.m_state, 'update_open_sash')
        self.assertTrue(station.window_open)

        # load_solid_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'load_solid_cartridge')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # update_loading_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'update_loading_cartridge')
        self.assertIsNotNone(station.loaded_cartridge)

        # operate_solid_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'operate_solid_cartridge')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # add_solid
        process.tick()
        self.assertEqual(process.m_state, 'add_solid')
        test_req_station_op(self, process, MTSynthDispenseSolidOp)

        # unload_solid_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'unload_solid_cartridge')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # update_unloading_cartridge
        process.tick()
        self.assertEqual(process.m_state, 'update_unloading_cartridge')
        self.assertIsNone(station.loaded_cartridge)

        # close_sash
        process.tick()
        self.assertEqual(process.m_state, 'close_sash')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # update_close_sash
        process.tick()
        self.assertEqual(process.m_state, 'update_close_sash')
        self.assertFalse(station.window_open)

        # start_adding_liquid_2
        process.tick()
        self.assertEqual(process.m_state, 'start_adding_liquid_2')
        test_req_station_op(self, process, MTSynthStartLiquidDispensingOp)

        # sample_reaction
        process.tick()
        self.assertEqual(process.m_state, 'sample_reaction')
        test_req_station_op(self, process, MTSynthSampleOp)

        # start_analysis_process
        from archemist.stations.waters_lcms_station.process import APCLCMSAnalysisProcess
        process.tick()
        self.assertEqual(process.m_state, 'start_analysis_process')
        test_req_station_proc(self, process, APCLCMSAnalysisProcess)

        # run_reaction
        process.tick()
        self.assertEqual(process.m_state, 'run_reaction')
        test_req_station_op(self, process, MTSynthHeatStirOp)

        # wait_for_result
        process.tick()
        self.assertEqual(process.m_state, 'wait_for_result')
        
        # manually add lcms analysis result to advance state
        from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
        solubility_result = LCMSAnalysisResult.from_args(origin_op=ObjectId(),
                                                         concentration=0.65,
                                                         result_filename="some_file.xml")
        batch.samples[0].add_result_op(solubility_result)

        # sample_reaction
        process.tick()
        self.assertEqual(process.m_state, 'sample_reaction')
        test_req_station_op(self, process, MTSynthSampleOp)

        # start_analysis_process
        process.tick()
        self.assertEqual(process.m_state, 'start_analysis_process')
        test_req_station_proc(self, process, APCLCMSAnalysisProcess)

        # run_reaction
        process.tick()
        self.assertEqual(process.m_state, 'run_reaction')
        test_req_station_op(self, process, MTSynthHeatStirOp)

        # wait_for_result
        process.tick()
        self.assertEqual(process.m_state, 'wait_for_result')
        
        # manually add lcms analysis result to advance state
        solubility_result = LCMSAnalysisResult.from_args(origin_op=ObjectId(),
                                                         concentration=0.99,
                                                         result_filename="some_file.xml")
        batch.samples[0].add_result_op(solubility_result)

        # stop_adding_liquid_2
        process.tick()
        self.assertEqual(process.m_state, 'stop_adding_liquid_2')
        test_req_station_op(self, process, MTSynthStopLiquidDispensingOp)

        # stop_reaction
        process.tick()
        self.assertEqual(process.m_state, 'stop_reaction')
        test_req_station_op(self, process, MTSynthStopReactionOp)

        # filtration_process
        process.tick()
        self.assertEqual(process.m_state, 'filtration_process')
        test_req_station_proc(self, process, MTAPCFilterProcess)
        
        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_filteration_process(self):

        # construct station
        station = MTSynthesisStation.from_dict(self.station_doc)
        
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        
        # add batches to station
        station.add_lot(lot)

        # create station process
        operations = [
            {
                    "name": "heat_stir_discharge",
                    "op": "MTSynthHeatStirOp",
                    "parameters": {
                        "target_temperature": 100,
                        "target_stirring_speed": 50,
                        "wait_duration": None,
                    }
            },
            {
                "name": "discharge_product",
                "op": "MTSynthTimedOpenReactionValveOp",
                "parameters": {
                    "duration": 1,
                    "time_unit": "second"                    
                }
            },
            {
                "name": "wash_product",
                "op": "MTSynthAddWashLiquidOp",
                "parameters": {
                    "liquid_name": "H2O",
                    "dispense_volume": 20,
                    "dispense_unit": "mL"                    
                }
            },
            {
                "name": "dry_product",
                "op": "MTSynthDryOp",
                "parameters": {
                    "duration": 5,
                    "time_unit": "minute"                    
                }
            },
        ]

        num_discharge_cycles = 3
        num_wash_cycles = 3
        process = MTAPCFilterProcess.from_args(lot=lot,
                                              operations=operations,
                                              num_discharge_cycles=num_discharge_cycles,
                                              num_wash_cycles=num_wash_cycles)
        process.lot_slot = 0
        process.assigned_to = station.object_id
        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')
        self.assertEqual(process.data['num_discharge_cycles'], 0)

        # start_heat_stir
        process.tick()
        self.assertEqual(process.m_state, 'start_heat_stir')
        test_req_station_op(self, process, MTSynthHeatStirOp)

        # open_close_drain_valve
        process.tick()
        self.assertEqual(process.m_state, 'open_close_drain_valve')
        test_req_station_op(self, process, MTSynthTimedOpenReactionValveOp)

        # increment_discharge_cycle
        process.tick()
        self.assertEqual(process.m_state, 'increment_discharge_cycle')
        self.assertEqual(process.data['num_discharge_cycles'], 1)

        # filter_product
        process.tick()
        self.assertEqual(process.m_state, 'filter_product')
        test_req_station_op(self, process, MTSynthFilterOp)

        for i in range(1, num_discharge_cycles):
            # open_close_drain_valve
            process.tick()
            self.assertEqual(process.m_state, 'open_close_drain_valve')
            test_req_station_op(self, process, MTSynthTimedOpenReactionValveOp)

            # increment_discharge_cycle
            process.tick()
            self.assertEqual(process.m_state, 'increment_discharge_cycle')
            self.assertEqual(process.data['num_discharge_cycles'], i+1)

            # filter_product
            process.tick()
            self.assertEqual(process.m_state, 'filter_product')
            test_req_station_op(self, process, MTSynthFilterOp)

        for i in range(num_discharge_cycles, num_wash_cycles + num_discharge_cycles):
            # test add_wash_liquid 
            process.tick()
            self.assertEqual(process.m_state, 'add_wash_liquid')
            test_req_station_op(self, process, MTSynthAddWashLiquidOp)

            # open_close_drain_valve
            process.tick()
            self.assertEqual(process.m_state, 'open_close_drain_valve')
            test_req_station_op(self, process, MTSynthTimedOpenReactionValveOp)

            # increment_discharge_cycle
            process.tick()
            self.assertEqual(process.m_state, 'increment_discharge_cycle')
            self.assertEqual(process.data['num_discharge_cycles'], i+1)

            # filter_product
            process.tick()
            self.assertEqual(process.m_state, 'filter_product')
            test_req_station_op(self, process, MTSynthFilterOp)

        # stop_heat_stir
        process.tick()
        self.assertEqual(process.m_state, 'stop_heat_stir')
        test_req_station_op(self, process, MTSynthStopReactionOp)

        # dry_product
        process.tick()
        self.assertEqual(process.m_state, 'dry_product')
        test_req_station_op(self, process, MTSynthDryOp)
        
        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_cleaning_process(self):

        # construct station
        station = MTSynthesisStation.from_dict(self.station_doc)
        
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        
        # add batches to station
        station.add_lot(lot)

        # create station process
        process = MTAPCCleanProcess.from_args(lot=lot,
                                              target_purity_concentration=0.01)
        process.lot_slot = 0
        process.assigned_to = station.object_id

        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')

        # load_cleaning_funnel
        process.tick()
        self.assertEqual(process.m_state, 'load_cleaning_funnel')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # add_wash_liquid
        process.tick()
        self.assertEqual(process.m_state, 'add_wash_liquid')
        test_req_station_op(self, process, MTSynthAddWashLiquidOp)

        # run_reaction
        process.tick()
        self.assertEqual(process.m_state, 'run_reaction')
        test_req_station_op(self, process, MTSynthHeatStirOp)

        # stop_reaction
        process.tick()
        self.assertEqual(process.m_state, 'stop_reaction')
        test_req_station_op(self, process, MTSynthStopReactionOp)

        # sample_reaction
        process.tick()
        self.assertEqual(process.m_state, 'sample_reaction')
        test_req_station_op(self, process, MTSynthSampleOp)

        # start_analysis_process
        from archemist.stations.waters_lcms_station.process import APCLCMSAnalysisProcess
        process.tick()
        self.assertEqual(process.m_state, 'start_analysis_process')
        test_req_station_proc(self, process, APCLCMSAnalysisProcess)

        # open_close_drain_valve
        process.tick()
        self.assertEqual(process.m_state, 'open_close_drain_valve')
        test_req_station_op(self, process, MTSynthTimedOpenReactionValveOp)

        # drain_filter
        process.tick()
        self.assertEqual(process.m_state, 'drain_filter')
        test_req_station_op(self, process, MTSynthDrainOp)

        # manually add lcms analysis result to advance state
        from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
        solubility_result = LCMSAnalysisResult.from_args(origin_op=ObjectId(),
                                                         concentration=0.65,
                                                         result_filename="some_file.xml")
        batch.samples[0].add_result_op(solubility_result)

        # add_wash_liquid
        process.tick()
        self.assertEqual(process.m_state, 'add_wash_liquid')
        test_req_station_op(self, process, MTSynthAddWashLiquidOp)

        # run_reaction
        process.tick()
        self.assertEqual(process.m_state, 'run_reaction')
        test_req_station_op(self, process, MTSynthHeatStirOp)

        # stop_reaction
        process.tick()
        self.assertEqual(process.m_state, 'stop_reaction')
        test_req_station_op(self, process, MTSynthStopReactionOp)

        # sample_reaction
        process.tick()
        self.assertEqual(process.m_state, 'sample_reaction')
        test_req_station_op(self, process, MTSynthSampleOp)

        # start_analysis_process
        process.tick()
        self.assertEqual(process.m_state, 'start_analysis_process')
        test_req_station_proc(self, process, APCLCMSAnalysisProcess)

        # open_close_drain_valve
        process.tick()
        self.assertEqual(process.m_state, 'open_close_drain_valve')
        test_req_station_op(self, process, MTSynthTimedOpenReactionValveOp)

        # drain_filter
        process.tick()
        self.assertEqual(process.m_state, 'drain_filter')
        test_req_station_op(self, process, MTSynthDrainOp)

        # manually add lcms analysis result to advance state
        solubility_result = LCMSAnalysisResult.from_args(origin_op=ObjectId(),
                                                         concentration=0.005,
                                                         result_filename="some_file.xml")
        batch.samples[0].add_result_op(solubility_result)

        # unload_cleaning_funnel
        process.tick()
        self.assertEqual(process.m_state, 'unload_cleaning_funnel')
        test_req_robot_ops(self, process, [RobotTaskOp])
        
        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_sim_handler(self):
        # construct station
        station = MTSynthesisStation.from_dict(self.station_doc)

        batch_1 = Batch.from_args(1)
        lot = Lot.from_args([batch_1])

        # add batches to station
        station.add_lot(lot)

        # construct handler
        handler = SimMTSynthesisStationHandler(station)

        # initialise the handler
        self.assertTrue(handler.initialise())

        # construct liquid dispense op
        t_op = MTSynthDispenseLiquidOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='C4O3H6',
                                            dispense_volume=100,
                                            dispense_unit="mL")
        station.add_station_op(t_op)
        station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], MaterialOpResult))
        self.assertEqual(op_results[0].material_names[0], "C4O3H6")
        self.assertEqual(op_results[0].amounts[0], 100)
        self.assertEqual(op_results[0].units[0], "mL")

        station.complete_assigned_op(outcome, op_results)

        # construct stop liquid dispense op
        t_op = MTSynthStopLiquidDispensingOp.from_args(target_sample=lot.batches[0].samples[0],
                                            liquid_name='C4O3H6')
        station.add_station_op(t_op)
        station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], MaterialOpResult))
        self.assertEqual(op_results[0].material_names[0], "C4O3H6")
        self.assertLessEqual(op_results[0].amounts[0], 5)
        self.assertEqual(op_results[0].units[0], "mL")

        station.complete_assigned_op(outcome, op_results)

        # construct solid dispense op
        station.load_cartridge(0)
        t_op = MTSynthDispenseSolidOp.from_args(target_sample=lot.batches[0].samples[0],
                                            solid_name='NaCl',
                                            dispense_mass=10,
                                            dispense_unit="mg")
        station.add_station_op(t_op)
        station.update_assigned_op()
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertTrue(isinstance(op_results[0], MaterialOpResult))
        self.assertEqual(op_results[0].material_names[0], "NaCl")
        self.assertEqual(op_results[0].amounts[0], 10)
        self.assertEqual(op_results[0].units[0], "mg")

        station.complete_assigned_op(outcome, op_results)

        # construct MTSynthHeatStirOp
        t_op = MTSynthHeatStirOp.from_args(target_sample=lot.batches[0].samples[0],
                                            target_temperature=100,
                                            target_stirring_speed=110,
                                            wait_duration=3,
                                            time_unit="minute")
        station.add_station_op(t_op)
        station.update_assigned_op()

        # get op 
        parameters = {}
        parameters["target_temperature"]  = t_op.target_temperature
        parameters["target_stirring_speed"]  = t_op.target_stirring_speed
        parameters["wait_duration"]  = t_op.wait_duration
        parameters["time_unit"]  = t_op.time_unit
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertDictEqual(op_results[0].parameters, parameters)

        station.complete_assigned_op(outcome, op_results)

        # construct MTSynthSampleOp
        t_op = MTSynthSampleOp.from_args(target_sample=lot.batches[0].samples[0],
                                            target_temperature=100,
                                            target_stirring_speed=110)
        station.add_station_op(t_op)
        station.update_assigned_op()

        # get op 
        parameters = {}
        parameters["target_temperature"]  = t_op.target_temperature
        parameters["target_stirring_speed"]  = t_op.target_stirring_speed
        
        outcome, op_results = handler.get_op_result()
        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertEqual(len(op_results), 1)
        self.assertEqual(op_results[0].origin_op, t_op.object_id)
        self.assertDictEqual(op_results[0].parameters, parameters)

        station.complete_assigned_op(outcome, op_results)