import unittest

from mongoengine import connect
from bson.objectid import ObjectId

from archemist.core.state.station import StationState
from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.stations.apc_meta_station.state import APCMetaStation
from archemist.stations.apc_fumehood_station.state import APCOpenSashOp, APCCloseSashOp
from archemist.stations.apc_weighing_station.process import APCWeighingProcess
from archemist.stations.syringe_pump_station.state import (SyringePumpDispenseVolumeOp,
                                                           SyringePumpDispenseRateOp,
                                                           SyringePumpFinishDispensingOp)
from archemist.stations.mt_synthesis_station.state import (MTSynthSampleOp,
                                                           MTSynthHeatStirOp,
                                                           MTSynthStopReactionOp,
                                                           MTSynthTimedOpenReactionValveOp)
from archemist.stations.diaphragm_pump_station.state import DiaphragmPumpDispenseVolumeOp
from archemist.stations.apc_filtration_station.state import (APCFilterProductOp,
                                                             APCDryProductOp,
                                                             APCDrainWasteOp)
from archemist.stations.apc_fumehood_station.process import APCSolidAdditionProcess
from archemist.stations.waters_lcms_station.process import APCLCMSAnalysisProcess
from archemist.core.state.robot_op import (RobotTaskOp,
                                           RobotNavOp,
                                           RobotWaitOp)
from archemist.stations.apc_meta_station.process import (APCSynthesisProcess,
                                                         APCFiltrationProcess,
                                                         APCCleaningProcess,
                                                         APCMeasureYieldProcess)
from archemist.core.util.enums import ProcessStatus
from .testing_utils import test_req_robot_ops, test_req_station_op, test_req_station_proc

class APCMetaStationTest(unittest.TestCase):

    def setUp(self) -> None:
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_dict = {
            'type': 'APCMetaStation',
            'id': 23,
            'location': {'coordinates': [1,7], 'descriptor': "APCMetaStation"},
            'total_lot_capacity': 1,
            'handler': 'SimStationOpHandler',
            'properties': None
        }

    def  tearDown(self) -> None:
        coll_list = self._client[self._db_name].list_collection_names()
        for coll in coll_list:
            self._client[self._db_name][coll].drop()

    def test_state(self):
        # test station is constructed properly
        station = APCMetaStation.from_dict(self.station_dict)
        self.assertIsNotNone(station)
        self.assertEqual(station.state, StationState.INACTIVE)

    def test_synthesis_process(self):

        # construct stations
        station = APCMetaStation.from_dict(self.station_dict)
        
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        
        # add batches to station
        station.add_lot(lot)

        # create station process
        operations = [
                {
                    "name": "add_liquid_1",
                    "op": "SyringePumpDispenseVolumeOp",
                    "parameters": {
                        "liquid_name": "water",
                        "dispense_volume": 50,
                        "dispense_unit": "mL",
                        "dispense_rate": 5,
                        "rate_unit": "mL/minute"                   
                    }
                },
                {
                    "name": "dispense_liquid_2",
                    "op": "SyringePumpDispenseRateOp",
                    "parameters": {
                        "liquid_name": "C4O3H6",
                        "dispense_rate": 5,
                        "rate_unit": "mL/minute"                  
                    }
                },
                {
                    "name": "add_solid",
                    "op": "APCDispenseSolidOp",
                    "parameters": {
                        "solid_name": "NaCl",
                        "dispense_mass": 15,
                        "dispense_unit": "mg"                    
                    }
                },
                {
                    "name": "heat_stir",
                    "op": "MTSynthHeatStirOp",
                    "parameters": {
                        "target_temperature": 100,
                        "target_stirring_speed": 50,
                        "wait_duration": None,
                    }
                }]

        process = APCSynthesisProcess.from_args(lot=lot,
                                              target_batch_index=0,
                                              target_sample_index=0,
                                              target_product_concentration=0.8,
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
        test_req_station_op(self, process, SyringePumpDispenseVolumeOp)

        # add_solid
        process.tick()
        self.assertEqual(process.m_state, 'add_solid')
        test_req_station_proc(self, process, APCSolidAdditionProcess)

        # start_adding_liquid_2
        process.tick()
        self.assertEqual(process.m_state, 'start_adding_liquid_2')
        test_req_station_op(self, process, SyringePumpDispenseRateOp)

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
        test_req_station_op(self, process, SyringePumpFinishDispensingOp)

        # stop_reaction
        process.tick()
        self.assertEqual(process.m_state, 'stop_reaction')
        test_req_station_op(self, process, MTSynthStopReactionOp)
        
        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_filtration_process(self):

        # construct station
        station = APCMetaStation.from_dict(self.station_dict)
        
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
                "op": "DiaphragmPumpDispenseVolumeOp",
                "parameters": {
                    "liquid_name": "water",
                    "dispense_volume": 20,
                    "dispense_unit": "mL"                    
                }
            },
            {
                "name": "dry_product",
                "op": "APCDryProductOp",
                "parameters": {
                    "duration": 5,
                    "time_unit": "minute"                    
                }
            },
        ]

        num_discharge_cycles = 3
        num_wash_cycles = 3
        process = APCFiltrationProcess.from_args(lot=lot,
                                                target_batch_index=0,
                                                target_sample_index=0,
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
        test_req_station_op(self, process, APCFilterProductOp)

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
            test_req_station_op(self, process, APCFilterProductOp)

        for i in range(num_discharge_cycles, num_wash_cycles + num_discharge_cycles):
            # test add_wash_liquid 
            process.tick()
            self.assertEqual(process.m_state, 'add_wash_liquid')
            test_req_station_op(self, process, DiaphragmPumpDispenseVolumeOp)

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
            test_req_station_op(self, process, APCFilterProductOp)

        # stop_heat_stir
        process.tick()
        self.assertEqual(process.m_state, 'stop_heat_stir')
        test_req_station_op(self, process, MTSynthStopReactionOp)

        # dry_product
        process.tick()
        self.assertEqual(process.m_state, 'dry_product')
        test_req_station_op(self, process, APCDryProductOp)
        
        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)

    def test_cleaning_process(self):

        # construct station
        station = APCMetaStation.from_dict(self.station_dict)
        
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        
        # add batches to station
        station.add_lot(lot)

        # create station process
        operations = [
                {
                "name": "add_wash_liquid",
                "op": "DiaphragmPumpDispenseVolumeOp",
                "parameters": {
                    "liquid_name": "water",
                    "dispense_volume": 20,
                    "dispense_unit": "mL"                    
                }
                },
                {
                    "name": "wash_heat_stir",
                    "op": "MTSynthHeatStirOp",
                    "parameters": {
                        "target_temperature": 100,
                        "target_stirring_speed": 50,
                        "wait_duration": None,
                    }
                },
                {
                "name": "wash_discharge",
                "op": "MTSynthTimedOpenReactionValveOp",
                "parameters": {
                    "duration": 1,
                    "time_unit": "second"                    
                }
            }]

        process = APCCleaningProcess.from_args(lot=lot,
                                              target_purity_concentration=0.01,
                                              target_batch_index=0,
                                              target_sample_index=0,
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

        # load_cleaning_funnel
        process.tick()
        self.assertEqual(process.m_state, 'load_cleaning_funnel')
        test_req_robot_ops(self, process, [RobotTaskOp])

        # add_wash_liquid
        process.tick()
        self.assertEqual(process.m_state, 'add_wash_liquid')
        test_req_station_op(self, process, DiaphragmPumpDispenseVolumeOp)

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
        test_req_station_op(self, process, APCDrainWasteOp)

        # manually add lcms analysis result to advance state
        from archemist.stations.waters_lcms_station.state import LCMSAnalysisResult
        solubility_result = LCMSAnalysisResult.from_args(origin_op=ObjectId(),
                                                         concentration=0.65,
                                                         result_filename="some_file.xml")
        batch.samples[0].add_result_op(solubility_result)

        # add_wash_liquid
        process.tick()
        self.assertEqual(process.m_state, 'add_wash_liquid')
        test_req_station_op(self, process, DiaphragmPumpDispenseVolumeOp)

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
        test_req_station_op(self, process, APCDrainWasteOp)

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

    def test_weighing_process(self):

        # construct stations
        station = APCMetaStation.from_dict(self.station_dict)
        
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        
        # add batches to station
        station.add_lot(lot)

        # create station process
        process = APCMeasureYieldProcess.from_args(lot=lot,
                                              target_batch_index=0,
                                              target_sample_index=0)
        process.lot_slot = 0
        process.assigned_to = station.object_id
        
        # assert initial state
        self.assertEqual(process.m_state, 'init_state')
        self.assertEqual(process.status, ProcessStatus.INACTIVE)

        # prep_state
        process.tick()
        self.assertEqual(process.status, ProcessStatus.RUNNING)
        self.assertEqual(process.m_state, 'prep_state')

        # navigate_to_weighing_station
        process.tick()
        self.assertEqual(process.m_state, 'navigate_to_weighing_station')
        test_req_robot_ops(self, process, [RobotNavOp, RobotWaitOp])

        # open_sash
        process.tick()
        self.assertEqual(process.m_state, 'open_sash')
        test_req_station_op(self, process, APCOpenSashOp)

        # weight_yield
        process.tick()
        self.assertEqual(process.m_state, 'weight_yield')
        test_req_station_proc(self, process, APCWeighingProcess)

        # close_sash
        process.tick()
        self.assertEqual(process.m_state, 'close_sash')
        test_req_station_op(self, process, APCCloseSashOp)
        
        # final_state
        process.tick()
        self.assertEqual(process.m_state, 'final_state')
        self.assertEqual(process.status, ProcessStatus.FINISHED)