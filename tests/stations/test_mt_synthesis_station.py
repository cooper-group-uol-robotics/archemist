import unittest

from mongoengine import connect

from archemist.stations.mt_synthesis_station.state import (MTSynthesisStation,
                                                           SynthesisCartridge,
                                                           MTSynthOpenWindowOp,
                                                           MTSynthCloseWindowOp,
                                                           MTSynthLoadCartridgeOp,
                                                           MTSynthDispenseSolidOp,
                                                           MTSynthUnloadCartridgeOp,
                                                           MTSynthDispenseLiquidOp,
                                                           MTSynthReactAndSampleOp,
                                                           MTSynthStopReactionOp,
                                                           MTSynthOpenReactionValveOp,
                                                           MTSynthCloseReactionValveOp,
                                                           MTSynthFilterDrainOp,
                                                           MTSynthFilterVacuumOp,
                                                           MTSynthFilterStopOp,
                                                           OptiMaxMode,
                                                           MTSynthReactAndWaitOp)
from archemist.core.util.enums import StationState
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import OpOutcome
from datetime import date

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
                    {'associated_solid': "NaCl", 'hotel_index': 0},
                    {'associated_solid': "NaCl", 'hotel_index': 1}
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
        self.assertFalse(station.optimax_valve_open)
        self.assertFalse(station.filter_drain_open)
        self.assertFalse(station.vacuum_active)
        self.assertEqual(station.num_sampling_vials, 12)
        self.assertIsNone(station.set_reaction_temperature)
        self.assertIsNone(station.set_stirring_speed)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        station.add_lot(lot)

        # test MTSynthOpenWindowOp
        t_op = MTSynthOpenWindowOp.from_args()
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(station.window_open)

        # test MTSynthCloseWindowOp
        t_op = MTSynthCloseWindowOp.from_args()
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(station.window_open)

        # test MTSynthLoadCartridgeOp
        t_op = MTSynthLoadCartridgeOp.from_args(cartridge_index=0)
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertEqual(station.loaded_cartridge.hotel_index, 0)
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

        # test MTSynthUnloadCartridgeOp 
        t_op = MTSynthUnloadCartridgeOp.from_args()
        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
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

        # test MTSynthReactionOp heating and stirring
        t_op = MTSynthReactAndWaitOp.from_args(target_sample=batch.samples[0],
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

        # test MTSynthReactionOp heating
        t_op = MTSynthReactAndWaitOp.from_args(target_sample=batch.samples[0],
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

        # test MTSynthReactionOp stirring
        t_op = MTSynthReactAndWaitOp.from_args(target_sample=batch.samples[0],
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

        # test MTSynthReactAndSampleOp heating
        t_op = MTSynthReactAndSampleOp.from_args(target_sample=batch.samples[0],
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

        # test MTSynthFilterDrainOp
        t_op = MTSynthFilterDrainOp.from_args(duration=1,
                                              time_unit="minute")
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.duration, 1)
        self.assertEqual(t_op.time_unit, "minute")

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(station.filter_drain_open)

        # test MTSynthFilterVacuumOp
        t_op = MTSynthFilterVacuumOp.from_args(duration=1,
                                              time_unit="minute")
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.duration, 1)
        self.assertEqual(t_op.time_unit, "minute")

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertTrue(station.vacuum_active)

        # test MTSynthFilterStopOp
        t_op = MTSynthFilterStopOp.from_args()
        self.assertIsNotNone(t_op)

        station.add_station_op(t_op)
        station.update_assigned_op()
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertFalse(station.filter_drain_open)
        self.assertFalse(station.vacuum_active)