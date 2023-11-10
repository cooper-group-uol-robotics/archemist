import unittest

from mongoengine import connect

from archemist.stations.mt_synthesis_station.state import (MTSynthesisStation,
                                                           SynthesisCartridge,
                                                           MTSynthesisPhase,
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
        # self.assertEqual(station.synthesis_phase, MTSynthesisPhase.REACTION)
        # station.synthesis_phase = MTSynthesisPhase.CLEANING
        # self.assertEqual(station.synthesis_phase, MTSynthesisPhase.CLEANING)
        
        self.assertIsNone(station.optimax_mode)
        self.assertEqual(len(station.cartridges), 2)
        self.assertEqual(station.cartridges[0].associated_solid, "NaCl")

        self.assertIsNone(station.loaded_cartridge)
        self.assertFalse(station.vertical_doors_open)
        self.assertFalse(station.horizontal_doors_open)
        self.assertEqual(station.num_sampling_vials, 12)
        self.assertIsNone(station.set_reaction_temperature)
        self.assertIsNone(station.set_stirring_speed)

        # construct lot and add it to station
        batch = Batch.from_args(1)
        lot = Lot.from_args([batch])
        station.add_lot(lot)

        # test MTSynthHOpenDoorOp

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
        self.assertIsNone(station.set_reaction_temperature)
        self.assertIsNone(station.set_stirring_speed)

        # test MTSynthReactionOp heating
        t_op = MTSynthReactAndWaitOp.from_args(target_sample=batch.samples[0],
                                            target_temperature=100,
                                            target_stirring_speed=None,
                                            wait_duration=None,
                                            time_unit=None)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_temperature, 100)
        self.assertIsNone(t_op.target_stirring_speed)
        self.assertIsNone(t_op.wait_duration)
        self.assertIsNone(t_op.time_unit)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.HEATING)
        self.assertEqual(station.set_reaction_temperature, 100)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)
        self.assertIsNone(station.set_reaction_temperature)

        # test MTSynthReactionOp stirring
        t_op = MTSynthReactAndWaitOp.from_args(target_sample=batch.samples[0],
                                            target_temperature=None,
                                            target_stirring_speed=50,
                                            wait_duration=None,
                                            time_unit=None)
        self.assertIsNotNone(t_op)
        self.assertEqual(t_op.target_stirring_speed, 50)
        self.assertIsNone(t_op.target_temperature)
        self.assertIsNone(t_op.wait_duration)
        self.assertIsNone(t_op.time_unit)

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.optimax_mode, OptiMaxMode.STIRRING)
        self.assertEqual(station.set_stirring_speed, 50)
        station.complete_assigned_op(OpOutcome.SUCCEEDED, None)
        self.assertIsNone(station.optimax_mode)
        self.assertIsNone(station.set_stirring_speed)
