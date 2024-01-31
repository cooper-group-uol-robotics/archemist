#!/usr/bin/env python3

import unittest

from archemist.stations.syringe_pump_station.state import (
    SyringePumpStation,
    SyringePumpDispenseRateOp,
    SyringePumpDispenseVolumeOp,
    SyringePumpFinishDispensingOp,
)

from archemist.core.state.material import Liquid

from archemist.stations.syringe_pump_station.handler import APCSyringePumpStationRosHandler
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot
from archemist.core.util.enums import StationState, OpOutcome
from archemist.core.state.station_op_result import ProcessOpResult, MaterialOpResult
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy, DictProxy



from datetime import datetime

from mongoengine import connect

print("importing done")


class syringPumpHandlerTest(unittest.TestCase):
    def setUp(self):
        self._db_name = 'archemist_test'
        self._client = connect(db=self._db_name, host='mongodb://localhost:27017', alias='archemist_state')

        self.station_doc = {
        'type': 'SyringePumpStation',
        'id': 20,
        'location': {'coordinates': [1,7], 'descriptor': "APCSyringePumpStation"},
        'total_lot_capacity': 1,
        'handler': 'APCSyringePumpStationHandler',
        'materials':
        {
            'liquids':
            [{
                'name': 'water',
                'amount': 400,
                'unit': 'mL',
                'density': 997,
                'density_unit': "kg/m3",
                "details": {"inlet_port": 3, "outlet_port": 4},
                'expiry_date': datetime.fromisoformat('2025-02-11')
            },
            {
                'name': 'acid',
                'amount': 400,
                'unit': 'mL',
                'density': 997,
                'density_unit': "kg/m3",
                "details": {"inlet_port": 1, "outlet_port": 2},
                'expiry_date': datetime.fromisoformat('2025-02-11')
            }]
        },
        'properties': None
    }


    def test_handler(self):
        station = SyringePumpStation.from_dict(self.station_doc)
        self.assertIsNotNone(station)
        self.assertEqual(station.state, StationState.INACTIVE)
            # construct lot
        batch = Batch.from_args(2)
        lot = Lot.from_args([batch])

        # add lot to station
        station.add_lot(lot)

        # construct handler
        handler = APCSyringePumpStationRosHandler(station)
        # initialise handler
        self.assertTrue(handler.initialise())

        # test IKAHeatStirBatchOp
        # construct op
        t_op = SyringePumpDispenseVolumeOp.from_args(
            target_sample= batch.samples[0],
            liquid_name= 'acid', 
            dispense_volume= 105,
            dispense_unit= "mL",
            dispense_rate= 5.2,
            rate_unit=  "mL/minute"
        )

        self.assertIsNotNone(t_op.object_id)
        self.assertEqual(t_op.liquid_name, "acid")
        self.assertEqual(t_op.dispense_volume, 105)
        self.assertEqual(t_op.dispense_unit, "mL")
        self.assertEqual(t_op.dispense_rate, 5.2)
        self.assertEqual(t_op.rate_unit, "mL/minute")

        station.add_station_op(t_op)
        station.update_assigned_op()
        self.assertEqual(station.assigned_op, t_op)
        self.assertEqual(station.assigned_op.liquid_name, "acid")
        self.assertIsInstance(station.liquids_dict, DictProxy)
        self.assertIsInstance(dict(station.liquids_dict), dict)
        # x = dict(station.liquids_dict)
        # print(x)
        print(station.liquids_dict)
        # print(station.liquids_dict.get(station.liquids_dict.))
        # # print(station.liquids_dict[station.assigned_op.liquid_name]['details'])
        liq = Liquid.from_object_id(station.liquids_dict[station.assigned_op.liquid_name].object_id)
        # self.assertIsInstance(liq, dict)
        details = liq.details
        print("inlet_port", details.get("inlet_port"))
        print("outlet_port", details.get("outlet_port"))


        # execute op
        handler.execute_op()

        # wait for results
        handler.is_op_execution_complete()

        self.assertEqual(handler.is_op_execution_complete(), False)

        handler._op_complete = True

        self.assertEqual(handler.is_op_execution_complete(), True)



        outcome, op_results = handler.get_op_result()


        self.assertEqual(outcome, OpOutcome.SUCCEEDED)
        self.assertIsInstance(op_results[0], MaterialOpResult)


if __name__ == '__main__':
    unittest.main()
