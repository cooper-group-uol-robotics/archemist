
from archemist.stations.syringe_pump_station.state import (
    SyringePumpStation,
    SyringePumpDispenseRateOp,
    SyringePumpDispenseVolumeOp,
    SyringePumpFinishDispensingOp,
)
from archemist.stations.syringe_pump_station.handler import (APCSyringePumpStationRosHandler)
from archemist.core.state.batch import Batch
from archemist.core.state.lot import Lot

from datetime import datetime

if __name__ == "__main__":
    station_doc = {
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
                "details": {"in_port": 3, "out_port": 4},
                'expiry_date': datetime.fromisoformat('2025-02-11')
            }]
        },
        'properties': None
    }
    # construct the station
    station = SyringePumpStation.from_dict(station_doc)

    # construct lot
    batch = Batch.from_args(1)
    lot = Lot.from_args([batch])

    # add lot to station
    station.add_lot(lot)

    # construct handler
    handler = APCSyringePumpStationRosHandler(station)
    # initialise handler
    handler.initialise()

    # test IKAHeatStirBatchOp
    # construct op
    t_op = SyringePumpDispenseVolumeOp.from_args(
        target_sample= batch.samples[0],
        liquid_name= 'water', 
        dispense_volume= 400,
        dispense_unit= "mL",
        dispense_rate= 5,
        rate_unit=  "mL/second"
    )
    station.add_station_op(t_op)
    station.update_assigned_op()

    # execute op
    handler.execute_op()

    # wait for results
    handler.is_op_execution_complete()

    # retrieve the result
    outcome, results = handler.get_op_result()

    # complete the operation
    station.complete_assigned_op(outcome, results)

    # # test IKAHeatBatchOp
    # # construct op
    # t_op = IKAHeatBatchOp.from_args(
    #     target_batch=batch, target_temperature=100, duration=10, time_unit="minute"
    # )
    # station.add_station_op(t_op)
    # station.update_assigned_op()

    # # execute op
    # handler.execute_op()

    # # wait for results
    # handler.is_op_execution_complete()

    # # retrieve the result
    # outcome, results = handler.get_op_result()

    # # complete the operation
    # station.complete_assigned_op(outcome, results)

    handler.shut_down()
