from archemist.state.station import Station
from archemist.state.stations.fisher_weighing_station import FisherWeightingStation
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.processing.handler import StationHandler
import time

class EmuFisherWeightingStation_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def process(self):
        print('doing operation')
        time.sleep(3)
        current_op_dict = self._station.assigned_batch.recipe.get_current_task_op_dict()
        current_op = ObjectConstructor.construct_station_op_from_dict(current_op_dict)
        current_op.add_timestamp()        
        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.weight = 10.1
        current_op.output.add_timestamp()

        return current_op

    def run(self):
        print(f'{self._station}_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print(f'{self._station}_handler is terminating!!!')

if __name__ == '__main__':
    p_manager = PersistenceManager('test')
    state = p_manager.construct_state_from_db()
    station = state.get_station('FisherWeightingStation', 5)
    fisher_handler = EmuFisherWeightingStation_Handler(station)