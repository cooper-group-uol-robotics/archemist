from archemist.state.stations.fisher_weighing_station import FisherWeightingStation
from archemist.state.state import State
from archemist.processing.handler import StationHandler
import time

class EmuFisherWeightingStation_Handler(StationHandler):
    def __init__(self):
        super().__init__('FisherWeightingStation')
        print(self._station_name  + '_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print('Terminating!!!')
            pass

    def process(self):
        print('doing operation')
        time.sleep(3)
        current_op = self._station.assigned_batch.recipe.get_current_task_op()
        current_op.add_timestamp()        
        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.weight = 10.1
        current_op.output.add_timestamp()

        return current_op

if __name__ == '__main__':
    fisher_handler = EmuFisherWeightingStation_Handler()