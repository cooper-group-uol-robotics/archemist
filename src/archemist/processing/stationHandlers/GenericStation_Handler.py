from archemist.state.station import Station
from archemist.processing.handler import StationHandler
import time

class GenericStation_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def process(self):
        current_op = self._station.get_station_op()
        current_op.add_timestamp()   
        print(f'performing operation {current_op}')    
        time.sleep(1) 
        current_op.output.has_result = True
        current_op.output.success = True
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