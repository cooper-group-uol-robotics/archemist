import time
from archemist.core.state.station import Station
from archemist.core.processing.handler import StationHandler

class InputStationHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def run(self):
        print(f'{self._station}_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print(f'{self._station}_handler is terminating!!!')