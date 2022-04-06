from archemist.state.station import Station
from archemist.state.stations.chemspeed_flex_station import ChemSpeedFlexStation,CSProcessingOpDescriptor,CSCloseDoorOpDescriptor,CSOpenDoorOpDescriptor
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.persistence.persistenceManager import PersistenceManager
from archemist.processing.handler import StationHandler
import time

class EmuChemSpeedFlexStation_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def process(self):
        current_op = self._station.get_station_op()
        current_op.add_timestamp()
        if (isinstance(current_op,CSOpenDoorOpDescriptor)):
            print('opening chemspeed door')
            time.sleep(3)
        elif (isinstance(current_op,CSCloseDoorOpDescriptor)):
            print('closing chemspeed door')
            time.sleep(3)
        elif (isinstance(current_op,CSProcessingOpDescriptor)):
            print('starting chemspeed job')
            time.sleep(3)
        
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

if __name__ == '__main__':
    p_manager = PersistenceManager('test')
    state = p_manager.construct_state_from_db()
    station = state.get_station('ChemSpeedFlexStation', 88)
    chemspeed_handler = ChemSpeedFlexStation(station)