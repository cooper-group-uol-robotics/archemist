from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import LCMSInsertBatchOpDescriptor, LCMSExtractBatchOpDescriptor, LCMSAnalysisOpDescriptor
from archemist.core.processing.handler import StationHandler
from threading import Thread
import socket
import time

class WaterLCMSSocketHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._host = '192.168.1.1'
        self._port = 19990
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((self._host, self._port))
        self._thread = None
        self._result_received = False
        self._op_result = False
        

    def run(self):
        print(f'{self._station}_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print(f'{self._station}_handler is terminating!!!')
        finally:
            self._socket.close()


    def execute_op(self):
        current_op = self._station.get_assigned_station_op()
        self._result_received = False
        self._op_result = False
        if (isinstance(current_op,LCMSInsertBatchOpDescriptor)):
            print(f'Autosampler - inserting rack {current_op.rack}')
            msg = f'InsertRack{current_op.rack}'
            self._socket.sendall(msg.encode('ascii'))
        elif (isinstance(current_op,LCMSExtractBatchOpDescriptor)):
            print(f'Autosampler - extracting rack {current_op.rack}')
            msg = f'ExtractRack{current_op.rack}'
            self._socket.sendall(msg.encode('ascii'))
        elif (isinstance(current_op,LCMSAnalysisOpDescriptor)):
            self._socket.sendall(b'StartAnalysisRack2')
        else:
            print(f'[{self.__class__.__name__}] Unkown operation was received')
        self._thread = Thread(target=self._lcsm_status_update, daemon=True)

    def is_op_execution_complete(self) -> bool:
        return self._result_received

    def get_op_result(self) -> Tuple[bool, Dict]:
       return self._op_result

    def _lcsm_status_update(self):
        msg = self._socket.recv(1024)
        self._result_received = True
        if msg == b'Completed':
            self._op_result = True
        elif msg == b'Error':
            self._op_result = False


        