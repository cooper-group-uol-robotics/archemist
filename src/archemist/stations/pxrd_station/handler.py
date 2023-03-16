import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import PXRDAnalysisOpDescriptor
from archemist.core.processing.handler import StationHandler
from std_msgs.msg import String
from pxrd_msgs.msg import PxrdCommand, PxrdStatus
import time
from threading import Thread

#### ACTUAL WORKFLOW (REAL SCANS + PXRD PYTHON DRIVER )

class PXRDStationROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._current_pxrd_status = PxrdStatus.NOT_LAUNCHED_YET
        self._desired_pxrd_status = None
        rospy.init_node(f'{self._station}_handler')
        self.pub_pxrd = rospy.Publisher("/PXRD_commands", PxrdCommand, queue_size=1)
        rospy.Subscriber('/PXRD_status', PxrdStatus, self._pxrd_state_update, queue_size=1)
        rospy.sleep(1)
        self._thread = None

    def execute_op(self):
        self._thread = Thread(target=self._real_execution)
        self._thread.start()

    def is_op_execution_complete(self):
        if self._thread.is_alive():
            return False
        else:
            return True

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}

    def run(self):
        print(f'{self._station}_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print(f'{self._station}_handler is terminating!!!')

    def _pxrd_state_update(self, msg):
        if self._current_pxrd_status != msg.pxrd_status:
            self._current_pxrd_status = msg.pxrd_status

    def _real_execution(self):
        current_op = self._station.get_assigned_station_op()
        print(f'performing pxrd operation {current_op}')  
        if (isinstance(current_op,PXRDAnalysisOpDescriptor)):
            if (isinstance(self._current_pxrd_status,PxrdStatus.NOT_LAUNCHED_YET)):
                rospy.loginfo('starting pxrd job')
                for i in range(10):
                    self.pub_pxrd.publish(PXRD_commands=PxrdCommand.EXECUTE)
                self._desired_pxrd_status =  PxrdStatus.EXECUTION_DONE
            elif (isinstance(self._current_pxrd_status,PxrdStatus.EXECUTION_DONE)):
                for i in range(10):
                    self.pub_pxrd.publish(PXRD_commands=PxrdCommand.TERMINATE)
        time.sleep(1)
