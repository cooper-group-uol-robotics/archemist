import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import WaitingOpDescriptor
from threading import Thread
from archemist.core.processing.handler import StationHandler

class WaitingStationHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._thread = None
        rospy.init_node(f'{self._station}_handler')
               
    def run(self):
        rospy.loginfo(f'{self._station}_handler is running')
        try:
            while not rospy.is_shutdown():
                self.handle()
                rospy.sleep(2)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._station}_handler is terminating!!!')

    def execute_op(self):
        current_op = self._station.get_assigned_station_op()
        if (isinstance(current_op, WaitingOpDescriptor)):
            self._thread = Thread(target=self._mock_execution, kwargs={'duration':current_op.duration})
            self._thread.start()
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        if self._thread is not None:
            if self._thread.is_alive():
                return False
            else:
                self._thread = None
                return True
        else:
            raise RuntimeError("op execution check is called before calling executing op")

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}
    
    def _mock_execution(self, **kwargs):
        target_duration = kwargs['duration']
        rospy.loginfo(f'Starting waiting operation for {target_duration} seconds')
        start_time = rospy.get_time()
        elepsed_duration = 0
        while elepsed_duration < target_duration:
            rospy.sleep(1)
            elepsed_duration = rospy.get_time() - start_time