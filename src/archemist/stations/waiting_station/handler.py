#test comment

import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import WaitingOpDescriptor
from threading import Thread
from archemist.core.processing.handler import StationHandler

class WaitingStationHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._cmd_time = -1
        self._task_finished = False
        self._total_waiting_duration = 120
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
        self._thread = Thread(target=self._mock_execution)
        self._thread.start()

    def is_op_execution_complete(self) -> bool:
        if self._thread.is_alive():
            return False
        else:
            return True

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}
    
    def _mock_execution(self):
        current_op = self._station.get_assigned_station_op()
        print(f'performing operation {current_op}')  
        if (isinstance(current_op,WaitingOpDescriptor)):
            rospy.loginfo('120s Waiting is started')
            self._cmd_time = rospy.get_time()
            while True:
                elepsed_duration = rospy.get_time() - self._cmd_time
                if elepsed_duration > self._total_waiting_duration:
                    self._task_finished = True
                    break    
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')