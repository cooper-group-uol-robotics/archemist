#test comment

import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import WaitingOpDescriptor
from archemist.core.processing.handler import StationHandler

class WaitingStationHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        #self._waiting_for = False
        #self._task_seq = 0
        self._cmd_time = -1
        self._task_finished = False
        self._total_waiting_duration = 40
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
        if (isinstance(current_op,WaitingOpDescriptor)):
            rospy.loginfo('40s Waiting is started')
            #rospy.loginfo(f'===>{current_op.duration}')
            self._cmd_time = rospy.get_time()
            while True:
                elepsed_duration = rospy.get_time() - self._cmd_time
                if elepsed_duration > self._total_waiting_duration:
                    self._task_finished = True
                    #self._waiting_for = False
                    break    
            #self._task_seq += 1

        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        return self._task_finished

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}

    # def _state_update(self):
    #     if self._waiting_for:
    #         elepsed_duration = rospy.get_time() - self._cmd_time
    #         if elepsed_duration > 40:
    #             self._task_finished = True
    #             self._waiting_for = False