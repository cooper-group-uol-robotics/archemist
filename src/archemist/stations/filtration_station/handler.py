import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import FiltrationValveOpenOpDescriptor,FiltrationValveCloseOpDescriptor
from archemist.core.processing.handler import StationHandler
from roslabware_msgs.msg import FiltrationValveCmd,FiltrationValveStatus
from rospy.core import is_shutdown

class FiltrationStationHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._current_fs_status = FiltrationValveStatus.VALVE_CLOSED
        self._desired_fs_status = None
        rospy.init_node(f'{self._station}_handler')
        self.pub_FS = rospy.Publisher("/Filtration_station_commands", FiltrationValveCmd, queue_size=1)
        rospy.Subscriber('/Filtration_station_status', FiltrationValveStatus, self._fs_state_update, queue_size=1)
        rospy.sleep(1)
               
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
        if (isinstance(current_op, FiltrationValveCloseOpDescriptor)):
            rospy.loginfo('opening chemspeed door')
            for i in range(10):
                self.pub_FS.publish(drain_valve_command=FiltrationValveCmd.CLOSE_VALVE)
            self._desired_fs_status = FiltrationValveStatus.VALVE_CLOSED
        elif (isinstance(current_op,FiltrationValveOpenOpDescriptor)):
            rospy.loginfo('closing chemspeed door')
            for i in range(10):
                self.pub_FS.publish(drain_valve_command=FiltrationValveCmd.OPEN_VALVE)
            self._desired_fs_status = FiltrationValveStatus.VALVE_OPENED

    def is_op_execution_complete(self) -> bool:
        if self._desired_fs_status == self._current_fs_status:
            self._desired_fs_status = None
            return True
        else:
            return False

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}
    
    def _fs_state_update(self, msg):
        if self._current_fs_status != msg.drain_valve_status:
            self._current_fs_status = msg.drain_valve_status