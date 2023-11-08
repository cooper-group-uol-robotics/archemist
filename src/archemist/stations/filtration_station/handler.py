import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import BaseValveCloseOpDescriptor, BaseValveOpenOpDescriptor, VacuumCloseOpDescriptor, VacuumOpenOpDescriptor, DrainValveCloseOpDescriptor, DrainValveOpenOpDescriptor, IdleOpDescriptor
from archemist.core.processing.handler import StationHandler
from roslabware_msgs.msg import FiltrationCmd,FiltrationStatus, BaseValveCmd
from std_msgs.msg import Bool
from rospy.core import is_shutdown

class FiltrationStationROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._Filtration_current_task_complete = False
        rospy.init_node(f'{self._station}_handler')
        self.pubBaseValve = rospy.Publisher("/Optimax_BaseValve_Commands", BaseValveCmd, queue_size=1)
        self.pubFS = rospy.Publisher("/Filtration_Commands", FiltrationCmd, queue_size=1)
        rospy.Subscriber('/filtration/task_complete', Bool, self._fs_state_update, queue_size=1)
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
        if (isinstance(current_op, BaseValveOpenOpDescriptor)):
            rospy.loginfo('opening base valve')
            for i in range(10):
                self.pubBaseValve.publish(valve_command=BaseValveCmd.OPEN)
        elif (isinstance(current_op,BaseValveCloseOpDescriptor)):
            rospy.loginfo('closing base valve')
            for i in range(10):
                self.pubBaseValve.publish(valve_command=BaseValveCmd.CLOSE)
        elif (isinstance(current_op,VacuumOpenOpDescriptor)):
            rospy.loginfo('opening vacuum')
            for i in range(10):
                self.pubFS.publish(filtration_system_command=FiltrationCmd.VACUUM)
        elif (isinstance(current_op,VacuumCloseOpDescriptor)):
            rospy.loginfo('closing vacuum')
            for i in range(10):
                self.pubFS.publish(filtration_system_command=FiltrationCmd.STOP)
        elif (isinstance(current_op,DrainValveOpenOpDescriptor)):
            rospy.loginfo('opening drain')
            for i in range(10):
                self.pubFS.publish(filtration_system_command=FiltrationCmd.DRAIN)
        elif (isinstance(current_op,DrainValveCloseOpDescriptor)):
            rospy.loginfo('closing drain')
            for i in range(10):
                self.pubFS.publish(filtration_system_command=FiltrationCmd.STOP)
        elif (isinstance(current_op,IdleOpDescriptor)):
            rospy.loginfo('Stopping all')
            for i in range(10):
                self.pubFS.publish(filtration_system_command=FiltrationCmd.STOP)

    def is_op_execution_complete(self) -> bool:
        return self._Filtration_current_task_complete

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}
    
    def _fs_state_update(self, msg):
        if msg.data == True:
            self._Filtration_current_task_complete = True
        elif msg.data == False:
            self._Filtration_current_task_complete = False
