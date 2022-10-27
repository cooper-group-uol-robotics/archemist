import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import CSOpenDoorOpDescriptor,CSCloseDoorOpDescriptor,CSProcessingOpDescriptor,CSCSVJobOpDescriptor
from archemist.core.processing.handler import StationHandler
from chemspeed_flex_msgs.msg import CSFlexCommand,CSFlexStatus
from rospy.core import is_shutdown

class ChemSpeedFlexROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._current_cs_status = CSFlexStatus.DOOR_CLOSED
        self._desired_cs_status = None
        rospy.init_node(f'{self._station}_handler')
        self.pubCS_Flex = rospy.Publisher("/ChemSpeed_Flex_commands", CSFlexCommand, queue_size=1)
        rospy.Subscriber('/ChemSpeed_Flex_status', CSFlexStatus, self._cs_state_update, queue_size=1)
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
        if (isinstance(current_op,CSOpenDoorOpDescriptor)):
            rospy.loginfo('opening chemspeed door')
            for i in range(10):
                self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.OPEN_DOOR)
            self._desired_cs_status = CSFlexStatus.DOOR_OPEN
        elif (isinstance(current_op,CSCloseDoorOpDescriptor)):
            rospy.loginfo('closing chemspeed door')
            for i in range(10):
                self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.CLOSE_DOOR)
            self._desired_cs_status = CSFlexStatus.DOOR_CLOSED
        elif (isinstance(current_op,CSProcessingOpDescriptor)):
            rospy.loginfo('starting chemspeed job')
            for i in range(10):
                self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.RUN_APP)
            self._desired_cs_status =  CSFlexStatus.JOB_COMPLETE
        elif (isinstance(current_op,CSCSVJobOpDescriptor)):
            rospy.loginfo('uploading csv file to rosparam server')
            rospy.set_param('chemspeed_input_csv', current_op.csv_string)
            rospy.sleep(3) # wait for csv to be uploaded
            rospy.loginfo('starting chemspeed job')
            for i in range(10):
                self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.RUN_APP)
            self._desired_cs_status = CSFlexStatus.JOB_COMPLETE
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        if self._desired_cs_status == self._current_cs_status:
            self._desired_cs_status = None
            return True
        else:
            return False

    def get_op_result(self) -> Tuple[bool, Dict]:
        if self._current_cs_status != CSFlexStatus.ERROR:
            return True, {}
        else:
            return False

    def _cs_state_update(self, msg):
        if self._current_cs_status != msg.cs_flex_status:
            self._current_cs_status = msg.cs_flex_status