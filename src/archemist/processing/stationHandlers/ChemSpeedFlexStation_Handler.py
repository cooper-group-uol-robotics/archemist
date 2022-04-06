#!/usr/bin/env python3

import rospy
from archemist.state.station import Station
from archemist.state.stations.chemspeed_flex_station import CSOpenDoorOpDescriptor,CSCloseDoorOpDescriptor,CSJobOutputDescriptor,CSProcessingOpDescriptor
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.processing.handler import StationHandler
from chemspeed_flex_msgs.msg import CSFlexCommand,CSFlexStatus
from rospy.core import is_shutdown

class ChemSpeedFlexStation_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._cs_status = CSFlexStatus.DOOR_CLOSED
        rospy.init_node(f'{self._station}_handler')
        self.pubCS_Flex = rospy.Publisher("/ChemSpeed_Flex_commands", CSFlexCommand, queue_size=1)
        rospy.Subscriber('/ChemSpeed_Flex_status', CSFlexStatus, self._cs_state_update, queue_size=2)
        rospy.sleep(1)
        

    def run(self):
        rospy.loginfo(f'{self._station}_handler is running')
        try:
            while not rospy.is_shutdown():
                self.handle()
                rospy.sleep(2)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._station}_handler is terminating!!!')

    def process(self):
        current_op = self._station.get_station_op()
        current_op.add_timestamp()
        if (isinstance(current_op,CSOpenDoorOpDescriptor)):
            rospy.loginfo('opening chemspeed door')
            self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.OPEN_DOOR)
            self._wait_for_status(CSFlexStatus.DOOR_OPEN)
        elif (isinstance(current_op,CSCloseDoorOpDescriptor)):
            rospy.loginfo('closing chemspeed door')
            self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.CLOSE_DOOR)
            self._wait_for_status(CSFlexStatus.DOOR_CLOSED)
        elif (isinstance(current_op,CSProcessingOpDescriptor)):
            rospy.loginfo('starting chemspeed job')
            self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.RUN_APP)
            self._wait_for_status(CSFlexStatus.JOB_COMPLETE)
        
        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.add_timestamp()

        return current_op

    def _cs_state_update(self, msg):
        # if self._station.status != msg.cs_flex_status:
        #     self._station.status = msg.cs_flex_status
        if self._cs_status != msg.cs_flex_status:
            self._cs_status = msg.cs_flex_status

    def _wait_for_status(self, status):
        while(self._cs_status != status):
            rospy.sleep(0.3)