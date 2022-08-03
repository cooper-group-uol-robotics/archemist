#!/usr/bin/env python3

import rospy
from archemist.processing.handler import StationHandler
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.state.station import Station
from colorimetry_msgs.msg import ColorimetryCommand,ColorimetryResult

from rospy.core import is_shutdown

class LightBoxStation_Handler(StationHandler):
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubCamera = rospy.Publisher("/colorimetry_station/command", ColorimetryCommand, queue_size=2)
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
        current_op_dict = self._station.assigned_batch.recipe.get_current_task_op_dict()
        current_op = ObjectConstructor.construct_station_op_from_dict(current_op_dict)
        current_op.add_timestamp()

        # Process Sample 
        op_msg = ColorimetryCommand()
        op_msg.op_name = 'take_pic'
        self.pubCamera.publish(op_msg)
        message = rospy.wait_for_message("/colorimetry_station/result", ColorimetryResult)

        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.add_timestamp()
        current_op.output.file_name = message.result_file_name
        current_op.output.red_value = message.red
        current_op.output.blue_value = message.blue
        current_op.output.green_value = message.green

        return current_op
