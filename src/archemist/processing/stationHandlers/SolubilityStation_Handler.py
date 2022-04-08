#!/usr/bin/env python3

import rospy
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.state.station import Station
from archemist_msgs.msg import CameraCommand

from rospy.core import is_shutdown

class SolubilityStation_Handler:
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubCamera = rospy.Publisher("/camera1/commands", CameraCommand, queue_size=2)
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
        rospy.loginfo('recording video')
        self._station.recording = True
        self.pubCamera.publish(camera_command=CameraCommand.RECORD)
        rospy.sleep(current_op.duration)
        self.pubCamera.publish(camera_command=CameraCommand.STOPRECORD)
        self._station.recording = False

        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.add_timestamp()

        return current_op

# if __name__ == '__main__':
#     ika_handler = SolubilityStation_Handler()
