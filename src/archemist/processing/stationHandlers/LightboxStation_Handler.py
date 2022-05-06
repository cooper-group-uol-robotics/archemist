#!/usr/bin/env python3

import rospy
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.state.station import Station
from lightbox_msgs.msg import LightboxCommand

from rospy.core import is_shutdown

class LightboxStaion_Handler:
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubCamera = rospy.Publisher("/lightboxPi/commands", LightboxCommand, queue_size=2)
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

        #Wake the station 
        self.pubCamera.publish(lightbox_command=LightboxCommand.PROCESS_SAMPLE)
        rospy.wait_for_message(";/", LightboxCommand)

        #Vial Is ready 
        self.pubCamera.publish(lightbox_command=LightboxCommand.)
        rospy.wait_for_message("/VIAL_DONE", LightboxCommand)

        #Batch is done 
        self.pubCamera.publish(lightbox_command=LightboxCommand.SLEEP)
