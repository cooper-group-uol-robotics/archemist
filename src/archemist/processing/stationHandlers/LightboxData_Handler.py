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
        
    def process(self):
        def messageSubscriber():
            rospy.init_node('"/lightboxPi/data"', anonymous=False)

        if __name__ == '__main__':
            try:
                messageSubscriber()
            except rospy.ROSInterruptException:
                pass

        #Need to add listener for recciving message for the data 
