#!/usr/bin/env python3

import rospy
from archemist.state.station import Station
from archemist.state.stations.input_station import InputStation
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.processing.handler import StationHandler

class OutputStation_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        rospy.sleep(1)

    def run(self):
        rospy.loginfo(f'{self._station}_handler is running')
        try:
            while not rospy.is_shutdown():
                self.handle()
                rospy.sleep(2)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._station}_handler is terminating!!!')

# if __name__ == '__main__':
#     fisher_handler = FisherWeightingStation_Handler()
