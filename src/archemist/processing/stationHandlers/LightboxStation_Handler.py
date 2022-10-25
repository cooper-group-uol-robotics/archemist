#!/usr/bin/env python3

import rospy
from typing import Dict, Tuple
from archemist.processing.handler import StationHandler
from archemist.state.station import Station
from colorimetry_msgs.msg import ColorimetryCommand,ColorimetryResult

from rospy.core import is_shutdown

class LightBoxStation_Handler(StationHandler):
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubCamera = rospy.Publisher("/colorimetry_station/command", ColorimetryCommand, queue_size=1)
        rospy.Subscriber("/colorimetry_station/result", ColorimetryResult, self._colorimetry_callback)
        self._received_results = False
        self._colorimetry_results = {}
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

        op_msg = ColorimetryCommand()
        op_msg.op_name = 'take_pic'
        self._received_results = False
        self._colorimetry_results = {}
        for i in range(10):
            self.pubCamera.publish(op_msg)

    def is_op_execution_complete(self) -> bool:
        return self._received_results

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, self._colorimetry_results

    def _colorimetry_callback(self, msg: ColorimetryResult):
        self._received_results = True
        self._colorimetry_results['result_filename'] = msg.result_file_name
        self._colorimetry_results['red_intensity'] = msg.red
        self._colorimetry_results['green_intensity'] = msg.green
        self._colorimetry_results['blue_intensity'] = msg.blue

    
