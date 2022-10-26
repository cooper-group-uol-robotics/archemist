#!/usr/bin/env python3

import rospy
from typing import Dict, Tuple
from archemist.processing.handler import StationHandler
from archemist.state.station import Station
from colorimetry_msgs.msg import ColorimetryCommand,ColorimetryResult
from archemist.state.stations.light_box_station import SampleColorOpDescriptor

class LightBoxROSHandler(StationHandler):
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubCamera = rospy.Publisher("/colorimetry_station/command", ColorimetryCommand, queue_size=1)
        rospy.Subscriber("/colorimetry_station/result", ColorimetryResult, self._colorimetry_callback)
        self._received_results = False
        self._op_results = {}
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
        self._received_results = False
        self._op_results = {}
        if isinstance(current_op, SampleColorOpDescriptor):
            op_msg = ColorimetryCommand()
            op_msg.op_name = 'take_pic'
            for i in range(10):
                self.pubCamera.publish(op_msg)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        return self._received_results

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, self._op_results

    def _colorimetry_callback(self, msg: ColorimetryResult):
        self._received_results = True
        self._op_results['result_filename'] = msg.result_file_name
        self._op_results['red_intensity'] = msg.red
        self._op_results['green_intensity'] = msg.green
        self._op_results['blue_intensity'] = msg.blue

    
