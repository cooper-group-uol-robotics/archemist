import rospy
from typing import Dict, Tuple
from archemist.core.processing.handler import StationHandler
from archemist.core.state.station import Station
from colorimetry_msgs.msg import ColorimetryCommand,ColorimetryRGBResult, ColorimetryLABResult
from .state import SampleColorLABOpDescriptor, SampleColorRGBOpDescriptor

class LightBoxROSHandler(StationHandler):
    def __init__(self, station:Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubCamera = rospy.Publisher("/colorimetry_station/command", ColorimetryCommand, queue_size=1)
        rospy.Subscriber("/colorimetry_station/result_rgb", ColorimetryRGBResult, self._colorimetry_rgb_callback)
        rospy.Subscriber("/colorimetry_station/result_lab", ColorimetryLABResult, self._colorimetry_lab_callback)
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
        if isinstance(current_op, SampleColorRGBOpDescriptor):
            op_msg = ColorimetryCommand()
            op_msg.op_name = 'rgb'
            for i in range(10):
                self.pubCamera.publish(op_msg)
        elif isinstance(current_op, SampleColorLABOpDescriptor):
            op_msg = ColorimetryCommand()
            op_msg.op_name = 'lab'
            for i in range(10):
                self.pubCamera.publish(op_msg)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        return self._received_results

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, self._op_results

    def _colorimetry_rgb_callback(self, msg: ColorimetryRGBResult):
        self._received_results = True
        self._op_results['result_filename'] = msg.result_file_name
        self._op_results['red_intensity'] = msg.red_intensity
        self._op_results['blue_intensity'] = msg.blue_intensity
        self._op_results['green_intensity'] = msg.green_intensity
        self._op_results['color_index'] = msg.color_index

    def _colorimetry_lab_callback(self, msg: ColorimetryLABResult):
        self._received_results = True
        self._op_results['result_filename'] = msg.result_file_name
        self._op_results['l_value'] = msg.L_value
        self._op_results['a_value'] = msg.a_value
        self._op_results['b_value'] = msg.b_value
        self._op_results['color_index'] = msg.color_index

    
