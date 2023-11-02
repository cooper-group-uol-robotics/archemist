from typing import List, Tuple, Union
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from .state import LightBoxStation
from .state import LBSampleAnalyseRGBOp, LBSampleAnalyseLABOp, LBAnalyseRGBResult, LBAnalyseLABResult
from archemist.core.util.enums import OpOutcome
import random

class SimLightBoxHandler(SimStationOpHandler):
    def __init__(self, station:LightBoxStation):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[Union[LBAnalyseRGBResult, LBAnalyseLABResult]]]:
        current_op = self._station.assigned_op
        if isinstance(current_op, LBSampleAnalyseRGBOp):
            rgb_target_index = self._station.rgb_target_index
            result = LBAnalyseRGBResult.from_args(origin_op=current_op.object_id,
                                                       r_value=random.randint(0, 127),
                                                       g_value=random.randint(0, 127),
                                                       b_value=random.randint(0, 127),
                                                       color_index=random.randint(0, 255),
                                                       target_index=rgb_target_index,
                                                       result_filename=f"pic{random.randint(0, 100)}.png")
        elif isinstance(current_op, LBSampleAnalyseLABOp):
            lab_target_index = self._station.lab_target_index
            result = LBAnalyseLABResult.from_args(origin_op=current_op.object_id,
                                                    l_value=random.randint(0, 100),
                                                    a_value=random.randint(-128, 127),
                                                    b_value=random.randint(-128, 127),
                                                    color_index=random.random()*100,
                                                    target_index=lab_target_index,
                                                    result_filename=f"pic{random.randint(0, 100)}.png")
        return OpOutcome.SUCCEEDED, [result]

try:
    import rospy
    from colorimetry_msgs.msg import ColorimetryCommand,ColorimetryRGBResult, ColorimetryLABResult
    
    class LightBoxROSHandler(StationOpHandler):
        def __init__(self, station:LightBoxStation):
            super().__init__(station)
            

        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self.pubCamera = rospy.Publisher("/colorimetry_station/command", ColorimetryCommand, queue_size=1)
            rospy.Subscriber("/colorimetry_station/result_rgb", ColorimetryRGBResult, self._colorimetry_rgb_callback)
            rospy.Subscriber("/colorimetry_station/result_lab", ColorimetryLABResult, self._colorimetry_lab_callback)
            self._op_result = None
            rospy.sleep(1)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_result = None
            if isinstance(current_op, LBSampleAnalyseRGBOp):
                op_msg = ColorimetryCommand()
                op_msg.op_name = 'rgb'
                for i in range(10):
                    self.pubCamera.publish(op_msg)
            elif isinstance(current_op, LBSampleAnalyseLABOp):
                op_msg = ColorimetryCommand()
                op_msg.op_name = 'lab'
                for i in range(10):
                    self.pubCamera.publish(op_msg)
            else:
                rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

        def is_op_execution_complete(self) -> bool:
            return self._op_result is not None

        def get_op_result(self) -> Tuple[OpOutcome, List[Union[LBAnalyseRGBResult, LBAnalyseLABResult]]]:
            return OpOutcome.SUCCEEDED, self._op_results
        
        def shut_down(self):
            pass

        def _colorimetry_rgb_callback(self, msg: ColorimetryRGBResult):
            origin_op_object_id = self._station.assigned_op.object_id
            rgb_target_index = self._station.rgb_target_index
            self._op_result = LBAnalyseRGBResult.from_args(origin_op=origin_op_object_id,
                                                        r_value=msg.red_intensity,
                                                        g_value=msg.green_intensity,
                                                        b_value=msg.blue_intensity,
                                                        color_index=msg.color_index,
                                                        target_index=rgb_target_index,
                                                        result_filename=msg.result_file_name)


        def _colorimetry_lab_callback(self, msg: ColorimetryLABResult):
            origin_op_object_id = self._station.assigned_op.object_id
            lab_target_index = self._station.lab_target_index
            self._op_result = LBAnalyseLABResult.from_args(origin_op=origin_op_object_id,
                                                        l_value=msg.L_value,
                                                        a_value=msg.a_value,
                                                        b_value=msg.b_value,
                                                        color_index=msg.color_index,
                                                        target_index=lab_target_index,
                                                        result_filename=msg.result_file_name)
except ImportError:
    pass

    
