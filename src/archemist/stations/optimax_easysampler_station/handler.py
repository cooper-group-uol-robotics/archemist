import rospy
from typing import Dict, Tuple
from archemist.core.processing.handler import StationHandler
from archemist.core.state.station import Station
from roslabware_msgs.msg import MettlerOptimaxCmd, MettlerOptimaxReading, LcmsCmd, LcmsStatus, BaseValveCmd
from .state import SynthesisStation, OptimaxTempStirringOpDescriptor, OptimaxStirringOpDescriptor, OptimaxTempOpDescriptor, OptimaxSamplingOpDescriptor, LcmsOpDescriptor, BaseValveOpDescriptor
from rospy.core import is_shutdown
from std_msgs.msg import Bool

class SynthesisStationROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubOptimax = rospy.Publisher(
            "mettler_optimax", MettlerOptimaxCmd, queue_size=2)
        self.pubLCMS = rospy.Publisher(
            "Lcms", LcmsCmd, queue_size=2)
        rospy.Subscriber("mettler_optimax_info",
                         MettlerOptimaxReading, self.optimax_callback)
        rospy.Subscriber("lcms_info",
                         LcmsStatus, self.lcms_callback)
        rospy.Subscriber('/mettler_optimax/task_complete', Bool, self._optimax_state_update, queue_size=1)
        self._received_results = False
        self._op_results = 0.0
        self._optimax_current_task_complete = False
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
        if (isinstance(current_op, OptimaxTempStirringOpDescriptor)):
            self._optimax_current_task_complete = False
            for i in range(10):
                self.pubOptimax.publish(optimax_command=MettlerOptimaxCmd.ADD_TEMP_STIR, temperature=current_op.temperature,
                                        temp_duration=current_op.temp_duration, stir_speed=current_op.stir_speed, stir_duration=current_op.stir_duration)
        elif (isinstance(current_op, OptimaxTempOpDescriptor)):
            self._optimax_current_task_complete = False
            for i in range(10):
                self.pubOptimax.publish(optimax_command=MettlerOptimaxCmd.ADD_TEMP, temperature=current_op.temperature,
                                        temp_duration=current_op.temp_duration)
        elif (isinstance(current_op, OptimaxStirringOpDescriptor)):
            self._optimax_current_task_complete = False
            for i in range(10):
                self.pubOptimax.publish(optimax_command=MettlerOptimaxCmd.ADD_STIR,
                                        stir_speed=current_op.stir_speed, stir_duration=current_op.stir_duration)
        elif (isinstance(current_op, OptimaxSamplingOpDescriptor)):
            self._optimax_current_task_complete = False
            for i in range(10):
                self.pubOptimax.publish(optimax_command=MettlerOptimaxCmd.ADD_SAMPLE,
                                        dilution = current_op.dilution)
        elif (isinstance(current_op, LcmsOpDescriptor)):
            for i in range(10):
                self.pubLCMS.publish(lcms_command=LcmsCmd.START)
        else:
            rospy.logwarn(
                f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        return self._optimax_current_task_complete

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, self._op_results

    def optimax_callback(self, msg):
        pass

    def lcms_callback(self, msg):
        pass

    def _optimax_state_update(self, msg):
        if msg.data == True:
            self._optimax_current_task_complete = True
        elif msg.data == False:
            self._optimax_current_task_complete = False