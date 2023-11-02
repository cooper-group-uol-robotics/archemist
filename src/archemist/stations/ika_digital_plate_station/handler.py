from archemist.core.state.station import Station
from .state import IKAHeatStirBatchOp, IKAHeatBatchOp, IKAStirBatchOp
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from archemist.core.util.enums import OpOutcome
from archemist.core.state.station_op_result import ProcessOpResult
from threading import Thread
from typing import Tuple, List

class SimIKAPlateDigitalHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[ProcessOpResult]]:
        current_op = self._station.assigned_op
        parameters = {}
        if isinstance(current_op, IKAHeatStirBatchOp):
            parameters["target_temperature"]  = current_op.target_temperature
            parameters["target_stirring_speed"]  = current_op.target_stirring_speed
            parameters["duration"]  = current_op.duration
            parameters["duration_unit"]  = current_op.duration_unit
        elif isinstance(current_op, IKAStirBatchOp):
            parameters["target_stirring_speed"]  = current_op.target_stirring_speed
            parameters["duration"]  = current_op.duration
            parameters["duration_unit"]  = current_op.duration_unit
        elif isinstance(current_op, IKAHeatBatchOp):
            parameters["target_temperature"]  = current_op.target_temperature
            parameters["duration"]  = current_op.duration
            parameters["duration_unit"]  = current_op.duration_unit
        op_result = ProcessOpResult.from_args(origin_op=current_op.object_id,
                                                parameters=parameters)
        return OpOutcome.SUCCEEDED, [op_result]

try:
    import rospy
    from ika_plate_rct_digital_msgs.msg import IKACommand
    
    class IKAPlateDigitalROSHandler(StationOpHandler):
        def __init__(self, station: Station):
            super().__init__(station)
        
        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self._ika_pub = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
            self._timer_thread = None
            rospy.sleep(1)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            if isinstance(current_op, IKAHeatStirBatchOp):
                rospy.loginfo("executing heating operation")
                for i in range(10):
                    self._ika_pub.publish(ika_command= IKACommand.HEATAT, ika_param=current_op.target_temperature)
            elif isinstance(current_op, IKAStirBatchOp):
                rospy.loginfo("executing stirring operation")
                for i in range(10):
                    self._ika_pub.publish(ika_command= IKACommand.STIRAT, ika_param=current_op.target_stirring_speed)
            elif isinstance(current_op, IKAHeatBatchOp):
                rospy.loginfo("executing heating and stirring operation")
                for i in range(10):
                    self._ika_pub.publish(ika_command= IKACommand.HEATAT, ika_param=current_op.target_temperature)
                    self._ika_pub.publish(ika_command= IKACommand.STIRAT, ika_param=current_op.target_stirring_speed)
            else:
                rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')
            
            kwargs = {
                'duration':current_op.duration,
                'duration_unit': current_op.duration_unit
                }
            self._timer_thread = Thread(target=self._sleep_for_duration,kwargs=kwargs)
            self._timer_thread.start()


        def is_op_execution_complete(self) -> bool:
            if self._timer_thread.is_alive():
                return False
            else:
                return True

        def get_op_result(self) -> Tuple[OpOutcome, List[ProcessOpResult]]:
            current_op = self._station.assigned_op
            parameters = {}
            if isinstance(current_op, IKAHeatStirBatchOp):
                parameters["target_temperature"]  = current_op.target_temperature
                parameters["target_stirring_speed"]  = current_op.target_stirring_speed
                parameters["duration"]  = current_op.duration
                parameters["duration_unit"]  = current_op.duration_unit
            elif isinstance(current_op, IKAStirBatchOp):
                parameters["target_stirring_speed"]  = current_op.target_stirring_speed
                parameters["duration"]  = current_op.duration
                parameters["duration_unit"]  = current_op.duration_unit
            elif isinstance(current_op, IKAHeatBatchOp):
                parameters["target_temperature"]  = current_op.target_temperature
                parameters["duration"]  = current_op.duration
                parameters["duration_unit"]  = current_op.duration_unit
            op_result = ProcessOpResult.from_args(origin_op=current_op.object_id,
                                                  parameters=parameters)
            return OpOutcome.SUCCEEDED, [op_result]

        def _sleep_for_duration(self, **kwargs):
            if kwargs["duration_unit"] == "second":
                total_seconds = kwargs['duration']
            elif kwargs["duration_unit"] == "minute":
                total_seconds = kwargs['duration']*60
            elif kwargs["duration_unit"] == "hour":
                total_seconds = kwargs['duration']*60*60
            
            rospy.sleep(total_seconds)
            for i in range(10):
                self._ika_pub.publish(ika_command= IKACommand.ALLOFF)

except ImportError:
    pass
