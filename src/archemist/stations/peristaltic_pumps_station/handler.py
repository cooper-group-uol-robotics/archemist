from typing import Tuple, List
from archemist.core.state.station import Station
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult
from .state import PPLiquidDispenseOp
from archemist.core.util.enums import OpOutcome


class SimPeristalticPumpsStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
        op = self._station.assigned_op
        result = MaterialOpResult.from_args(origin_op=op.object_id,
                                            material_names=[op.liquid_name],
                                            amounts=[op.dispense_volume],
                                            units=[op.dispense_unit])
        return OpOutcome.SUCCEEDED, [result]


try:
    import rospy
    from pi4_peristaltic_pump_msgs.msg import DispenserCommand
    from std_msgs.msg import String

    class PeristalticPumpsStationROSHandler(StationOpHandler):
        def __init__(self, station: Station):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self._pump_pub = rospy.Publisher(
                "/Dispenser_Commands", DispenserCommand, queue_size=1)
            rospy.Publisher('/Dispenser_Done', String, self._pump_callback)
            self._op_complete = False
            rospy.sleep(1)

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_complete = False
            if isinstance(current_op, PPLiquidDispenseOp):
                # TODO depending on the liquid we can select the correct pump
                # pump_index = self._station.liquid_pump_map[current_op.liquid_name]
                for i in range(10):
                    self._pump_pub.publish(
                        dispenser_command=DispenserCommand.DISPENSEPID, dispenser_ml=current_op.dispense_volume)
            else:
                rospy.logwarn(
                    f'[{self.__class__.__name__}] Unkown operation was received')

        def is_op_execution_complete(self) -> bool:
            return self._op_complete

        def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
            op = self._station.assigned_op
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                material_names=[
                                                    op.liquid_name],
                                                amounts=[op.dispense_volume],
                                                units=[op.dispense_unit])
            return OpOutcome.SUCCEEDED, [result]

        def _pump_callback(self, msg):
            self._op_complete = True

except ImportError:
    pass
