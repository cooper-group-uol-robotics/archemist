
from typing import Tuple, List
from archemist.core.state.station import Station
from .state import (QuantosOpenDoorOp,
                    QuantosCloseDoorOp,
                    QuantosMoveCarouselOp,
                    QuantosDispenseOp,
                    QuantosLoadCartridgeOp,
                    QuantosUnloadCartridgeOp)
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult
from archemist.core.util.enums import OpOutcome

class SimQuantosSolidDispenserQS2Handler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
            op = self._station.assigned_op
            if isinstance(op, QuantosDispenseOp):
                result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                            material_names=[op.solid_name],
                                                            amounts=[op.dispense_mass],
                                                            units=[op.dispense_unit])
                return OpOutcome.SUCCEEDED, [result]
            else:
                return OpOutcome.SUCCEEDED, None

try:
    import rospy
    from std_msgs.msg import String
    from mettler_toledo_quantos_q2_msgs.msg import QuantosCommand

    
    '''TODO write a subscriber that publish quantos status such as dispensed weight, door status, sampler position and possibly dispensed solid'''

    class QuantosSolidDispenserQS2ROSHandler(StationOpHandler):
        def __init__(self, station: Station):
            super().__init__(station)
            
            
        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self._quantos_pub = rospy.Publisher("/Quantos_Commands", QuantosCommand, queue_size=1)
            rospy.Subscriber('/Quantos_Done', String, self._quantos_callback)
            self._received_results = False
            # put quantos in the correct place before robott
            rospy.sleep(3)
            self._quantos_pub.publish(quantos_command=QuantosCommand.SETSAMPLEPOS, quantos_int= 20)
            try:
                rospy.wait_for_message("/Quantos_Done", String, timeout=30)
                return True
            except rospy.ROSException:
                return False

        def execute_op(self):
            current_op = self._station.assigned_op
            self._received_results = False
            if isinstance(current_op, QuantosDispenseOp):
                #Dispense solid
                # TODO need to handle dispense unit
                self._quantos_pub.publish(quantos_command=QuantosCommand.DISPENSESOLID, quantos_int= self._station.carousel_pos, quantos_float= current_op.dispense_mass)
            elif isinstance(current_op, QuantosOpenDoorOp):
                self._quantos_pub.publish(quantos_command=QuantosCommand.MOVEDOOR, quantos_bool=True)
            elif isinstance(current_op, QuantosCloseDoorOp):
                self._quantos_pub.publish(quantos_command=QuantosCommand.MOVEDOOR, quantos_bool=False)
            elif isinstance(current_op, QuantosMoveCarouselOp):
                self._quantos_pub.publish(quantos_command=QuantosCommand.SETSAMPLEPOS, quantos_int=current_op.target_pos)
            elif isinstance(current_op, QuantosLoadCartridgeOp) or isinstance(current_op, QuantosUnloadCartridgeOp):
                 self._received_results = True
            else:
                rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

        def is_op_execution_complete(self) -> bool:
            return self._received_results

        def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
            op = self._station.assigned_op
            if isinstance(op, QuantosDispenseOp):
                result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                            material_names=[op.solid_name],
                                                            amounts=[op.dispense_mass],
                                                            units=[op.dispense_unit])
                return OpOutcome.SUCCEEDED, [result]
            else:
                return OpOutcome.SUCCEEDED, None

        def _quantos_callback(self, msg):
            self._received_results = False

except ImportError:
    pass