import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import OpenDoorOpDescriptor, CloseDoorOpDescriptor, QuantosDispenseOpDescriptor, MoveCarouselOpDescriptor
from roslabware_msgs.msg import MettlerQuantosQB1Cmd, MettlerQuantosQB1Reading
from archemist.core.processing.handler import StationHandler



'''TODO write a subscriber that publish quantos status such as dispensed weight, door status, sampler position and possibly dispensed solid'''

class QuantosSolidDispenserQB1ROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self._quantos_pub = rospy.Publisher("/mettler_quantos_qB1_commands", MettlerQuantosQB1Cmd, queue_size=1)
        rospy.Subscriber('/mettler_quantos_qB1_info', data_class=MettlerQuantosQB1Reading, callback=self._quantos_callback)
        self._received_results = False
        self._op_results = {}
        rospy.sleep(1)
        
      
    def run(self):
        rospy.loginfo(f'{self._station}_handler is running')
        try:
            while not rospy.is_shutdown():
                self.handle() #The core handler loop
                rospy.sleep(2)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._station}_handler is terminating!!!')



    def execute_op(self):
        current_op = self._station.get_assigned_station_op()
        self._received_results = False
        self._op_results = {}
        if isinstance(current_op, QuantosDispenseOpDescriptor):
            #Dispense solid
                self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.DISPENSE, quantos_tolerance = current_op.tolerance, quantos_amount= current_op.target_mass,
                                       )
        elif isinstance(current_op, OpenDoorOpDescriptor):
            self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.OPEN_FRONT_DOOR)
            self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.OPEN_SIDE_DOORS)
        elif isinstance(current_op, CloseDoorOpDescriptor):
            self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.CLOSE_FRONT_DOOR)
            self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.CLOSE_SIDE_DOORS)
        elif isinstance(current_op, MoveCarouselOpDescriptor):
            self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.MOVE_SAMPLER, quantos_int=current_op.carousel_pos)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unknown operation was received')

    def is_op_execution_complete(self) -> bool:
        return self._received_results

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, self._op_results 

    def _quantos_callback(self, msg:MettlerQuantosQB1Reading):
        self._received_results = True
        self._op_results["success"] = msg.success
        self._op_results["command"] = msg.command_running
        self._op_results["output"] = msg.output
        self._op_results["front_door_open"] = msg.front_door_open
        self._op_results["side_door_open"] = msg.side_door_open
        self._op_results["autosampler_position"] = msg.sampler_pos