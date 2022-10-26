import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from archemist.core.state.stations.solid_dispensing_quantos_QS2 import OpenDoorOpDescriptor, CloseDoorOpDescriptor, QuantosDispenseOpDescriptor, MoveCarouselOpDescriptor
from mettler_toledo_quantos_q2_msgs.msg import QuantosCommand
from archemist.core.processing.handler import StationHandler
from std_msgs.msg import String


'''TODO write a subscriber that publish quantos status such as dispensed weight, door status, sampler position and possibly dispensed solid'''

class QuantosSolidDispenserQS2ROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self._quantos_pub = rospy.Publisher("/Quantos_Commands", QuantosCommand, queue_size=1)
        rospy.Subscriber('/Quantos_Done', String, self._quantos_callback)
        self._received_results = False
        
        # put quantos in the correct place before robott
        # rospy.sleep(3)
        # self._quantos_pub.publish(quantos_command=QuantosCommand.SETSAMPLEPOS, quantos_int= 20)
        # rospy.wait_for_message("/Quantos_Done", String)
        # self._station.carousel_pos = 20

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
        if isinstance(current_op, QuantosDispenseOpDescriptor):
            #Dispense solid
            self._quantos_pub.publish(quantos_command=QuantosCommand.DISPENSESOLID, quantos_int= self._station.carousel_pos, quantos_float= current_op.dispense_mass)
        elif isinstance(current_op, OpenDoorOpDescriptor):
            self._quantos_pub.publish(quantos_command=QuantosCommand.MOVEDOOR, quantos_bool=True)
        elif isinstance(current_op, CloseDoorOpDescriptor):
            self._quantos_pub.publish(quantos_command=QuantosCommand.MOVEDOOR, quantos_bool=False)
        elif isinstance(current_op, MoveCarouselOpDescriptor):
            self._quantos_pub.publish(quantos_command=QuantosCommand.SETSAMPLEPOS, quantos_int=current_op.carousel_pos)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool:
        return self._received_results

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {} 

    def _quantos_callback(self, msg):
        self._received_results = False