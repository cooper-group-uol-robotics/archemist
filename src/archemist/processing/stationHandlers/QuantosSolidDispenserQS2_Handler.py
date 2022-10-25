#!/usr/bin/env python3

import rospy
from archemist.state.station import Station
from mettler_toledo_quantos_q2_msgs.msg import QuantosCommand
from archemist.processing.handler import StationHandler
from std_msgs.msg import String

class QuantosSolidDispenserQS2_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self._pubQuantos = rospy.Publisher("/Quantos_Commands", QuantosCommand, queue_size=1)
        
        # put quantos in the correct place before robott
        rospy.sleep(3)
        self._pubQuantos.publish(quantos_command=QuantosCommand.SETSAMPLEPOS, quantos_int= 20)
        rospy.wait_for_message("/Quantos_Done", String)
        self._station.carousel_pos = 20

    def run(self):
        rospy.loginfo(f'{self._station}_handler is running')
        try:
            while not rospy.is_shutdown():
                self.handle()
                rospy.sleep(2)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._station}_handler is terminating!!!')



    def process(self):
        current_op_dict = self._station.assigned_batch.recipe.get_current_task_op_dict()
        current_op = ObjectConstructor.construct_station_op_from_dict(current_op_dict)
        current_op.add_timestamp()

        #Dispense solid
        self._pubQuantos.publish(quantos_command=QuantosCommand.DISPENSESOLID, quantos_int= 5, quantos_float= 200.0)
        self._station.carousel_pos = 5
        rospy.wait_for_message("/Quantos_Done", String)
        
        #Move carousel back to the start position
        self._pubQuantos.publish(quantos_command=QuantosCommand.SETSAMPLEPOS, quantos_int= 20)
        rospy.wait_for_message("/Quantos_Done", String)
        self._station.carousel_pos = 20
        #Well 5 exposed to KUKA, unload filed vial
        cartridge_id = self._station.get_cartridge_id(current_op.solid_name)
        self._station.dispense(cartridge_id, current_op.dispense_mass)

        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.add_timestamp()

        return current_op

        

# if __name__ == '__main__':
#     ika_handler = QuantosSolidDispenserQS2_Handler()