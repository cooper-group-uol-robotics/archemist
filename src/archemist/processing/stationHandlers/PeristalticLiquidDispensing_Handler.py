#!/usr/bin/env python3

import rospy
from archemist.state.station import Station
from pi4_peristaltic_pump_msgs.msg import DispenserCommand
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.processing.handler import StationHandler
from std_msgs.msg import String

class PeristalticLiquidDispensing_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self._pubPeristaltic = rospy.Publisher("/Dispenser_Commands", DispenserCommand, queue_size=1)

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

        self._pubPeristaltic.publish(dispenser_command=DispenserCommand.DISPENSEPID, dispenser_ml=current_op.dispense_volume)
        rospy.wait_for_message("/Dispenser_Done", String)
        
        self._station.dispense_liquid(current_op.liquid_name, current_op.dispense_volume)
        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.add_timestamp()

        return current_op

# if __name__ == '__main__':
#     ika_handler = PeristalticLiquidDispensing_Handler()