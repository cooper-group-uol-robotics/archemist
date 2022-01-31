#!/usr/bin/env python3

import rospy
from archemist.state.station import Station
from archemist.state.stations import IkaPlateRCTDigital
from archemist.state.stations.ika_place_rct_digital import IKAMode, IKAOutputDescriptor
from archemist.persistence.objectConstructor import ObjectConstructor
from archemist.processing.handler import StationHandler
from archemist_msgs.msg import IKACommand
from enum import Enum

from rospy.core import is_shutdown

class IkaPlateRCTDigital_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubIka = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
        
        print(f'{self._station }_handler is running')
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)

    def process(self):
        current_op_dict = self._station.assigned_batch.recipe.get_current_task_op_dict()
        current_op = ObjectConstructor.construct_station_op_from_dict(current_op_dict)
        current_op.add_timestamp()
        if (current_op.mode == IKAMode.HEATING):
            print("heating")
            self.pubIka.publish(ika_command= 7, ika_param=current_op.set_temperature)
        elif (current_op.mode == IKAMode.STIRRING):
            print("stirring")
            self.pubIka.publish(ika_command= 6, ika_param=current_op.set_stirring_speed)
        elif (current_op.mode == IKAMode.HEATINGSTIRRING):
            print("heatingstirring")
            self.pubIka.publish(ika_command= 7, ika_param=current_op.set_temperature)
            self.pubIka.publish(ika_command= 6, ika_param=current_op.set_stirring_speed)
        rospy.sleep(current_op.duration)
        self.pubIka.publish(ika_command= 8)
        
        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.add_timestamp()

        return current_op

# if __name__ == '__main__':
#     ika_handler = IkaPlateRCTDigital_Handler()