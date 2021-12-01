#!/usr/bin/env python3

import rospy
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations import IkaPlateRCTDigital
from archemist.state.stations.ika_place_rct_digital import IKAMode, IKAOutputDescriptor
from archemist.state.state import State
from archemist.processing.handler import StationHandler
from archemist_msgs.msg import IKACommand
from enum import Enum

from rospy.core import is_shutdown

class IkaPlateRCTDigital_Handler(StationHandler):
    def __init__(self):
        super.__init__('IkaPlateRCTDigital')
        rospy.init_node(self._station_name + '_handler')
        self.pubIka = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
        
        print(self._station_name  + '_handler is running')
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)

    def process(self):
        current_op = self._station.assigned_batch.recipe.get_current_task_op()
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
        
        current_op.has_result = True
        current_op.output.success = True
        current_op.output.add_timestamp()

        return current_op


    # def handle(self):
    #     self.state.updateFromDB()
    #     self._ikaState = self.state.getStation('IkaPlateRCTDigital')
    #     if self._ikaState._assigned_batch is not None:
    #         print('inside')
    #         current_op = self._ikaState._assigned_batch.getCurrentOp()
    #         if (current_op.mode == IKAMode.HEATING):
    #             print("heating")
    #             self.pubIka.publish(ika_command= 7, ika_param=current_op.setTemperature)
    #         elif (current_op.mode == IKAMode.STIRRING):
    #             print("stirring")
    #             self.pubIka.publish(ika_command= 6, ika_param=current_op.setStirringSpeed)
    #         elif (current_op.mode == IKAMode.HEATINGSTIRRING):
    #             print("heatingstirring")
    #             self.pubIka.publish(ika_command= 7, ika_param=current_op.setTemperature)
    #             self.pubIka.publish(ika_command= 6, ika_param=current_op.setStirringSpeed)
    #         rospy.sleep(current_op.duration)
    #         self.pubIka.publish(ika_command= 8)
    #         # result = IKAOutputDescriptor(current_op.__class__.__name__)
    #         # result.has_result = True
    #         # result.success = True
    #         # result.add_timestamp()
    #         # current_op.output = result
    #         self._ikaState._assigned_batch.advanceProcessState()
    #         self._ikaState._processed_batch = self._ikaState._assigned_batch
    #         self._ikaState._assigned_batch = None
    #         self.state.modifyObjectDB(self._ikaState)

if __name__ == '__main__':
    ika_handler = IkaPlateRCTDigital_Handler()