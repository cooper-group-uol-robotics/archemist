#!/usr/bin/env python3

import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations import IkaPlateRCTDigital
from archemist.state.stations.ika_place_rct_digital import IKAMode, IKAOutputDescriptor
from archemist.state.state import State
from archemist_msgs.msg import IKACommand
from enum import Enum

from rospy.core import is_shutdown

class IkaPlateRCTDigital_Handler:
    def __init__(self):
        rospy.init_node("IKAPlateHandler")
        print("IKAPlateHandler running")
        self.coder = rosMsgCoder()
        self.state = State()
        self.state.initializeState(False)
        self._ikaState = self.state.getStation('IkaPlateRCTDigital')
        self.pub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self.pubIka = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)
        #rospy.spin()

    def handler_cb(self, msg):
        if(msg.station_name == self._ikaState.__class__.__name__ and msg.station_id == self._ikaState.id):
            rospy.loginfo("Receiving Handler")
            print("got something")
            rospy.loginfo(vars(self.coder.decode(msg.opDescriptor)))
            print(vars(self.coder.decode(msg.opDescriptor)))
            descriptor = self.coder.decode(msg.opDescriptor)
            #self._ikaState.setStationOp(descriptor)
            #dbhandler.updateStationState("ika_plate_rct_digital", descriptor)
            print(descriptor.mode)
            if (descriptor.mode == IKAMode.HEATING):
                print("heating")
                self.pubIka.publish(ika_command= 7, ika_param=descriptor.setTemperature)
            elif (descriptor.mode == IKAMode.STIRRING):
                print("stirring")
                self.pubIka.publish(ika_command= 6, ika_param=descriptor.setStirringSpeed)
            elif (descriptor.mode == IKAMode.HEATINGSTIRRING):
                print("heatingstirring")
                self.pubIka.publish(ika_command= 7, ika_param=descriptor.setTemperature)
                self.pubIka.publish(ika_command= 6, ika_param=descriptor.setStirringSpeed)
            rospy.sleep(descriptor.duration)
            self.pubIka.publish(ika_command= 8)

    def handle(self):
        self.state.updateFromDB()
        self._ikaState = self.state.getStation('IkaPlateRCTDigital')
        if self._ikaState._assigned_batch is not None:
            current_op = self._ikaState._assigned_batch.getCurrentOp()
            current_op.addTimeStamp()
            if (current_op.mode == IKAMode.HEATING):
                print("heating")
                self.pubIka.publish(ika_command= 7, ika_param=current_op.setTemperature)
            elif (current_op.mode == IKAMode.STIRRING):
                print("stirring")
                self.pubIka.publish(ika_command= 6, ika_param=current_op.setStirringSpeed)
            elif (current_op.mode == IKAMode.HEATINGSTIRRING):
                print("heatingstirring")
                self.pubIka.publish(ika_command= 7, ika_param=current_op.setTemperature)
                self.pubIka.publish(ika_command= 6, ika_param=current_op.setStirringSpeed)
            rospy.sleep(current_op.duration)
            self.pubIka.publish(ika_command= 8)
            # result = IKAOutputDescriptor(current_op.__class__.__name__)
            # result.has_result = True
            # result.success = True
            # result.addTimeStamp()
            # current_op.output = result
            self._ikaState._assigned_batch.advanceProcessState()
            self._ikaState._processed_batch = self._ikaState._assigned_batch
            self._ikaState._assigned_batch = None
            self.state.modifyObjectDB(self._ikaState)

