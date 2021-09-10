#!/usr/bin/env python3

import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations import IkaPlateRCTDigital
from archemist.state.stations.ika_place_rct_digital import IKAMode
from archemist.persistence.dbHandler import dbHandler
from archemist_msgs.msg import IKACommand
from enum import Enum

class IkaPlateRCTDigital_Handler:
    def __init__(self):
        rospy.init_node("IKAPlateHandler")
        print("IKAPlateHandler running")
        self.coder = rosMsgCoder()
        self._ikaState = IkaPlateRCTDigital(123, None)
        #dbhandler = dbHandler.dbHandler()
        self.pub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self.pubIka = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        rospy.spin()

    def handler_cb(self, msg):
        print("got something")
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
