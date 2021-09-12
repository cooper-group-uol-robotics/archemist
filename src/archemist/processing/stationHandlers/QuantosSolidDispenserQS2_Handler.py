#!/usr/bin/env python3

import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.state import State
from archemist.state.stations.solid_dispensing_quantos_QS2 import QuantosSolidDispenserQS2, QuantosDispenseOpDescriptor, QuantosOutputDescriptor
from archemist.persistence.dbHandler import dbHandler
from archemist_msgs.msg import QuantosCommand
from archemist.state.material import Solid
from datetime import datetime
from rospy.core import is_shutdown
from std_msgs.msg import String

class QuantosSolidDispenserQS2_Handler:
    def __init__(self):
        # self.persistence = persistenceManager()
        # self.state = persistenceManager().pull()
        # self.stationState = None
        # for station in self.state
        #     if (self.state[station] == "QuantosSolidDispenserQS2"):
        #         self.stationState = station
        rospy.init_node("QuantosDispensingHandler")
        print("Quantos Handler Running")
        self.state = State()
        self.state.initializeState(False)
        self._ackpub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self._pubQuantos = rospy.Publisher("/Quantos_Commands", QuantosCommand, queue_size=1)
        self._pubQuantos.publish(quantos_command=20, quantos_int=opDescriptor, quantos_float=opDescriptor.dispense_mass())
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)

        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)

    def handler_cb(self, msg):
        if(msg.station_name == self.stationState.__class__.__name__ and msg.station_id == self.stationState.id):
            rospy.loginfo("Receiving Handler")
            opDescriptor = self.coder.decode(msg.opDescriptor)
            self.stationState.setStationOp(opDescriptor)
            #dbhandler.updateStationState("ika_plate_rct_digital", descriptor)
            if (opDescriptor.__class__.__name__ == 'QuantosDispenseOpDescriptor'):
                print("dispensing solid")
                self._pubQuantos.publish(quantos_command=20, quantos_int=opDescriptor., quantos_float=opDescriptor.dispense_mass())
                rospy.wait_for_message("/Dispenser_Done", String)
                print(PeristalticPumpOutputDescriptor(opDescriptor.__class__.__name__, True))
                    # send back to the central
                # elif (pump_id == 'p2'):
                #   publish to another pump or indicate that in your message using pumpid field

    def handle(self):
        self.state.updateFromDB()
        self.quantos = self.state.getStation('QuantosSolidDispenserQS2')
        
        if self.quantos._assigned_batch is not None:
            current_op = self.quantos._assigned_batch.getCurrentOp()
            current_op.addTimeStamp()
            
            self._pubQuantos.publish(quantos_command=20, quantos_int=opDescriptor., quantos_float=opDescriptor.dispense_mass())
            rospy.wait_for_message("/Dispenser_Done", String)

            # write the quantos logic here

            self.quantos._assigned_batch.advanceProcessState()
            self.quantos._processed_batch = self.quantos._assigned_batch
            self.quantos._assigned_batch = None
            self.state.modifyObjectDB(self.quantos)