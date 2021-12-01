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
        # put quantos in the correct place before robot
        rospy.sleep(3)
        self._pubQuantos.publish(quantos_command=8, quantos_int= 20)
        rospy.wait_for_message("/Quantos_Done", String)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)

        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)

    def handler_cb(self):
        pass


    def handle(self):
        self.state.updateFromDB()
        self.quantos = self.state.getStation('QuantosSolidDispenserQS2')
        
        if self.quantos._assigned_batch is not None:
            print ('got work to do')
            current_op = self.quantos._assigned_batch.getCurrentOp()
            current_op.add_timestamp()

            # self._pubQuantos.publish(quantos_command=8, quantos_int= 20)
            # rospy.wait_for_message("/Quantos_Done", String)
            #Well 5 exposed to KUKA, load in empty vial

            #Dispense solid
            self._pubQuantos.publish(quantos_command=20, quantos_int= 5, quantos_float= 200.0)
            rospy.wait_for_message("/Quantos_Done", String)
            
            self._pubQuantos.publish(quantos_command=8, quantos_int= 20)
            rospy.wait_for_message("/Quantos_Done", String)
            #Well 5 exposed to KUKA, unload filed vial

            self.quantos._assigned_batch.advanceProcessState()
            self.quantos._processed_batch = self.quantos._assigned_batch
            self.quantos._assigned_batch = None
            self.state.modifyObjectDB(self.quantos)

if __name__ == '__main__':
    ika_handler = QuantosSolidDispenserQS2_Handler()