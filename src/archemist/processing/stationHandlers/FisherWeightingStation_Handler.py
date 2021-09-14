#!/usr/bin/env python3

import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations.fisher_weighing_station import FisherWeightingStation
from archemist.state.state import State
from fisherbrand_pps4102_balance.msg import BalanceCommand, BalanceReading

from rospy.core import is_shutdown

class FisherWeightingStation_Handler:
    def __init__(self):
        rospy.init_node("FisherWeightingHandler")
        print("FisherWeightingStation running")
        self.coder = rosMsgCoder()
        self.state = State()
        self.state.initializeState(False)
        self._weightingStation= self.state.getStation('FisherWeightingStation')
        self.pub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self.pubFisherScale = rospy.Publisher("/Balance_Commands", BalanceCommand, queue_size=2)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)
        #rospy.spin()

    def handler_cb(self):
        pass

    def handle(self):
        self.state.updateFromDB()
        self._weightingStation = self.state.getStation('FisherWeightingStation')
        if self._weightingStation._assigned_batch is not None:
            print('starting to handle')
            stationOp = self._weightingStation._assigned_batch.getCurrentOp()

            self.pubFisherScale.publish(balance_command=BalanceCommand.WEIGHT_STABLE)
            result = rospy.wait_for_message('/Balance_Weights',BalanceReading)
            print(result.weight)
            rospy.sleep(1.5)

            self._weightingStation._assigned_batch.advanceProcessState()
            self._weightingStation._processed_batch = self._weightingStation._assigned_batch
            self._weightingStation._assigned_batch = None
            self.state.modifyObjectDB(self._weightingStation)

if __name__ == '__main__':
    fisher_handler = FisherWeightingStation_Handler()
