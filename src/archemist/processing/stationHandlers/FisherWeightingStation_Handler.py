#!/usr/bin/env python3

import rospy
from archemist.state.stations.fisher_weighing_station import FisherWeightingStation
from archemist.state.state import State
from fisherbrand_pps4102_balance.msg import BalanceCommand, BalanceReading
from archemist.processing.handler import StationHandler

class FisherWeightingStation_Handler(StationHandler):
    def __init__(self):
        super.__init__('FisherWeightingStation')
        rospy.init_node(self._station_name + '_handler')
        self.pubFisherScale = rospy.Publisher("/Balance_Commands", BalanceCommand, queue_size=2)

        print(self._station_name  + '_handler is running')
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)

    def process(self):
        current_op = self._station.assigned_batch.recipe.get_current_task_op()
        current_op.add_timestamp()
        print('reading stable weight')
        self.pubFisherScale.publish(balance_command=BalanceCommand.WEIGHT_STABLE)
        balance_reading = rospy.wait_for_message('/Balance_Weights',BalanceReading)
        
        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.weight = balance_reading.weight
        current_op.output.addTimeStamp()

        return current_op

    # def handle(self):
    #     self._state.updateFromDB() # maybe this can be changed to retrieve only one entity/might be problamtic if multiple stations access the same batch
    #     self._weightingStation = self.state.getStation('FisherWeightingStation')
    #     if self._weightingStation._assigned_batch is not None:
    #         print('starting to handle')
    #         stationOp = self._weightingStation._assigned_batch.getCurrentOp()

    #         self.pubFisherScale.publish(balance_command=BalanceCommand.WEIGHT_STABLE)
    #         result = rospy.wait_for_message('/Balance_Weights',BalanceReading)
    #         print(result.weight)
    #         rospy.sleep(1.5)

    #         self._weightingStation._assigned_batch.advanceProcessState()
    #         self._weightingStation._processed_batch = self._weightingStation._assigned_batch
    #         self._weightingStation._assigned_batch = None
    #         self._state.modifyObjectDB(self._weightingStation)

if __name__ == '__main__':
    fisher_handler = FisherWeightingStation_Handler()
