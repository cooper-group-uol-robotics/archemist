#!/usr/bin/env python3

import rospy
from archemist.state.station import Station
from archemist.state.stations.fisher_weighing_station import FisherWeightingStation
from archemist.persistence.objectConstructor import ObjectConstructor
from fisherbrand_pps4102_balance_msgs.msg import BalanceCommand, BalanceReading
from archemist.processing.handler import StationHandler

class FisherWeightingStation_Handler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self.pubFisherScale = rospy.Publisher("/Balance_Commands", BalanceCommand, queue_size=2)
        rospy.sleep(1)

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
        rospy.loginfo('reading stable weight')
        self.pubFisherScale.publish(balance_command=BalanceCommand.WEIGHT_STABLE)
        balance_reading = rospy.wait_for_message('/Balance_Weights',BalanceReading)
        
        current_op.output.has_result = True
        current_op.output.success = True
        current_op.output.weight = balance_reading.weight
        current_op.output.add_timestamp()

        return current_op

# if __name__ == '__main__':
#     fisher_handler = FisherWeightingStation_Handler()
