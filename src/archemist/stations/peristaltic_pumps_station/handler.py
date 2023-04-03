#!/usr/bin/env python3

import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from pi4_peristaltic_pump_msgs.msg import DispenserCommand
from archemist.core.processing.handler import StationHandler
from .state import PeristalticPumpOpDescriptor
from std_msgs.msg import String


class PeristalticLiquidDispensingROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f"{self._station}_handler")
        self._pump_pub = rospy.Publisher(
            "/Dispenser_Commands", DispenserCommand, queue_size=1
        )
        rospy.Publisher("/Dispenser_Done", String, self._pump_callback)
        self._received_results = False
        rospy.sleep(1)

    def run(self):
        rospy.loginfo(f"{self._station}_handler is running")
        try:
            while not rospy.is_shutdown():
                self.handle()
                rospy.sleep(2)
        except KeyboardInterrupt:
            rospy.loginfo(f"{self._station}_handler is terminating!!!")

    def execute_op(self):
        current_op = self._station.get_assigned_station_op()
        self._received_results = False
        if isinstance(current_op, PeristalticPumpOpDescriptor):
            # TODO depending on the liquid we can select the correct pump
            for i in range(10):
                self._pump_pub.publish(
                    dispenser_command=DispenserCommand.DISPENSEPID,
                    dispenser_ml=current_op.dispense_volume,
                )
        else:
            rospy.logwarn(f"[{self.__class__.__name__}] Unkown operation was received")

    def is_op_execution_complete(self) -> bool:
        return self._received_results

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}

    def _pump_callback(self, msg):
        self._received_results = True
