#!/usr/bin/env python3

import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations.peristaltic_liquid_dispensing import PeristalticLiquidDispensing, PeristalticPumpOutputDescriptor
from archemist.persistence.dbHandler import dbHandler
from archemist_msgs.msg import DispenserCommand
from archemist.state.material import Liquid
from archemist.state.state import State
from datetime import datetime
from std_msgs.msg import String

class PeristalticLiquidDispensing_Handler:
    def __init__(self):
        rospy.init_node("PeristalticDispensingHandler")
        print("PeristalticDispensingHandler running")
        self.coder = rosMsgCoder()
        self.state = State()
        self.state.initializeState(False)
        self._peristalticDispenser = self.state.getStation('PeristalticLiquidDispensing')
        self._ackpub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self._pubPeristaltic = rospy.Publisher("/Dispenser_Commands", DispenserCommand, queue_size=1)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)

    def handler_cb(self, msg):
        if(msg.station_name == self._peristalticDispenser.__class__.__name__ and msg.station_id == self._peristalticDispenser.id):
            rospy.loginfo("Receiving Handler")
            opDescriptor = self.coder.decode(msg.opDescriptor)
            self._peristalticDispenser.setStationOp(opDescriptor)
            #dbhandler.updateStationState("ika_plate_rct_digital", descriptor)
            if (opDescriptor.__class__.__name__ == 'PeristalticPumpOpDescriptor'):
                print("dispensing liquid")
                pump_id = self._peristalticDispenser.getPumpID(opDescriptor.liquid)
                if (pump_id == 'p1'):
                    self._pubPeristaltic.publish(dispenser_command=8, dispenser_ml=opDescriptor.liquid.volume)
                    rospy.wait_for_message("/Dispenser_Done", String)
                    print(PeristalticPumpOutputDescriptor(opDescriptor.__class__.__name__, True))
                    # send back to the central
                # elif (pump_id == 'p2'):
                #   publish to another pump or indicate that in your message using pumpid field

    def handle(self):
        self.state.updateFromDB()
        self._peristalticDispenser = self.state.getStation('PeristalticLiquidDispensing')
        if self._peristalticDispenser._assigned_batch is not None:
            print ('inside')
            opDescriptor = self._peristalticDispenser._assigned_batch.getCurrentOp()
            temp_liquid = Liquid(opDescriptor.liquid_name,'',None,0,0,opDescriptor.dispense_volume)
            pump_id = self._peristalticDispenser.getPumpID(temp_liquid)
            if (pump_id == 'p1'):
                self._pubPeristaltic.publish(dispenser_command=8, dispenser_ml=opDescriptor.dispense_volume)
                rospy.wait_for_message("/Dispenser_Done", String)
                
                # print(PeristalticPumpOutputDescriptor(opDescriptor.__class__.__name__, True))
                # result = PeristalticPumpOutputDescriptor(opDescriptor.__class__.__name__)
                # result.has_result = True
                # result.success = True
                # result.addTimeStamp()
                # opDescriptor.output = result
                self._peristalticDispenser._assigned_batch.advanceProcessState()
                self._peristalticDispenser._processed_batch = self._peristalticDispenser._assigned_batch
                self._peristalticDispenser._assigned_batch = None
                self.state.modifyObjectDB(self._peristalticDispenser)

if __name__ == '__main__':
    ika_handler = PeristalticLiquidDispensing_Handler()