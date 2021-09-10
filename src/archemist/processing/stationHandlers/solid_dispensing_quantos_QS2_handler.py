import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations.peristaltic_liquid_dispensing import PeristalticLiquidDispensing, PeristalticPumpOutputDescriptor
from archemist.persistence.dbHandler import dbHandler
from archemist_msgs.msg import DispenserCommand
from archemist.state.material import Liquid
from datetime import datetime
from std_msgs.msg import String

class QuantoDispensingHandler:
    def __init__(self):
        rospy.init_node("QuantosDispensingHandler")
        print("Quantos Handler Running")
        self.coder = rosMsgCoder()
        location
        parameters
        liquids
        solids
        self._peristalticDispenser = PeristalticLiquidDispensing(123, location, parameters, liquids, solids)
        #dbhandler = dbHandler.dbHandler()
        self._ackpub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self._pubPeristaltic = rospy.Publisher("/Dispenser_Commands", DispenserCommand, queue_size=1)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        rospy.spin()

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
