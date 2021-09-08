import rospy
from archemist_msgs.msg import HandlerBusMessage
from src.archemist.util.rosMsgCoder import rosMsgCoder
from src.archemist.state.stations import ika_plate_rct_digital
from src.archemist.persistence.dbHandler import dbHandler
from archemist_msgs.msg import IKACommand
class ikaHandler:
    def __init__():
        global pub
        self._ikaState = ika_plate_rct_digital()
        global dbhandler
        dbhandler = dbHandler.dbHandler()
        pub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        pubIka = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, handler_cb)

    def handler_cb(self, msg):
        if(msg.station_name == _ikaState.name and msg.station_id == _ikaState.id):
            rospy.loginfo("Receiving Handler")
            rospy.loginfo(vars(rosMsgCoder.decode(msg.opDescriptor)))
            descriptor = rosMsgCoder.decode(msg.opDescriptor)
            #_ikaState.readDescriptor(descriptor)
            dbhandler.updateStationState("ika_plate_rct_digital", descriptor)
            if (descriptor[6] == True):
                if (descriptor[5] == 1):
                    pubIka.publish(ika_command= 7, ika_param=descriptor[1])
                elif (descriptor[5] == 2):
                    pubIka.publish(ika_command= 6, ika_param=descriptor[2])
                elif (descriptor[5] == 3):
                    pubIka.publish(ika_command= 7, ika_param=descriptor[1])
                    pubIka.publish(ika_command= 6, ika_param=descriptor[2])
            else:
                pubIka.publish(ika_command= 8)
