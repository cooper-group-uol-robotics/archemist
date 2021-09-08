import rospy
from archemist.msg import HandlerBusMessage
from src.archemist.util.rosMsgCoder import rosMsgCoder
from src.archemist.state.robots import pandaFranka
from src.archemist.persistence.dbHandler import dbHandler

class PandaHandler:
    def __init__():
        global pub
        self.__pandaState = pandaFranka()
        global dbhandler
        dbhandler = dbHandler.dbHandler()
        pub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        pubIka = rospy.Publisher("/IKA_Commands", IKACommand, queue_size=1)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, handler_cb)

    def handler_cb(self, msg):
        if(msg.station_name == _pandaState.name and msg.station_id == _pandaState.id):
            rospy.loginfo("Receiving Handler")
            descriptor = vars(rosMsgCoder.decode(msg.opDescriptor))
            _pandaState.readDescriptor(descriptor)
            dbhandler.updateRobotState("pandaFranka", descriptor)
            
