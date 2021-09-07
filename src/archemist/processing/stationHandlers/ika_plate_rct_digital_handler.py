import rospy
from archemist.msgs import HandlerBusMessage
class ikaHandler:
    def __init__():
        global pub
        pub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, handler_cb)

    def handler_cb():
        pass
