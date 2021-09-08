import rospy
from archemist_msgs.msg import HandlerBusMessage
from src.archemist.util.rosMsgCoder import rosMsgCoder
from src.archemist.state.stations.ika_place_rct_digital import IKAHeatingOpDescriptor

def main():
    rospy.init_node("stationHandlerTester")
    pub = rospy.Publisher("/processing/HandlerBus", HandlerBusMessage, queue_size=1)
    descriptor = IKAHeatingOpDescriptor(30, 5)
    coder = rosMsgCoder()
    pickDesc = coder.code(dict=descriptor)
    message = HandlerBusMessage(station_name="IkaPlateRCTDigital", station_id=123, opDescriptor=pickDesc)
    pub.publish(message)

if __name__ == '__main__':
    main()
