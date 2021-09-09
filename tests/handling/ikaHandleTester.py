import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations.ika_place_rct_digital import IKAHeatingOpDescriptor, IKAStriingOpDescriptor

def main():
    rospy.init_node("ikaHandlerTester")
    pub = rospy.Publisher("/processing/HandlerBus", HandlerBusMessage, queue_size=1)
    descriptor = IKAStriingOpDescriptor(500, 5)
    coder = rosMsgCoder()
    pickDesc = coder.code(dict=descriptor)
    message = HandlerBusMessage(station_name="IkaPlateRCTDigital", station_id=123, opDescriptor=pickDesc)
    rospy.sleep(2)
    pub.publish(message)

if __name__ == '__main__':
    main()
