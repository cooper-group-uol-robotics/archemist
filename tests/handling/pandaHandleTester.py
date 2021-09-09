import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.robots.pandaFranka import PandaMoveOpDescriptor

def main():
    rospy.init_node("pandaHandlerTester")
    pub = rospy.Publisher("/processing/HandlerBus", HandlerBusMessage, queue_size=1)
    descriptor = PandaMoveOpDescriptor('pump', 'ika')
    coder = rosMsgCoder()
    pickDesc = coder.code(dict=descriptor)
    message = HandlerBusMessage(station_name="PandaFranka", station_id=123, opDescriptor=pickDesc)
    rospy.sleep(2)
    pub.publish(message)

if __name__ == '__main__':
    main()