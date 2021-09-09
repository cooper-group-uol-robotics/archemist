from archemist.state.material import Liquid
import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.stations.peristaltic_liquid_dispensing import PeristalticPumpOpDescriptor

def main():
    rospy.init_node("peristalticHandlerTester")
    pub = rospy.Publisher("/processing/HandlerBus", HandlerBusMessage, queue_size=1)
    water = Liquid('water','p1',None,2,997,2)
    descriptor = PeristalticPumpOpDescriptor(water)
    coder = rosMsgCoder()
    pickDesc = coder.code(dict=descriptor)
    message = HandlerBusMessage(station_name="PeristalticLiquidDispensing", station_id=123, opDescriptor=pickDesc)
    rospy.sleep(2)
    pub.publish(message)

if __name__ == '__main__':
    main()
