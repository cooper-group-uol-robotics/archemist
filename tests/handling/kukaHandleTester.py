import rospy
from archemist_msgs.msg import HandlerBusMessage
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.robots.kukaLBRIIWA import KukaMoveBaseOpDescriptor, KukaCalibrateArmOpDescriptor, KukaMoveArmOpDescriptor
from archemist.state.station import Location

def main():
    rospy.init_node("KukaHandlerTester")
    pub = rospy.Publisher("/processing/HandlerBus", HandlerBusMessage, queue_size=1)
    # loc1 = Location('cross_road', 8, 7, 12, '')
    # baseMovedescriptor = KukaMoveBaseOpDescriptor(robot_id=1, target_loc=loc1, fine_localization=True)
    # coder = rosMsgCoder()
    # codedMoveBase = coder.code(dict=baseMovedescriptor)
    # message = HandlerBusMessage(station_name="KukaLBRIIWA", station_id=123, opDescriptor=codedMoveBase)
    # locPanda = Location('panda', 8, 7, 12, '')
    # armCalibratedescriptor = KukaCalibrateArmOpDescriptor(location=locPanda.name)
    # coder = rosMsgCoder()
    # calib = coder.code(dict=armCalibratedescriptor)
    # message = HandlerBusMessage(station_name="KukaLBRIIWA", station_id=123, opDescriptor=calib)
    # rospy.sleep(2)
    # pub.publish(message)
    moveArmDescriptor = KukaMoveArmOpDescriptor(start_pos='robot_rack1', end_pos='panda_vial')
    coder = rosMsgCoder()
    codedMsg = coder.code(dict=moveArmDescriptor)
    message = HandlerBusMessage(station_name="KukaLBRIIWA", station_id=123, opDescriptor=codedMsg)
    rospy.sleep(2)
    pub.publish(message)

if __name__ == '__main__':
    main()