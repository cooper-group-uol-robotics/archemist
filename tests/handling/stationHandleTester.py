import rospy
from archemist.msg import HandlerBusMessage
from src.archemist.util.rosMsgCoder import rosMsgCoder
from src.archemist.state.stations import ika_plate_rct_digital
def main():
    rospy.init_node("stationHandlerTester")
    pub = rospy.Publisher()
    pass

if __name__ == '__main__':
    main()
