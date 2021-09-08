import rospy
from archemist.msg import HandlerBusMessage
from src.archemist.util.rosMsgCoder import rosMsgCoder
from src.archemist.state.stations import ika_plate_rct_digital
from ika_rct_digital.msg import IKACommand

def main():
    rospy.init_node("stationHandlerTester")
    pub = rospy.Publisher("/IKA_Commands", IKACommand, )
    pass

if __name__ == '__main__':
    main()
