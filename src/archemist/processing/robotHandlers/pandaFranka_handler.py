import rospy
from archemist.msg import HandlerBusMessage
from kmriiwa_chemist_msgs.msg import TaskStatus
from src.archemist.util.rosMsgCoder import rosMsgCoder
from src.archemist.state.robots import pandaFranka
from src.archemist.persistence.dbHandler import dbHandler
from archemist_msgs.msg import PandaCommand

class PandaHandler:
    def __init__():
        global panda_task, panda_done
        global pub
        self.__pandaState = pandaFranka()
        global dbhandler
        dbhandler = dbHandler.dbHandler()
        pub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        pandaCmdPub = rospy.Publisher('/panda1/commands', PandaCommand, queue_size=1)
        rospy.Subscriber('/panda1/task_status', TaskStatus, panda_task_cb, queue_size=2)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, handler_cb)
        panda_task = ''
        panda_done = False


    def handler_cb(self, msg):
        if(msg.station_name == _pandaState.name and msg.station_id == _pandaState.id):
            rospy.loginfo("Receiving Handler")
            descriptor = vars(rosMsgCoder.decode(msg.opDescriptor))
            _pandaState.readDescriptor(descriptor)
            dbhandler.updateRobotState("pandaFranka", descriptor)

            pandaJob = PandaCommand(panda_command=descriptor["pandaJob"])
            panda_task = str(pandaJob.panda_command)
            rospy.loginfo('exec ' + panda_task)
            pandaCmdPub.publish(pandaJob)
            wait_for_panda()

    def panda_task_cb(self, msg):
        global panda_task, panda_done
        if msg.task_name == panda_task and msg.task_state == TaskStatus.FINISHED:
            panda_task = ''
            panda_done = True

    def wait_for_panda(self):
        global panda_done
        while(not panda_done):
            rospy.sleep(0.1)
        panda_done = False
