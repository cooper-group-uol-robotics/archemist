import rospy
from archemist_msgs.msg import HandlerBusMessage, PandaCommand
from kmriiwa_chemist_msgs.msg import TaskStatus
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.robots import PandaFranka
from archemist.persistence.dbHandler import dbHandler

class PandaHandler:
    def __init__(self):
        rospy.init_node("panda_handler")
        print("panda_handler running")
        self._pandaState = PandaFranka(123)
        self.coder = rosMsgCoder()
        #self._dbhandler = dbHandler.dbHandler()
        self._ackpub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self._pandaCmdPub = rospy.Publisher('/panda1/commands', PandaCommand, queue_size=1)
        rospy.Subscriber('/panda1/task_status', TaskStatus, self.panda_task_cb, queue_size=2)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        self.panda_task = ''
        self.panda_done = False


    def handler_cb(self, msg):
        if(msg.station_name == self._pandaState.__class__.__name__ and msg.station_id == self._pandaState.id):
            rospy.loginfo("Receiving Handler")
            opDescriptor = self.coder.decode(msg.opDescriptor)
            #self._pandaState.setRobotOp(opDescriptor)
            #self.dbhandler.updateRobotState("pandaFranka", vars(opDescriptor))

            pandaJob = self.process_op(opDescriptor)
            panda_task = str(pandaJob.panda_command)
            rospy.loginfo('exec ' + panda_task)
            self._pandaCmdPub.publish(pandaJob)
            self.wait_for_panda()

    def panda_task_cb(self, msg):
        global panda_task, panda_done
        if msg.task_name == self.panda_task and msg.task_state == TaskStatus.FINISHED:
            self.panda_task = ''
            self.panda_done = True

    def wait_for_panda(self):
        self.panda_done
        while(not self.panda_done):
            rospy.sleep(0.1)
        self.panda_done = False

    def process_op(self, robotOp):
        if robotOp.start_pos == 'initial' and robotOp.end_pos == 'pump':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALINITIALPOSITIONTOPUMP)
        elif robotOp.start_pos == 'pump' and robotOp.end_pos == 'ika':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALPUMPTOSTIRRER)
        elif robotOp.start_pos == 'ika' and robotOp.end_pos == 'initial':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALSTIRRERTOINITIALPOSITION)
        else:
            rospy.logerr('unknown panda op')    
        return None