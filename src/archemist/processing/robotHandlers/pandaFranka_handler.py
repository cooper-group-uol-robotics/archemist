import rospy
from archemist_msgs.msg import HandlerBusMessage, PandaCommand
from kmriiwa_chemist_msgs.msg import TaskStatus
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.robots import PandaFranka
from archemist.state.state import State
from archemist.util.location import Location
from rospy.core import is_shutdown

class PandaHandler:
    def __init__(self):
        rospy.init_node("panda_handler")
        print("panda_handler running")
        self.state = State()
        self.state.initializeState(False)
        self._pandaState = self.state.getRobot('PandaFranka',1)
        self.coder = rosMsgCoder()
        #self._dbhandler = dbHandler.dbHandler()
        self._ackpub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self._pandaCmdPub = rospy.Publisher('/panda1/commands', PandaCommand, queue_size=1)
        rospy.Subscriber('/panda1/task_status', TaskStatus, self.panda_task_cb, queue_size=2)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        self.panda_task = ''
        self.panda_done = False
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)


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
        if msg.task_name == self.panda_task and msg.task_state == TaskStatus.FINISHED:
            self.panda_task = ''
            self.panda_done = True

    def wait_for_panda(self):
        while(not self.panda_done):
            rospy.sleep(0.1)
        self.panda_done = False

    def process_op(self, robotOp):
        if robotOp.start_pos.frame_name == 'handover_cube' and robotOp.end_pos.frame_name == 'pump':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALINITIALPOSITIONTOPUMP)
        elif robotOp.start_pos.frame_name == 'pump' and robotOp.end_pos.frame_name == 'ika':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALPUMPTOSTIRRER)
        elif robotOp.start_pos.frame_name == 'ika' and robotOp.end_pos.frame_name == 'handover_cube':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALSTIRRERTOINITIALPOSITION)
        elif robotOp.start_pos.frame_name == 'ika' and robotOp.end_pos.frame_name == 'scale':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALHEATERTOSCALE)
        elif robotOp.start_pos.frame_name == 'scale' and robotOp.end_pos.frame_name == 'ika':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALSCALETOHEATER)
        elif robotOp.start_pos.frame_name == 'scale' and robotOp.end_pos.frame_name == 'handover_cube':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALSCALETOINITIALPOSITION)
        elif robotOp.start_pos.frame_name == 'pump' and robotOp.end_pos.frame_name == 'scale':
            return PandaCommand(panda_command=PandaCommand.MOVEVIALPUMPTOSCALE)
        else:
            rospy.logerr('unknown panda op')    
        return None

    def handle(self):
        self.state.updateFromDB()
        self._pandaState = self.state.getRobot('PandaFranka',1)
        
        if self._pandaState._assigned_batch is not None:
            opDescriptor = self._pandaState._assigned_batch.getCurrentOp()
            pandaJob = self.process_op(opDescriptor)
            self.panda_task = str(pandaJob.panda_command)
            rospy.loginfo('exec ' + self.panda_task)
            self._pandaCmdPub.publish(pandaJob)
            self.wait_for_panda()

            self._pandaState._processed_batch = self._pandaState._assigned_batch
            self._pandaState._processed_batch.getCurrentSample().location = Location(8,7,opDescriptor.end_pos.frame_name)
            self._pandaState._processed_batch.advanceProcessState()
            self._pandaState._assigned_batch = None
            self.state.modifyObjectDB(self._pandaState)

if __name__ == '__main__':
    panad_handler = PandaHandler()