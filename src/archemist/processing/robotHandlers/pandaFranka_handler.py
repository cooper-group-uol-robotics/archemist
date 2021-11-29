import rospy
from archemist_msgs.msg import HandlerBusMessage, PandaCommand
from kmriiwa_chemist_msgs.msg import TaskStatus
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.robots import PandaFranka
from archemist.state.state import State
from archemist.util.location import Location
from archemist.processing.handler import RobotHandler
from rospy.core import is_shutdown

class PandaHandler(RobotHandler):
    def __init__(self, robot_id: int):
        super.__init__('PandaFranka', robot_id)
        rospy.init_node(self._robot_name + f"_{robot_id}_handler")
        self._pandaCmdPub = rospy.Publisher('/panda1/commands', PandaCommand, queue_size=1)
        rospy.Subscriber('/panda1/task_status', TaskStatus, self.panda_task_cb, queue_size=2)
        self._panda_task = ''
        self._panda_done = False

        rospy.loginfo(self._robot_name + f" with id:{robot_id} handler is running")
        while (not rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)

    def panda_task_cb(self, msg):
        if msg.task_name == self._panda_task and msg.task_state == TaskStatus.FINISHED:
            self._panda_task = ''
            self._panda_done = True

    def wait_for_panda(self):
        while(not self._panda_done):
            rospy.sleep(0.1)
        self._panda_done = False

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

    def execute_job(self):
        (assigned_job, station) = self._robot.assigned_job
        assigned_job.addTimeStamp()
        pandaJob = self.process_op(assigned_job) # this has to be changed to convert vial job to a good message to panda
        self._panda_task = str(pandaJob.panda_command)
        rospy.loginfo('executing ' + self._panda_task)
        self._pandaCmdPub.publish(pandaJob)
        self.wait_for_panda()

        assigned_job.output.has_result = True
        assigned_job.output.success = True
        assigned_job.output.addTimeStamp()

        #self._pandaState._processed_batch.getCurrentSample().location = Location(8,7,opDescriptor.end_pos.frame_name)

if __name__ == '__main__':
    panad_handler = PandaHandler(1)