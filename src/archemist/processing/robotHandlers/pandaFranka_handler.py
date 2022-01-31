from archemist.state.robot import Robot
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
    def __init__(self, robot: Robot):
        super().__init__(robot)
        rospy.init_node( f'{self._robot}_handler')
        self._pandaCmdPub = rospy.Publisher('/panda1/commands', PandaCommand, queue_size=1)
        rospy.Subscriber('/panda1/task_status', TaskStatus, self.panda_task_cb, queue_size=2)
        self._panda_task = ''
        self._panda_done = False

        rospy.loginfo(f'{self._robot}_handler is running')
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
        station_robot_job = self._robot.assigned_job
        station_robot_job.robot_op.add_timestamp()
        # this has to be changed to convert vial job to a good message to panda
        pandaJob = self.process_op(station_robot_job.robot_op)
        self._panda_task = str(pandaJob.panda_command)
        rospy.loginfo('executing ' + self._panda_task)
        self._pandaCmdPub.publish(pandaJob)
        self.wait_for_panda()

        station_robot_job.robot_op.output.has_result = True
        station_robot_job.robot_op.output.success = True
        station_robot_job.robot_op.output.add_timestamp()

        return station_robot_job



if __name__ == '__main__':
    panad_handler = PandaHandler(1)