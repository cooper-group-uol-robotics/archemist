from .state import PandaRobotTask
from archemist.core.state.robot import Robot
import rospy
from franka_msgs_archemist.msg import PandaTask, TaskStatus
from archemist.core.processing.handler import RobotHandler


class PandaROSHandler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        rospy.init_node(f'{self._robot}_handler')
        # TODO robot topic can be set from the config file or even be associated with the robot id
        self._panda_pub = rospy.Publisher(
            '/panda1/task', PandaTask, queue_size=1)
        rospy.Subscriber('/panda1/task_status', TaskStatus,
                         self._panda_task_cb, queue_size=2)
        self._panda_task = None
        # self._panda_cmd_seq = 0
        self._task_complete = False
        self._op_result = False
        self._task_counter = 0

    def run(self):
        try:
            # update local counter since robot cmd counter is ahead while we restarted (self._panda_cmd_seq = 0)
            if self._task_counter == 0:
                latest_task_msg = rospy.wait_for_message(
                    '/panda1/task_status', TaskStatus, timeout=5)
                if latest_task_msg.cmd_seq > self._task_counter:
                    self._task_counter = latest_task_msg.cmd_seq
            rospy.loginfo(f'{self._robot}_handler is running')
            while (not rospy.is_shutdown()):
                self.handle()
                rospy.sleep(3)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._robot}_handler is terminating!!!')

    def _panda_task_cb(self, msg: TaskStatus):
        if not self._task_complete and msg.task_name == self._panda_task.task_name and msg.task_seq == self._panda_task.task_seq:
            if msg.task_state == TaskStatus.FINISHED:
                self._task_complete = True
                self._op_result = True
            elif msg.task_state == TaskStatus.ERROR:
                self._task_complete = True
                self._op_result = False

    def _process_op(self, robotOp):
        task = None
        if isinstance(robotOp, PandaRobotTask):
            task = PandaTask(
                task_name=f'{robotOp.name}', cmd_seq=self._task_counter)
            self._task_counter += 1
        else:
            rospy.logerr('unknown robot op')
        return task

    def execute_op(self):
        robot_op = self._robot.get_assigned_op()
        # this has to be changed to convert vial job to a good message to panda
        self._panda_task = self._process_op(robot_op)
        rospy.loginfo('executing ' + self._panda_task.task_name)
        self._task_complete = False
        for i in range(10):
            self._panda_pub.publish(self._panda_task)

    def is_op_execution_complete(self) -> bool:
        return self._task_complete

    def get_op_result(self) -> bool:
        return self._op_result
