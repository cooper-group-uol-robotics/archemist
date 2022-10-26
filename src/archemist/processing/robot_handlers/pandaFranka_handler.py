from archemist.state.robot import Robot, RobotTaskOpDescriptor
import rospy
from franka_msgs_archemist.msg import PandaTask, TaskStatus
from archemist.processing.handler import RobotHandler

class PandaFranka_Handler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        rospy.init_node( f'{self._robot}_handler')
        #TODO robot topic can be set from the config file or even be associated with the robot id
        self._panda_pub = rospy.Publisher('/panda1/task', PandaTask, queue_size=1)
        rospy.Subscriber('/panda1/task_status', TaskStatus, self._panda_task_cb, queue_size=2)
        self._panda_task = None
        self._task_complete = False
        self._op_result = False
        self._task_counter = 0

    def run(self):
        try:
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
        if isinstance(robotOp, RobotTaskOpDescriptor):
            self._task_counter += 1
            return PandaTask(task_name=f'{robotOp.name}', task_seq=self._task_counter,
                             task_parameters=robotOp.params)
        else:
            rospy.logerr('unknown robot op')    
        return None

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
