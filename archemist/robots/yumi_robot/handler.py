# External
import rospy
from yumi_task_msgs.msg import YuMiTask, TaskStatus

# Core
from archemist.core.processing.handler import RobotHandler
from archemist.core.state.robot import Robot
from .state import YuMiRobotTask


class YuMiROSHandler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        rospy.init_node(f"{self._robot}_handler")
        # TODO robot topic can be set from the config file or even
        # be associated with the robot id
        self._yumi_pub = rospy.Publisher("/yumi/task", YuMiTask, queue_size=1)
        rospy.Subscriber("/yumi/status", TaskStatus, self._yumi_task_cb, queue_size=2)
        self._yumi_task = YuMiTask()
        self._task_complete = False
        self._op_result = False
        self._task_counter = 0

    def run(self):
        try:
            if self._task_counter == 0:
                latest_task_msg = rospy.wait_for_message(
                    "/yumi/status", TaskStatus, timeout=5
                )
                if latest_task_msg.cmd_seq > self._task_counter:
                    self._task_counter = latest_task_msg.cmd_seq
                    rospy.loginfo(
                        "relaunched handler while driver running. Task \
                            message counter updated."
                    )
            rospy.loginfo(f"{self._robot}_handler is running")
            while not rospy.is_shutdown():
                self.handle()
                rospy.sleep(3)
        except KeyboardInterrupt:
            rospy.loginfo(f"{self._robot}_handler is terminating!!!")

    def _yumi_task_cb(self, msg: TaskStatus):
        if (
            not self._task_complete
            and msg.task_name == self._yumi_task.task_name
            and msg.cmd_seq == self._yumi_task.cmd_seq
        ):
            if msg.task_state == TaskStatus.FINISHED:
                self._task_complete = True
                self._op_result = True
            elif msg.task_state == TaskStatus.ERROR:
                self._task_complete = True
                self._op_result = False

    def _process_op(self, robot_op) -> YuMiTask:
        task = None
        if isinstance(robot_op, YuMiRobotTask):
            self._task_counter += 1
            task = YuMiTask(task_name=f"{robot_op.name}", cmd_seq=self._task_counter)
        else:
            rospy.logerr("unknown robot op")
        return task

    def execute_op(self):
        robot_op = self._robot.get_assigned_op()
        self._yumi_task = self._process_op(robot_op)
        rospy.loginfo("executing " + self._yumi_task.task_name)
        self._task_complete = False
        for _ in range(10):
            self._yumi_pub.publish(self._yumi_task)

    def is_op_execution_complete(self) -> bool:
        return self._task_complete

    def get_op_result(self) -> bool:
        return self._op_result
