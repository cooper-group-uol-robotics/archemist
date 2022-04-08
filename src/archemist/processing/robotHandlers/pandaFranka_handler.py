from archemist.state.robot import Robot, MoveSampleOp
import rospy
from panda_archemist_msgs.msg import PandaTask, PandaTaskStatus
from archemist.processing.handler import RobotHandler

class PandaFranka_Handler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        rospy.init_node( f'{self._robot}_handler')
        self._pandaCmdPub = rospy.Publisher('/panda1/task', PandaTask, queue_size=1)
        rospy.Subscriber('/panda1/task_status', PandaTaskStatus, self._panda_task_cb, queue_size=2)
        self._panda_task = ''
        self._panda_done = False
        self._task_counter = 0

    def run(self):
        try:
            rospy.loginfo(f'{self._robot}_handler is running')
            while (not rospy.is_shutdown()):
                self.handle()
                rospy.sleep(3)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._robot}_handler is terminating!!!')



    def _panda_task_cb(self, msg):
        if msg.task_name != '' and msg.task_name == self._panda_task and msg.task_state == PandaTaskStatus.FINISHED:
            self._panda_task = ''
            self._panda_done = True

    def _wait_for_panda(self):
        while(not self._panda_done):
            rospy.sleep(0.1)
        self._panda_done = False

    def _process_op(self, robotOp):
        if isinstance(robotOp, MoveSampleOp):
            return PandaTask(task_type=PandaTask.MOVE_VIAL_TASK, 
                             task_name=f'MOVE_VIAL_TASK_{self._task_counter}', 
                             task_parameters=[robotOp.start_location.frame_name,robotOp.target_location.frame_name, str(robotOp.sample_index)])
        else:
            rospy.logerr('unknown robot op')    
        return None

    def execute_job(self):
        station_robot_job = self._robot.assigned_job
        station_robot_job.robot_op.add_timestamp()
        # this has to be changed to convert vial job to a good message to panda
        pandaJob = self._process_op(station_robot_job.robot_op)
        self._panda_task = pandaJob.task_name
        rospy.loginfo('executing ' + self._panda_task)
        self._pandaCmdPub.publish(pandaJob)
        self._wait_for_panda()

        station_robot_job.robot_op.output.has_result = True
        station_robot_job.robot_op.output.success = True
        station_robot_job.robot_op.output.add_timestamp()
        station_robot_job.robot_op.output.executing_robot = str(self._robot)
        self._task_counter += 1
        return station_robot_job



# if __name__ == '__main__':
#     panad_handler = PandaHandler(1)