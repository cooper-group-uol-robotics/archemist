import rospy
from kmriiwa_chemist_msgs.msg import TaskStatus, LBRCommand, NavCommand
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask
from archemist.state.robot import Robot
from archemist.util.location import Location
from archemist.processing.handler import RobotHandler

class KukaLBRIIWA_Handler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        rospy.init_node(f'{self._robot}_handler')
        self._kmrCmdPub = rospy.Publisher('/kuka2/kmr/nav_commands', NavCommand, queue_size=1)
        self._lbrCmdPub = rospy.Publisher('/kuka2/lbr/command', LBRCommand, queue_size=1)
        rospy.Subscriber('/kuka2/kmr/task_status', TaskStatus, self._kmr_task_cb, queue_size=2)
        rospy.Subscriber('/kuka2/lbr/task_status', TaskStatus, self._lbr_task_cb, queue_size=2)
        
        # TODO add callbacks to check on the robot status so that charging or other operations can 
        # performed to run the process smoothly.
        #rospy.Subscriber('/kuka2/lbr/robot_status', TaskStatus, self.update_lbr_status_cb, queue_size=2)
        #rospy.Subscriber('/kuka2/kmr/robot_status', TaskStatus, self.update_lbr_kmr_status_cb, queue_size=2)
        
        self._kmr_task = ''
        self._kmr_done = False
        self._lbr_task = ''
        self._lbr_done = False
        rospy.sleep(3)

    def run(self):
        try:
            rospy.loginfo(f'{self._robot}_handler is running')
            while (not rospy.is_shutdown()):
                self.handle()
                rospy.sleep(3)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._robot}_handler is terminating!!!')

    def _kmr_task_cb(self, msg):
        if msg.task_name != '' and msg.task_name == self._kmr_task and msg.task_state == TaskStatus.FINISHED:
            self._kmr_task = ''
            self._kmr_done = True

    def _wait_for_kmr(self):
        while(not self._kmr_done):
            rospy.sleep(0.2)
        self._kmr_done = False

    def _lbr_task_cb(self, msg):
        if msg.task_name != '' and msg.task_name == self._lbr_task and msg.task_state == TaskStatus.FINISHED:
            self._lbr_task = ''
            self._lbr_done = True

    def _wait_for_lbr(self):
        while(not self._lbr_done):
            rospy.sleep(0.1)
        self._lbr_done = False

    def _process_lbriiwa_task_op(self, robotOp):
        if isinstance(robotOp, KukaLBRTask):
            nav_task = None
            if robotOp.job_location.get_map_coordinates() != self._robot.location.get_map_coordinates():
                nav_task = NavCommand(robot_id=self._robot.id,graph_id=robotOp.job_location.graph_id,
                                        node_id=robotOp.job_location.node_id,
                                        fine_localization=True)
                robotOp.job_params[0] = True # Six point calibration is needed
            else:
                robotOp.job_params[0] = False
            lbr_task = LBRCommand(task_name=robotOp.job_name, task_parameters=[str(param) for param in robotOp.job_params])
            return lbr_task, nav_task
        else:
            rospy.logerr('unknown KUKAIIWA robot op')
        return None

    def _handle_robot_status(self):
        pass

    def execute_job(self):
        station_robot_job = self._robot.assigned_job
        station_robot_job.robot_op.add_timestamp()
        lbr_job, kmr_job = self._process_lbriiwa_task_op(station_robot_job.robot_op)
        
        if kmr_job is not None:
            if kmr_job.fine_localization:
                self._kmr_task = f'fine_nav to n:{kmr_job.node_id} g:{kmr_job.graph_id}'
            else:
                self._kmr_task = f'nav to n:{kmr_job.node_id} g:{kmr_job.graph_id}'
            rospy.loginfo('executing ' + self._kmr_task)
            self._kmrCmdPub.publish(kmr_job)
            self._wait_for_kmr()
            print('done with navigation')
            self._robot.location = Location(node_id=kmr_job.node_id, graph_id=kmr_job.graph_id,frame_name='')
        
        self._lbr_task = lbr_job.task_name
        rospy.loginfo('executing ' + self._lbr_task)
        self._lbrCmdPub.publish(lbr_job)
        self._wait_for_lbr()
        rospy.loginfo('completed executing ' + self._lbr_task)

        station_robot_job.robot_op.output.has_result = True
        station_robot_job.robot_op.output.success = True
        station_robot_job.robot_op.output.add_timestamp()
        station_robot_job.robot_op.output.executing_robot = str(self._robot)
        return station_robot_job


# if __name__ == '__main__':
#     kuka_handler = KukaHandler()
