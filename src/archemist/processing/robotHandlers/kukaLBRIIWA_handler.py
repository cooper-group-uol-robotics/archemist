import rospy
from kmriiwa_chemist_msgs.msg import TaskStatus, LBRCommand, NavCommand, KMRStatus, LBRStatus
from archemist.state.robots.kukaLBRIIWA import KukaLBRTask, KukaNAVTask, KukaLBRMaintenanceTask
from archemist.state.robot import Robot
from archemist.util.location import Location
from archemist.processing.handler import RobotHandler

class KukaLBRIIWA_Handler(RobotHandler):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        rospy.init_node(f'{self._robot}_handler')
        self._kmrCmdPub = rospy.Publisher('/kuka2/kmr/nav_commands', NavCommand, queue_size=1)
        self._lbrCmdPub = rospy.Publisher('/kuka2/lbr/command', LBRCommand, queue_size=1)
        rospy.Subscriber('/kuka2/kmr/task_status', TaskStatus, self._kmr_task_cb, queue_size=1)
        rospy.Subscriber('/kuka2/lbr/task_status', TaskStatus, self._lbr_task_cb, queue_size=1)
        
        # TODO add callbacks to check on the robot status so that charging or other operations can 
        # performed to run the process smoothly.
        rospy.Subscriber('/kuka2/lbr/robot_status', LBRStatus, self._update_lbr_status_cb, queue_size=2)
        #rospy.Subscriber('/kuka2/kmr/robot_status', KMRStatus, self._update_lbr_kmr_status_cb, queue_size=2)
        
        #self._kmr_current_status = ''
        self._kmr_cmd_seq = 0
        self._kmr_task = None
        self._kmr_task_name = ''
        self._kmr_done = False
        
        self._lbr_current_op_state = ''
        self._lbr_cmd_seq = 0
        self._lbr_task = None
        self._lbr_task_name = ''
        self._lbr_done = False
        rospy.sleep(3)

    def run(self):
        try:
            #update local counter since robot cmd counter is ahead while we restarted (self._lbr_cmd_seq = 0)
            if self._lbr_cmd_seq == 0:
                latest_task_msg = rospy.wait_for_message('/kuka2/lbr/task_status', TaskStatus,timeout=5)
                if latest_task_msg.cmd_seq > self._lbr_cmd_seq:
                    self._lbr_cmd_seq = latest_task_msg.cmd_seq
            rospy.loginfo(f'{self._robot}_handler is running')
            while (not rospy.is_shutdown()):
                self.handle()
                rospy.sleep(3)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._robot}_handler is terminating!!!')

    def _kmr_task_cb(self, msg):
        #TODO if published task sequence != 0, while local task counter == 0 it means we restarted and thus set local task counter = published task counter 
        if msg.task_name != '' and msg.task_name == self._kmr_task_name and msg.task_state == TaskStatus.FINISHED:
            self._kmr_task_name = ''
            self._kmr_done = True

    def _wait_for_kmr(self):
        while(not self._kmr_done):
            rospy.sleep(0.2)
        self._kmr_done = False

    def _update_lbr_status_cb(self, msg):
        if msg.robot_op_state != self._lbr_current_op_state:
            self._lbr_current_op_state = msg.robot_op_state
            if self._lbr_current_op_state == 'IDLE':
                self._robot.operational = True
            elif self._lbr_current_op_state != 'BUSY':
                self._robot.operational = False

    def _lbr_task_cb(self, msg):
        if msg.task_name != '' and msg.task_name == self._lbr_task_name and msg.task_state == TaskStatus.FINISHED:
            self._lbr_task_name = ''
            self._lbr_done = True

    def _wait_for_lbr(self):
        while(not self._lbr_done):
            rospy.sleep(0.1)
        self._lbr_done = False

    def _process_kmriiwa_task_op(self, robotOp):
        lbr_task = None
        kmr_task = None
        if isinstance(robotOp, KukaLBRMaintenanceTask):
            self._lbr_cmd_seq += 1
            lbr_task = LBRCommand(cmd_seq=self._lbr_cmd_seq, priority_task=True, task_name=robotOp.job_name, task_parameters=[str(param) for param in robotOp.job_params])
        elif isinstance(robotOp, KukaLBRTask):
            if robotOp.job_location.get_map_coordinates() != self._robot.location.get_map_coordinates():
                self._kmr_cmd_seq += 1
                kmr_task = NavCommand(cmd_seq=self._kmr_cmd_seq, priority_task=False,robot_id=self._robot.id,graph_id=robotOp.job_location.graph_id,
                                        node_id=robotOp.job_location.node_id,
                                        fine_localization=True)
                robotOp.job_params[0] = True # Six point calibration is needed
            else:
                robotOp.job_params[0] = False
            self._lbr_cmd_seq += 1
            lbr_task = LBRCommand(cmd_seq=self._lbr_cmd_seq, priority_task=False,task_name=robotOp.job_name, task_parameters=[str(param) for param in robotOp.job_params])
        elif isinstance(robotOp, KukaNAVTask):
            self._kmr_cmd_seq += 1
            kmr_task = NavCommand(cmd_seq=self._kmr_cmd_seq, priority_task=False,robot_id=self._robot.id,graph_id=robotOp.target_location.graph_id,
                                        node_id=robotOp.target_location.node_id,
                                        fine_localization=robotOp.fine_localisation)
        else:
            rospy.logerr('unknown KUKAIIWA robot op')
        return kmr_task, lbr_task

    def _handle_robot_status(self):
        pass

    def execute_job(self):
        self._handled_robot_op = self._robot.assigned_job
        self._handled_robot_op.robot_op.add_timestamp()
        
        self._kmr_task, self._lbr_task = self._process_kmriiwa_task_op(self._handled_robot_op.robot_op)
        if self._kmr_task is not None:
            if self._kmr_task.fine_localization:
                self._kmr_task_name = f'fine_nav to n:{self._kmr_task.node_id} g:{self._kmr_task.graph_id}'
            else:
                self._kmr_task_name = f'nav to n:{self._kmr_task.node_id} g:{self._kmr_task.graph_id}'
            rospy.loginfo('executing ' + self._kmr_task_name)
            for i in range(10):
                self._kmrCmdPub.publish(self._kmr_task)        
        elif self._lbr_task is not None:
            self._lbr_task_name = self._lbr_task.task_name
            rospy.loginfo('executing ' + self._lbr_task_name)
            for i in range(10):
                self._lbrCmdPub.publish(self._lbr_task)
        
        self._robot.start_job_execution()

    def is_job_execution_complete(self):
        if self._kmr_task is not None and self._kmr_done:
            rospy.loginfo('KMR task complete')
            self._robot.location = Location(node_id=self._kmr_task.node_id, graph_id=self._kmr_task.graph_id,frame_name='')
            self._kmr_task = None
            self._kmr_done = False
            if self._lbr_task is not None:
                self._lbr_task_name = self._lbr_task.task_name
                rospy.loginfo('executing ' + self._lbr_task.task_name)
                for i in range(10):
                    self._lbrCmdPub.publish(self._lbr_task)
                return False
            else:
                return True
        
        if self._lbr_task is not None and self._lbr_done:
            if (self._lbr_task.task_name == 'ChargeRobot'):
                self._robot.location = Location(node_id=3, graph_id=1,frame_name='') #TODO need to set robot location automatically
            self._lbr_task = None
            self._lbr_done = False
            self._handled_robot_op.robot_op.output.has_result = True
            self._handled_robot_op.robot_op.output.success = True
            self._handled_robot_op.robot_op.output.add_timestamp()
            self._handled_robot_op.robot_op.output.executing_robot = str(self._robot)
            return True
        else:
            return False
