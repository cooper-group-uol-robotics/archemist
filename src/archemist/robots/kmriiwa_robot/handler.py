import rospy
from kmriiwa_chemist_msgs.msg import TaskStatus, LBRCommand, KMPCommand, RobotCommand, RobotStatus
from archemist.core.state.robot import Robot, MobileRobot, MobileRobotMode, RobotState
from archemist.core.state.robot_op import RobotTaskOp, RobotNavOp, RobotOp
from archemist.core.util.location import Location
from archemist.core.processing.handler import RobotHandler, RobotOpHandler
from .state import KMRIIWARobot

class KmriiwaROSHandler(RobotOpHandler):
    def __init__(self, robot: KMRIIWARobot):
        super().__init__(robot)

    def initialise(self) -> bool:
        rospy.init_node(f'{self._robot}_handler')
        self._kukaCmdPub = rospy.Publisher('/kuka4/commands', RobotCommand, queue_size=1)
        # rospy.Subscriber('/kuka4/robot_status', RobotStatus, self._update_kmp_status_cb, queue_size=1)
        rospy.Subscriber('/kuka4/task_status', TaskStatus, self._kuka_task_cb, queue_size=1)
        
        self._kuka_cmd_seq = 17
        self._assigned_task_type = None
        self._kuka_task_name = ''
        self._kmr_done = False
        self._kmr_exec_successful = False 
        self._lbr_current_op_state = ''
        self._need_to_charge = False
        self._need_to_calibrate = False
        rospy.sleep(2)
        return True

    def run(self):
        try:
            #update local counter since robot cmd counter is ahead while we restarted (self._lbr_cmd_seq = 0)
            if self._kuka_cmd_seq == 0:
                latest_task_msg = rospy.wait_for_message('/kuka4/task_status', TaskStatus,timeout=5)
                if latest_task_msg.cmd_seq > self._kuka_cmd_seq:
                    self._kuka_cmd_seq = latest_task_msg.cmd_seq
            rospy.loginfo(f'{self._robot}_handler is running')
            while (not rospy.is_shutdown()):
                self.handle()
                if self._robot.state == RobotState.IDLE:
                    self._handle_robot_status()
                rospy.sleep(3)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._robot}_handler is terminating!!!')

    def _update_kmp_status_cb(self, msg:RobotStatus):
        if self._robot.location.coordinates[0] != msg.last_graph_node_id: 
            location_dict = {"coordinates": [msg.last_graph_node_id, 8],"descriptor": "Example Location"}
            loc = Location.from_dict(location_dict)
            self._robot.location = Location(loc) #TODO pull graph id from config file
        self._lbr_current_op_state = msg.robot_op_state
    
    def _kuka_task_cb(self, msg:TaskStatus):
        if msg.cmd_seq == self._kuka_cmd_seq:
            if self._assigned_task_type == RobotCommand.LBR_TASK:
                if msg.task_lbr_state == TaskStatus.FINISHED:
                    self._kmr_exec_successful = True
                    self._kmr_done = True
                    self._kuka_cmd_seq += 1
                elif msg.task_lbr_state == TaskStatus.ERROR:
                    self._kmr_exec_successful = True
                    self._kmr_done = False
                    self._kuka_cmd_seq += 1
            elif self._assigned_task_type ==RobotCommand.KMP_TASK:
                if msg.task_kmp_state == TaskStatus.FINISHED:
                    self._kmr_exec_successful = True
                    self._kmr_done = True
                    self._kuka_cmd_seq += 1
                elif msg.task_kmp_state == TaskStatus.ERROR:
                    self._kmr_exec_successful = True
                    self._kmr_done = False
                    self._kuka_cmd_seq += 1
            elif self._assigned_task_type ==RobotCommand.COMBINED_TASK:
                if msg.task_kmp_state == TaskStatus.FINISHED and msg.task_lbr_state == TaskStatus.FINISHED:
                    self._kmr_exec_successful = True
                    self._kmr_done = True
                    self._kuka_cmd_seq += 1
                elif msg.task_kmp_state == TaskStatus.ERROR or msg.task_lbr_state == TaskStatus.ERROR:
                    self._kmr_exec_successful = True
                    self._kmr_done = False 
                    self._kuka_cmd_seq += 1   

    def _handle_robot_status(self):
        if self._robot.operational_mode ==  MobileRobotMode.OPERATIONAL:
            if self._need_to_charge or self._need_to_calibrate:
                if len(self._robot.onboard_batches) == 0 or self._robot.is_onboard_capacity_full():
                    self._robot.operational_mode =  MobileRobotMode.COOLDOWN
        if self._robot.operational_mode ==  MobileRobotMode.COOLDOWN:
            if len(self._robot.onboard_batches) == 0:
                self._robot.operational_mode =  MobileRobotMode.MAINTENANCE
                # enable auto functions
                self._kuka_cmd_seq += 1
                maintanence_task = RobotCommand(cmd_seq=self._kuka_cmd_seq, priority_task= True, task_name="EnableAutoFunctions", task_type=2,program_name = "EnableAutoFunctions", program_parameters = ["False"],  node_id = self._robot.location.coordinates[0], graph_id = self._robot.location.coordinates[1], fine_localization = False)
                for _ in range(10):
                    self._kukaCmdPub.publish(maintanence_task)
        if self._robot.operational_mode ==  MobileRobotMode.MAINTENANCE:
            if not self._need_to_charge and self._lbr_current_op_state == "IDLE" and not self._need_to_calibrate:
                # disable auto functions
                self._kuka_cmd_seq += 1
                maintanence_task = RobotCommand(cmd_seq=self._kuka_cmd_seq, priority_task= True, task_name="DisableAutoFunctions", task_type=2,program_name = "EnableAutoFunctions", program_parameters = ["False"], node_id = self._robot.location.coordinates[0], graph_id = self._robot.location.coordinates[1], fine_localization = False)
                for _ in range(10):
                    self._kukaCmdPub.publish(maintanence_task)
                self._robot.operational_mode =  MobileRobotMode.OPERATIONAL

    def execute_op(self):
        robot_op:RobotOp = self._robot.assigned_op
        self._kmr_exec_successful = False
        if robot_op is not None:
            self._kmr_exec_successful = False
            self._kmr_done = False
            if isinstance(robot_op, RobotNavOp):
                self._assigned_task_type = RobotCommand.KMP_TASK

                kmp_task = RobotCommand()
                kmp_task.seq = self._kuka_cmd_seq
                kmp_task.priority_task = False
                kmp_task.task_type = 1
                kmp_task.task_name = robot_op.name

                _base_command = KMPCommand()
                _base_command.node_id = robot_op.target_location.coordinates[0]
                _base_command.graph_id = robot_op.target_location.coordinates[1] 
                _base_command.fine_localization = True
                kmp_task.base_command = _base_command
                
                rospy.loginfo(f'executing task {robot_op.name} with cmd_seq: {self._kuka_cmd_seq}')
                for i in range(10):
                    self._kukaCmdPub.publish(kmp_task)
                
            elif isinstance(robot_op, RobotTaskOp):
                if robot_op.task_type == 2:
                    self._assigned_task_type = RobotCommand.COMBINED_TASK
                elif robot_op.task_type == 1:
                    self._assigned_task_type = RobotCommand.KMP_TASK

                combined_task = RobotCommand()
                combined_task.seq = self._kuka_cmd_seq
                combined_task.priority_task = False
                combined_task.task_type = robot_op.task_type
                combined_task.task_name = robot_op.name

                _base_command = KMPCommand()
                print(f"++++++++++++++++++++++++{robot_op.target_location.coordinates[0]}--------------")
                print(f"////////////////////////{robot_op.target_location.coordinates[1]}--------------")
                _base_command.node_id = robot_op.target_location.coordinates[0]
                _base_command.graph_id = robot_op.target_location.coordinates[1] 
                _base_command.fine_localization = robot_op.fine_localization

                _arm_command = LBRCommand()
                _arm_command.program_name = robot_op.lbr_program_name
                _arm_command.program_parameters = robot_op.lbr_program_params

                combined_task.base_command = _base_command
                combined_task.arm_command = _arm_command

                rospy.loginfo(f'executing task {robot_op.name} with cmd_seq: {self._kuka_cmd_seq}')
                for i in range(10):
                    self._kukaCmdPub.publish(combined_task)       
         

    def is_op_execution_complete(self):
        return self._kmr_exec_successful

    def get_op_result(self) -> bool:
        return self._kmr_done
    
    def shut_down(self):
        pass
