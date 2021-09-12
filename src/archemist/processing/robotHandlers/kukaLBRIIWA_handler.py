import rospy
from archemist_msgs.msg import HandlerBusMessage
from kmriiwa_chemist_msgs.msg import TaskStatus, LBRCommand, NavCommand
from archemist.util.rosMsgCoder import rosMsgCoder
from archemist.state.robots import KukaLBRIIWA
from archemist.state.state import State
from archemist.util.location import Location
from rospy.core import is_shutdown

class KukaHandler:
    def __init__(self):
        rospy.init_node("kuka_handler")
        print("kuka_handler running")
        # Read state from database
        self.state = State()
        self.state.initializeState(False)
        self._kukaRobotState = self.state.getRobot('KukaLBRIIWA',1)
        self.coder = rosMsgCoder()
        #self._dbhandler = dbHandler.dbHandler()
        self._ackpub = rospy.Publisher("/processing/HandlerReturnBus", HandlerBusMessage, queue_size=1)
        self._kmrCmdPub = rospy.Publisher('/kuka2/kmr/nav_commands', NavCommand, queue_size=1)
        self._lbrCmdPub = rospy.Publisher('/kuka2/lbr/command', LBRCommand, queue_size=1)
        rospy.Subscriber('/kuka2/kmr/task_status', TaskStatus, self.kmr_task_cb, queue_size=2)
        rospy.Subscriber('/kuka2/lbr/task_status', TaskStatus, self.lbr_task_cb, queue_size=2)
        rospy.Subscriber("/processing/HandlerBus", HandlerBusMessage, self.handler_cb)
        self.kmr_task = ''
        self.kmr_done = False
        self.lbr_task = ''
        self.lbr_done = False

        while (rospy.is_shutdown()):
            self.handle()
            rospy.sleep(3)


    def handler_cb(self, msg):
        if(msg.station_name == self._kukaRobotState.__class__.__name__ and msg.station_id == self._kukaRobotState.id):
            rospy.loginfo("Receiving Handler")
            opDescriptor = self.coder.decode(msg.opDescriptor)
            #self._pandaState.setRobotOp(opDescriptor)
            #self.dbhandler.updateRobotState("pandaFranka", vars(opDescriptor))
            if (opDescriptor.__class__.__name__ == 'KukaMoveBaseOpDescriptor'):
                navCommand = self.process_kmr_op(opDescriptor)
                kmr_task = '{0}nav to n:{1} g:{2}'.format('fine_' if navCommand.fine_localization else '' , navCommand.node_id, navCommand.graph_id)
                rospy.loginfo('exec ' + kmr_task)
                self._kmrCmdPub.publish(navCommand)
                self.wait_for_kmr()
                # Create result discriptor --=--==-=-=-=-=-=-
                # TODO calibrate the arm after every motion
            elif (opDescriptor.__class__.__name__ == 'KukaMoveArmOpDescriptor'):
                armCommand = self.process_lbr_op(opDescriptor)
                lbr_task = armCommand.job_name
                print('exec ' + lbr_task)
                self._lbrCmdPub.publish(armCommand)
                self.wait_for_lbr()
            elif (opDescriptor.__class__.__name__ == 'KukaCalibrateArmOpDescriptor'):
                armCommand = self.process_calib_op(opDescriptor)
                lbr_task = armCommand.job_name
                print('exec ' + lbr_task)
                self._lbrCmdPub.publish(armCommand)
                self.wait_for_lbr()


    def kmr_task_cb(self, msg):
        if msg.task_name == self.kmr_task and msg.task_state == TaskStatus.FINISHED:
            self.kmr_task = ''
            self.kmr_done = True

    def wait_for_kmr(self):
        while(not self.kmr_done):
            rospy.sleep(0.1)
        self.kmr_done = False

    def lbr_task_cb(self, msg):
        if msg.task_name == self.lbr_task and msg.task_state == TaskStatus.FINISHED:
            self.lbr_task = ''
            self.lbr_done = True

    def wait_for_lbr(self):
        while(not self.lbr_done):
            rospy.sleep(0.1)
        self.lbr_done = False

    def process_kmr_op(self, robotOp):
        return NavCommand(robot_id=robotOp.robot_id,graph_id=robotOp.target_loc.graph_id,node_id=robotOp.target_loc.node_id,fine_localization=robotOp.fine_localization)

    def process_lbr_rack_op(self, robotOp):
        if robotOp.start_pos == 'quantos_rack':
            return LBRCommand(job_name='quantosPickRack_job')
        else:
            rospy.logerr('unknown panda op')
        return None

    def process_lbr_vial_op(self, robotOp):
        if robotOp.start_pos == 'quantos_rack' and robotOp.end_pos == 'quantos_carousel':
            return LBRCommand(job_name='quantosLoad_job')
        elif robotOp.start_pos == 'quantos_carousel' and robotOp.end_pos == 'quantos_rack':
            return LBRCommand(job_name='quantosUnload_job')
        elif robotOp.start_pos == 'robot_rack' and robotOp.end_pos == 'handover_cube':
            return LBRCommand(job_name='pandaLoad_job')
        elif robotOp.start_pos == 'handover_cube' and robotOp.end_pos == 'robot_rack':
            return LBRCommand(job_name='pandaUnload_job')
        else:
            rospy.logerr('unknown panda op')
        return None

    def process_calib_op(self, robotOp):
        if robotOp.location == 'quantos':
            return LBRCommand(job_name='quantosStation_calib')
        elif robotOp.location == 'panda':
            return LBRCommand(job_name='pandaStation_calib')
        else:
            rospy.logerr('unknown panda op')
        return None


    def handle(self):
        self.state.updateFromDB()
        self._kukaRobotState = self.state.getRobot('KukaLBRIIWA',1)
        
        if self._kukaRobotState._assigned_batch is not None:
            opDescriptor = self._kukaRobotState._assigned_batch.getCurrentOp()
            opDescriptor.addTimeStamp()
            
            if (opDescriptor.__class__.__name__ == 'KukaMoveBaseOpDescriptor'):
                navCommand = self.process_kmr_op(opDescriptor)
                kmr_task = '{0}nav to n:{1} g:{2}'.format('fine_' if navCommand.fine_localization else '' , navCommand.node_id, navCommand.graph_id)
                rospy.loginfo('exec ' + kmr_task)
                self._kmrCmdPub.publish(navCommand)
                self.wait_for_kmr()
                self._kukaRobotState.location = opDescriptor.target_loc
                
                if (opDescriptor.target_loc.get_map_coordinates() == (7,7)):
                    armCommand = LBRCommand(job_name='quantosStation_calib')
                    lbr_task = armCommand.job_name
                    print('exec ' + lbr_task)
                    self._lbrCmdPub.publish(armCommand)
                    self.wait_for_lbr()
                elif (opDescriptor.target_loc.get_map_coordinates() == (8,7)):
                    armCommand = LBRCommand(job_name='pandaStation_calib')
                    lbr_task = armCommand.job_name
                    print('exec ' + lbr_task)
                    self._lbrCmdPub.publish(armCommand)
                    self.wait_for_lbr()
                # Create result discriptor --=--==-=-=-=-=-=-
                # TODO calibrate the arm after every motion
                self._kukaRobotState._processed_batch =self._kukaRobotState._assigned_batch
                self._kukaRobotState.location =  Location(opDescriptor.target_loc.node_id, opDescriptor.target_loc.graph_id, 'robot_rack')
                self._kukaRobotState._processed_batch.location = Location(opDescriptor.target_loc.node_id, opDescriptor.target_loc.graph_id, 'robot_rack')
                self._kukaRobotState._processed_batch.advanceProcessState()
                self._kukaRobotState._assigned_batch = None
                self.state.modifyObjectDB(self._kukaRobotState)
            
            elif (opDescriptor.__class__.__name__ == 'KukaVialMoveOpDescriptor'):
                if (self._kukaRobotState.get_map_coordinates() != opDescriptor.target_loc.get_map_coordinates()):
                    navCommand = NavCommand(robot_id=self._kukaRobotState.id,graph_id=opDescriptor.target_loc.graph_id,node_id=opDescriptor.target_loc.node_id,fine_localization=True)
                    kmr_task = '{0}nav to n:{1} g:{2}'.format('fine_' if navCommand.fine_localization else '' , navCommand.node_id, navCommand.graph_id)
                    rospy.loginfo('exec ' + kmr_task)
                    self._kmrCmdPub.publish(navCommand)
                    self.wait_for_kmr()
                    self._kukaRobotState.location = opDescriptor.target_loc
                    if (opDescriptor.target_loc.get_map_coordinates() == (7,7)):
                        armCommand = LBRCommand(job_name='quantosStation_calib')
                        lbr_task = armCommand.job_name
                        print('exec ' + lbr_task)
                        self._lbrCmdPub.publish(armCommand)
                        self.wait_for_lbr()
                    elif (opDescriptor.target_loc.get_map_coordinates() == (8,7)):
                        armCommand = LBRCommand(job_name='pandaStation_calib')
                        lbr_task = armCommand.job_name
                        print('exec ' + lbr_task)
                        self._lbrCmdPub.publish(armCommand)
                        self.wait_for_lbr()

                armCommand = self.process_lbr_vial_op(opDescriptor)
                lbr_task = armCommand.job_name
                print('exec ' + lbr_task)
                self._lbrCmdPub.publish(armCommand)
                self.wait_for_lbr()
                
                self._kukaRobotState._processed_batch =self._kukaRobotState._assigned_batch
                self._kukaRobotState._processed_batch.getCurrentSample().location = opDescriptor.target_loc
                self._kukaRobotState._processed_batch.advanceProcessState()
                self._kukaRobotState._assigned_batch = None
                self.state.modifyObjectDB(self._kukaRobotState)

            elif (opDescriptor.__class__.__name__ == 'KukaPickOpDescriptor'):
                armCommand = self.process_lbr_rack_op(opDescriptor)
                lbr_task = armCommand.job_name
                print('exec ' + lbr_task)
                self._lbrCmdPub.publish(armCommand)
                self.wait_for_lbr()

                self._kukaRobotState._processed_batch =self._kukaRobotState._assigned_batch
                self._kukaRobotState._processed_batch = Location(opDescriptor.target_loc.node_id, opDescriptor.target_loc.graph_id, 'robot_rack')
                self._kukaRobotState._processed_batch.advanceProcessState()
                self._kukaRobotState._assigned_batch = None
                self.state.modifyObjectDB(self._kukaRobotState)
