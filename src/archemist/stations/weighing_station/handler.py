import rospy
import time
from typing import Dict, Tuple, List, Union
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from archemist.core.state.station import Station
from roslabware_msgs.msg import KernPCB2500Cmd, KernPCB2500Reading, KernDoorCmd, KernDoorStatus, sashDoorCmd, sashDoorStatus
from .state import (
    WeighingStation, 
    WeighingVOpenDoorOp, 
    WeighingVCloseDoorOp, 
    BalanceOpenDoorOp, 
    BalanceCloseDoorOp,
    LoadFunnelOp,
    UnloadFunnelOp,
    TareOp,
    WeighingOp,
    WeighResult
)
from archemist.core.util.enums import OpOutcome
import random

class SimWeighingStationHandler(SimStationOpHandler):
    def __init__(self, station: WeighingStation):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[WeighResult]]:
        current_op = self._station.assigned_op
        result = WeighResult.from_args(
            origin_op=current_op.object_id,
            reading_value=random.random()*100,
            unit="g"
        )
        return OpOutcome.SUCCEEDED, [result]
    

try:  
    import rospy
    from roslabware_msgs.msg import KernPCB2500Cmd, KernPCB2500Reading
    from roslabware_msgs.msg import KernDoorCmd, KernDoorStatus 
    from roslabware_msgs.msg import sashDoorCmd, sashDoorStatus #TODO is this right?  Maybe don't need to import at the top of file

    class WeighingStationROSHandler(StationOpHandler):
        def __init__(self, station:WeighingStation):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self._pub_balance = rospy.Publisher("kern_PCB2500_Commands", KernPCB2500Cmd, queue_size=2)
            self._pub_door = rospy.Publisher("kern_door_Commands", KernDoorCmd, queue_size=2)
            self._pub_sash = rospy.Publisher("sash_door_Commands", sashDoorCmd, queue_size=2)
            rospy.Subscriber("kern_PCB2500_Readings", KernPCB2500Reading, self.weight_callback)
            rospy.Subscriber("kern_Door_Status", KernDoorStatus, self.door_callback)
            rospy.Subscriber("sash_door_Status", sashDoorStatus, self.sash_callback)
            self._current_balance_door_status = None #TODO is this needed?
            self._current_sash_door_status = None #TODO is this needed?
            self._received_mass = False
            self._op_results = {}
            rospy.sleep(2)
            return True
        
        # TODO Satheesh had this whole run method - needed?
        # def run(self):
        #     rospy.loginfo(f'{self._station}_handler is running')
        #     try:
        #         while not rospy.is_shutdown():
        #             self.handle()
        #             rospy.sleep(2)
        #     except KeyboardInterrupt:
        #         rospy.loginfo(f'{self._station}_handler is terminating!')

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_result = None

            if isinstance(current_op, WeighingOp):
                rospy.loginfo('Reading stable weight.')
                for i in range(10):
                    self._pub_balance.publish(kern_command=KernPCB2500Cmd.GET_MASS_STABLE)
            elif isinstance(current_op, TareOp):
                rospy.loginfo('Taring balance.')
                for i in range(10):
                    self._pub_balance.publish(kern_command=KernPCB2500Cmd.TARE_BALANCE)
            elif isinstance(current_op, WeighingVOpenDoorOp):
                rospy.loginfo('Opening vertical door.')
                for i in range(10):
                    self._pub_sash.publish(sash_door_command=sashDoorCmd.OPEN_DOOR)
            elif isinstance(current_op, WeighingVCloseDoorOp):
                rospy.loginfo('Closing vertical door.')
                for i in range(10):
                    self._pub_sash.publish(sash_door_command=sashDoorCmd.CLOSE_DOOR) 
            elif isinstance(current_op, BalanceOpenDoorOp):
                rospy.loginfo('Opening balance door.')
                for i in range(10):
                    self._pub_door.publish(kern_door_command=KernDoorCmd.OPEN_DOOR)
            elif isinstance(current_op, BalanceCloseDoorOp):
                rospy.loginfo('Closing balance door.')
                for i in range(10):
                    self._pub_door.publish(kern_door_command=KernDoorCmd.CLOSE_DOOR)
            elif isinstance(current_op, LoadFunnelOp):
                rospy.loginfo('Loading funnel onto balance.')
                for i in range(10):
                    #TODO send ROS command to KUKA - how?
                    pass
            elif isinstance(current_op, UnloadFunnelOp):
                rospy.loginfo('Unloading funnel fro, balance.')
                for i in range(10):
                    #TODO send ROS command to KUKA - how?
                    pass
            else:
                rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

        def is_op_execution_complete(self) -> bool: #TODO not sure what this is doing
            return self._op_result is not None

        def get_op_result(self) -> Tuple[OpOutcome, List[WeighResult]]: #TODO not sure what this is doing
            return OpOutcome.SUCCEEDED, [self._op_result]
        
        def shut_down(self):
            pass

        def weight_callback(self, msg):
            origin_op_object_id = self._station.assigned_op.object_id #TODO not sure what this line does?
            if not msg.mass == 0:
                self._op_results['mass'] = msg.mass
                self._op_results = WeighResult.from_args(
                    origin_op=origin_op_object_id,
                    reading_value=msg.mass,
                    unit='g'
                )
                rospy.loginfo(f'The weight of the funnel is {msg.mass}g.')
                self._received_mass = True
            else:
                rospy.loginfo('Invalid message from the ROS driver!')
        
        def door_callback(self, msg):
            if self._current_balance_door_status != str(msg.status):
                    self._current_balance_door_status = str(msg.status)

        def sash_callback(self, msg):
            if self._current_sash_door_status != str(msg.status):
                    self._current_sash_door_status = str(msg.status)

except ImportError:
    pass