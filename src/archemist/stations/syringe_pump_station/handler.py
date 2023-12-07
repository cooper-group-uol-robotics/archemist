from typing import Tuple, List, Optional
from archemist.core.state.station import Station
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from archemist.core.state.station_op_result import MaterialOpResult, ProcessOpResult
from .state import (SyringePumpStation, SyringePumpDispenseVolumeOp, SyringePumpDispenseRateOp,SyringePumpFinishDispensingOp)
from archemist.core.util.enums import OpOutcome
from random import random

class SimSyringePumpStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
        op = self._station.assigned_op
        if isinstance(op, SyringePumpDispenseVolumeOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                material_names=[op.liquid_name],
                                                amounts=[op.dispense_volume],
                                                units=[op.dispense_unit])
        elif isinstance(op, SyringePumpFinishDispensingOp):
            result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                material_names=[op.liquid_name],
                                                amounts=[random()*5],
                                                units=["mL"])
        elif isinstance(op, SyringePumpDispenseRateOp):
            parameters = {
                "dispense_rate": op.dispense_rate,
                "rate_unit": op.rate_unit
            }
            result = ProcessOpResult.from_args(origin_op=op.object_id,
                                                parameters=parameters)

        return OpOutcome.SUCCEEDED, [result]

try:  
    import rospy
    from roslabware_msgs.msg import TecanXlp6000Cmd, TecanXlp6000Reading, TecanXlp6000Task
    from std_msgs.msg import Bool

    class APCSyringePumpStationRosHandler(StationOpHandler):
        def __init__(self, station:SyringePumpStation):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self._pub_pump = rospy.Publisher("tecan_xlp6000_command", TecanXlp6000Cmd, queue_size=2)
            rospy.Subscriber("/tecan_xlp/task_complete", TecanXlp6000Task, self.syringe_pump_callback)
            self._op_complete = False
            self._op_results = {}
            self._seq_id = 1
            rospy.sleep(2)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_complete = False
            if isinstance(current_op, SyringePumpDispenseVolumeOp):
                rospy.loginfo(f'Sending Dispense Volume Command (volume : {current_op.dispense_volume} {current_op.dispense_unit} of {current_op.liquid_name})')
                _withdraw_port = self._station.liquids_dict[current_op.liquid_name]['details']['inlet_port']
                _dispense_port = self._station.liquids_dict[current_op.liquid_name]['details']['outlet_port']
                for i in range(10):
                    self._pub_balance.publish(seq=self._seq_id, tecan_xlp_command=TecanXlp6000Cmd.DISPENSE, xlp_withdraw_port=_withdraw_port, xlp_dispense_port=_dispense_port, xlp_volume = current_op.dispense_volume, xlp_speed = current_op.dispense_rate)
            elif isinstance(current_op, SyringePumpDispenseRateOp):
                _withdraw_port = self._station.liquids_dict[current_op.liquid_name]['details']['inlet_port']
                _dispense_port = self._station.liquids_dict[current_op.liquid_name]['details']['outlet_port']
                rospy.loginfo(f'Sending Dispense Rate Command (rate : {current_op.dispense_rate} {current_op.rate_unit} of {current_op.liquid_name})')
                for i in range(10):
                    self._pub_balance.publish(seq=self._seq_id, tecan_xlp_command=TecanXlp6000Cmd.DISPENSE, xlp_withdraw_port=_withdraw_port, xlp_dispense_port=_dispense_port, xlp_speed = current_op.dispense_rate)
            elif isinstance(current_op, SyringePumpFinishDispensingOp):
                rospy.loginfo(f'Sending Stop Dispense Command')
                for i in range(10):
                    self._pub_door.publish(seq=self._seq_id, tecan_xlp_command=TecanXlp6000Cmd.STOP)
            else:
                rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')
            

        def is_op_execution_complete(self) -> bool: 
            return self._op_complete

        def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
            op = self._station.assigned_op
            if isinstance(op, SyringePumpDispenseVolumeOp):
                result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                    material_names=[op.liquid_name],
                                                    amounts=[op.dispense_volume],
                                                    units=[op.dispense_unit])
            elif isinstance(op, SyringePumpFinishDispensingOp):
                result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                    material_names=[op.liquid_name],
                                                    amounts=[random()*5],
                                                    units=["mL"])
            elif isinstance(op, SyringePumpDispenseRateOp):
                parameters = {
                    "dispense_rate": op.dispense_rate,
                    "rate_unit": op.rate_unit
                }
                result = ProcessOpResult.from_args(origin_op=op.object_id,
                                                    parameters=parameters)
            return OpOutcome.SUCCEEDED, [result]
        
        def shut_down(self):
            pass

        def syringe_pump_callback(self, msg):
            if msg.seq == self._seq_id and msg.complete:
                self._op_complete = msg.complete
                self._seq_id+=1


except ImportError:
    pass

try:  
    import rospy
    from roslabware_msgs.msg import TecanXlp6000Cmd, TecanXlp6000Reading, TecanXlp6000Task
    from std_msgs.msg import Bool

    class APCSyringePumpStationRosHandler(StationOpHandler):
        def __init__(self, station:SyringePumpStation):
            super().__init__(station)

        def initialise(self) -> bool:
            rospy.init_node(f'{self._station}_handler')
            self._pub_pump = rospy.Publisher("tecan_xlp6000_command", TecanXlp6000Cmd, queue_size=2)
            rospy.Subscriber("/tecan_xlp/task_complete", TecanXlp6000Task, self.syringe_pump_callback)
            self._op_complete = False
            self._op_results = {}
            self._seq_id = 1
            rospy.sleep(2)
            return True

        def execute_op(self):
            current_op = self._station.assigned_op
            self._op_complete = False
            if isinstance(current_op, SyringePumpDispenseVolumeOp):
                rospy.loginfo(f'Sending Dispense Volume Command (volume : {current_op.dispense_volume} {current_op.dispense_unit} of {current_op.liquid_name})')
                _withdraw_port = self._station.liquids_dict[current_op.liquid_name]['details']['inlet_port']
                _dispense_port = self._station.liquids_dict[current_op.liquid_name]['details']['outlet_port']
                for i in range(10):
                    self._pub_balance.publish(seq=self._seq_id, tecan_xlp_command=TecanXlp6000Cmd.DISPENSE, xlp_withdraw_port=_withdraw_port, xlp_dispense_port=_dispense_port, xlp_volume = current_op.dispense_volume, xlp_speed = current_op.dispense_rate)
            elif isinstance(current_op, SyringePumpDispenseRateOp):
                _withdraw_port = self._station.liquids_dict[current_op.liquid_name]['details']['inlet_port']
                _dispense_port = self._station.liquids_dict[current_op.liquid_name]['details']['outlet_port']
                rospy.loginfo(f'Sending Dispense Rate Command (rate : {current_op.dispense_rate} {current_op.rate_unit} of {current_op.liquid_name})')
                for i in range(10):
                    self._pub_balance.publish(seq=self._seq_id, tecan_xlp_command=TecanXlp6000Cmd.DISPENSE, xlp_withdraw_port=_withdraw_port, xlp_dispense_port=_dispense_port, xlp_speed = current_op.dispense_rate)
            elif isinstance(current_op, SyringePumpFinishDispensingOp):
                rospy.loginfo(f'Sending Stop Dispense Command')
                for i in range(10):
                    self._pub_door.publish(seq=self._seq_id, tecan_xlp_command=TecanXlp6000Cmd.STOP)
            else:
                rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')
            

        def is_op_execution_complete(self) -> bool: 
            return self._op_complete

        def get_op_result(self) -> Tuple[OpOutcome, List[MaterialOpResult]]:
            op = self._station.assigned_op
            if isinstance(op, SyringePumpDispenseVolumeOp):
                result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                    material_names=[op.liquid_name],
                                                    amounts=[op.dispense_volume],
                                                    units=[op.dispense_unit])
            elif isinstance(op, SyringePumpFinishDispensingOp):
                result = MaterialOpResult.from_args(origin_op=op.object_id,
                                                    material_names=[op.liquid_name],
                                                    amounts=[random()*5],
                                                    units=["mL"])
            elif isinstance(op, SyringePumpDispenseRateOp):
                parameters = {
                    "dispense_rate": op.dispense_rate,
                    "rate_unit": op.rate_unit
                }
                result = ProcessOpResult.from_args(origin_op=op.object_id,
                                                    parameters=parameters)
            return OpOutcome.SUCCEEDED, [result]
        
        def shut_down(self):
            pass

        def syringe_pump_callback(self, msg):
            if msg.seq == self._seq_id and msg.complete:
                self._op_complete = msg.complete
                self._seq_id+=1


except ImportError:
    pass