from typing import Tuple, List, Optional
from archemist.core.state.station import Station
from .state import (LCMSSampleAnalysisOp,
                    LCMSPrepAnalysisOp,
                    LCMSAnalysisResult, 
                    LCMSInsertRackOp, 
                    LCMSEjectRackOp)
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from archemist.core.util.enums import OpOutcome
import random
from threading import Thread
import socket

random.seed(123)


class SimWatersLCMSStationHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, Optional[List[LCMSAnalysisResult]]]:
            current_op = self._station.assigned_op
            if isinstance(current_op, LCMSSampleAnalysisOp):
                result = LCMSAnalysisResult.from_args(origin_op=current_op.object_id,
                                                      concentration=random.choice([0.99, 0.01]),
                                                      result_filename="file.xml")
                return OpOutcome.SUCCEEDED, [result]
            else:
                return OpOutcome.SUCCEEDED, None

class WaterLCMSSocketHandler(StationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def initialise(self) -> bool:
        self._host = '192.168.1.1'
        self._port = 19990
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.connect((self._host, self._port))
        self._thread = None
        self._result_received = False
        self._op_result = False

    def shut_down(self):
        self._socket.close()

    def execute_op(self):
        current_op = self._station.assigned_op
        self._result_received = False
        self._op_result = False
        if isinstance(current_op,LCMSInsertRackOp):
            print(f'Autosampler - inserting rack {current_op.rack}')
            # msg = f'InsertRack{current_op.rack}'
            # self._socket.sendall(msg.encode('ascii'))
            msg = "load"
            self._socket.sendall(msg.encode('utf-8'))

        elif isinstance(current_op,LCMSEjectRackOp):
            print(f'Autosampler - extracting rack {current_op.rack}')
            # msg = f'ExtractRack{current_op.rack}'
            # self._socket.sendall(msg.encode('ascii'))
            msg = "unload"
            self._socket.sendall(msg.encode('utf-8'))
        
        elif isinstance(current_op,LCMSPrepAnalysisOp):
            msg = "unload"
            self._socket.sendall(msg.encode('utf-8'))

        elif isinstance(current_op,LCMSSampleAnalysisOp):
            self._socket.sendall(b'StartAnalysisRack2')
        else:
            print(f'[{self.__class__.__name__}] Unkown operation was received')
        self._thread = Thread(target=self._lcsm_status_update, daemon=True)

    def is_op_execution_complete(self) -> bool:
        return self._result_received

    def get_op_result(self) -> Tuple[OpOutcome, Optional[List[LCMSAnalysisResult]]]:
            current_op = self._station.assigned_op
            if isinstance(current_op, LCMSSampleAnalysisOp):
                result = LCMSAnalysisResult.from_args(origin_op=current_op.object_id,
                                                      concentration=0.5,
                                                      result_filename="file.xml") #TODO need to receive from LCMS
                return OpOutcome.SUCCEEDED, [result]
            else:
                return OpOutcome.SUCCEEDED, None

    def _lcsm_status_update(self):
        msg = self._socket.recv(1024)
        self._result_received = True
        if msg == b'Completed':
            self._op_result = True
        elif msg == b'Error':
            self._op_result = False


import rospy
from roslabware_msgs.msg import LcmsCmd, LcmsReading, LcmsTask
from std_msgs.msg import Bool

class WaterLCMSRosHandler(StationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def initialise(self) -> bool:
        rospy.init_node(f'{self._station}_handler')
        self._lcms_pub = rospy.Publisher("/lcms_command", LcmsCmd, queue_size=2)
        rospy.Subscriber("/lcms/task_complete", LcmsTask, self.lcms_callback)
        self._op_complete = False
        self.check_cb = None
        self._op_results = {}
        self._seq_id = 1
        rospy.sleep(2)
        return True
    
    def execute_op(self):
        current_op = self._station.assigned_op
        self._result_received = False
        self._op_result = False
        if isinstance(current_op,LCMSInsertRackOp):
            print(f'Autosampler - inserting rack {current_op}')
            for i in range(10):
                self._lcms_pub.publish(seq=self._seq_id, lcms_command=LcmsCmd.LOAD_BATCH)

        elif isinstance(current_op,LCMSEjectRackOp):
            print(f'Autosampler - extracting rack {current_op}')
            for i in range(10):
                self._lcms_pub.publish(seq=self._seq_id, lcms_command=LcmsCmd.UNLOAD_BATCH)

        elif isinstance(current_op,LCMSPrepAnalysisOp):
            print('LCMS Prep Operation')
            for i in range(10):
                self._lcms_pub.publish(seq=self._seq_id, lcms_command=LcmsCmd.START_PREP, lcms_num_samples = 1)

        elif isinstance(current_op,LCMSSampleAnalysisOp):
            print('LCMS Analysis Operation')
            for i in range(10):
                self._lcms_pub.publish(seq=self._seq_id, lcms_command=LcmsCmd.START_ANALYSIS)
        else:
            print(f'[{self.__class__.__name__}] Unkown operation was received')

    def is_op_execution_complete(self) -> bool: 
        return self._op_complete
    
    def get_op_result(self) -> Tuple[OpOutcome, Optional[List[LCMSAnalysisResult]]]:
        current_op = self._station.assigned_op
        if isinstance(current_op, LCMSSampleAnalysisOp):
            result = LCMSAnalysisResult.from_args(origin_op=current_op.object_id,
                                                    concentration=0.5,
                                                    result_filename="file.xml") #TODO need to receive from LCMS
            return OpOutcome.SUCCEEDED, [result]
        else:
            return OpOutcome.SUCCEEDED, None

    def shut_down(self):
        pass

    def lcms_callback(self, msg):
        self.check_cb = False
        print("I am outside condition.")
        if msg.seq == self._seq_id and msg.complete:
            self.check_cb = True
            print("I am inside condition.")
            self._op_complete = msg.complete
            self._seq_id+=1