import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import SyringePumpDispenseOpDescriptor
from archemist.core.processing.handler import StationHandler
from std_msgs.msg import String
from roslabware_msgs.msg import TecanXlp6000Cmd, TecanXlp6000Reading
from std_msgs.msg import Bool
import time
from threading import Thread

class SyringePumpStationROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        self._syringe_pump_current_task_complete = False
        rospy.init_node(f'{self._station}_handler')
        self.pub_pump = rospy.Publisher("/Tecan_XLP6000_Commands", TecanXlp6000Cmd, queue_size=1)
        rospy.Subscriber('/tecan_xlp/task_complete', Bool, self._syringe_pump_state_update, queue_size=1)
        rospy.sleep(1)
        self._thread = None

    def execute_op(self):
        current_op = self._station.get_assigned_station_op()
        print(f'performing syringe pump operation {current_op}')

        if (isinstance(current_op,SyringePumpDispenseOpDescriptor)):
            # while True:
            #     if self._syringe_pump_current_task_complete:
            #         break
            rospy.loginfo('starting dispence operation')
            self._syringe_pump_current_task_complete = False
            for i in range(10):
                self.pub_pump.publish(tecan_xlp_command=TecanXlp6000Cmd.DISPENSE, xlp_withdraw_port = current_op.withdraw_port, xlp_dispense_port = current_op.dispense_port, xlp_volume = current_op.volume , xlp_speed = current_op.speed)
        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

        time.sleep(1)

    def is_op_execution_complete(self):
        return self._syringe_pump_current_task_complete

    def get_op_result(self) -> Tuple[bool, Dict]:
        return True, {}

    def run(self):
        print(f'{self._station}_handler is running')
        try:
            while True:
                self.handle()
                time.sleep(2)
        except KeyboardInterrupt:
            print(f'{self._station}_handler is terminating!!!')

    def _syringe_pump_state_update(self, msg):
        if msg.data == True:
            self._syringe_pump_current_task_complete = True
        elif msg.data == False:
            self._syringe_pump_current_task_complete = False