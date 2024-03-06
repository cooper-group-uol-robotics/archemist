from typing import Tuple, List, Type
from archemist.core.state.station import Station
from .state import CSOpenDoorOp, CSCloseDoorOp, CSRunJobOp, CSLiquidDispenseOp
from archemist.core.processing.handler import StationOpHandler, SimStationOpHandler
from archemist.core.state.station_op_result import StationOpResult, MaterialOpResult
from archemist.core.util.enums import OpOutcome


class SimChemSpeedFlexHandler(SimStationOpHandler):
    def __init__(self, station: Station):
        super().__init__(station)

    def get_op_result(self) -> Tuple[OpOutcome, List[Type[StationOpResult]]]:
        current_op = self._station.assigned_op
        if (isinstance(current_op, CSLiquidDispenseOp)):
            materials_names = [material_name for material_name in current_op.dispense_table.keys()]
            samples_qtys = zip(*[qtys for qtys in current_op.dispense_table.values()])
            dispense_results = []
            for qty_tuple in samples_qtys:
                sample_op_result = MaterialOpResult.from_args(origin_op=current_op.object_id,
                                                              material_names=materials_names,
                                                              amounts=list(qty_tuple),
                                                              units=[current_op.dispense_unit]*len(materials_names))
                dispense_results.append(sample_op_result)
            return OpOutcome.SUCCEEDED, dispense_results
        else:
            return OpOutcome.SUCCEEDED, None


try:
    import rospy
    from chemspeed_flex_msgs.msg import CSFlexCommand, CSFlexStatus

    class ChemSpeedFlexROSHandler(StationOpHandler):
        def __init__(self, station: Station):
            super().__init__(station)

        def initialise(self) -> bool:
            self._current_cs_status = None
            self._desired_cs_status = None
            rospy.init_node(f'{self._station}_handler')
            self.pubCS_Flex = rospy.Publisher("/ChemSpeed_Flex_commands", CSFlexCommand, queue_size=1)
            rospy.Subscriber('/ChemSpeed_Flex_status', CSFlexStatus, self._cs_state_update, queue_size=1)
            rospy.sleep(2)
            if self._current_cs_status:
                return True
            else:
                return False

        def execute_op(self):
            current_op = self._station.assigned_op
            if (isinstance(current_op, CSOpenDoorOp)):
                rospy.loginfo('opening chemspeed door')
                for i in range(10):
                    self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.OPEN_DOOR)
                self._desired_cs_status = CSFlexStatus.DOOR_OPEN
            elif (isinstance(current_op, CSCloseDoorOp)):
                rospy.loginfo('closing chemspeed door')
                for i in range(10):
                    self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.CLOSE_DOOR)
                self._desired_cs_status = CSFlexStatus.DOOR_CLOSED
            elif (isinstance(current_op, CSRunJobOp)):
                rospy.loginfo('starting chemspeed job')
                for i in range(10):
                    self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.RUN_APP)
                self._desired_cs_status = CSFlexStatus.JOB_COMPLETE
            elif (isinstance(current_op, CSLiquidDispenseOp)):
                rospy.loginfo('uploading csv file to rosparam server')
                rospy.set_param('chemspeed_input_csv', current_op.to_csv_string())
                rospy.sleep(3)  # wait for csv to be uploaded
                rospy.loginfo('starting chemspeed job')
                for i in range(10):
                    self.pubCS_Flex.publish(cs_flex_command=CSFlexCommand.RUN_APP)
                self._desired_cs_status = CSFlexStatus.JOB_COMPLETE
            else:
                rospy.logwarn(f'[{self.__class__.__name__}] Unkown operation was received')

        def is_op_execution_complete(self) -> bool:
            if self._desired_cs_status == self._current_cs_status:
                self._desired_cs_status = None
                return True
            else:
                return False

        def get_op_result(self) -> Tuple[OpOutcome, List[Type[StationOpResult]]]:
            current_op = self._station.assigned_op
            if (isinstance(current_op, CSLiquidDispenseOp)):
                materials_names = [material_name for material_name in current_op.dispense_table.keys()]
                samples_qtys = zip(*[qtys for qtys in current_op.dispense_table.values()])
                dispense_results = []
                for qty_tuple in samples_qtys:
                    sample_op_result = MaterialOpResult.from_args(origin_op=current_op.object_id,
                                                                  material_names=materials_names,
                                                                  amounts=list(qty_tuple),
                                                                  units=[current_op.dispense_unit]*len(materials_names))
                    dispense_results.append(sample_op_result)
                return OpOutcome.SUCCEEDED, dispense_results
            else:
                return OpOutcome.SUCCEEDED, None

        def shut_down(self):
            pass

        def _cs_state_update(self, msg):
            if self._current_cs_status != msg.cs_flex_status:
                self._current_cs_status = msg.cs_flex_status
except ImportError:
    pass
