from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.robot_op import RobotTaskOpDescriptor, CollectBatchOpDescriptor
from archemist.core.state.station_process import StationProcess, StationProcessModel
from archemist.core.state.station import Station

from typing import Union

class PXRDWorkflowShakingProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        
        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'),
            State(name='load_shaker_plate', on_enter='request_load_shaker_plate'),
            State(name='shake', on_enter='request_shake_op'),
            State(name='unload_shaker_plate', on_enter='request_unload_shaker_plate'),
            State(name='unscrew_caps', on_enter='request_unscrew_caps'),
            State(name='pick_lot', on_enter='request_lot_pickup'),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'load_shaker_plate'},
            {'source':'load_shaker_plate','dest':'shake', 'conditions':'are_req_robot_ops_completed'},
            {'source':'shake','dest':'unload_shaker_plate', 'conditions':'are_req_station_ops_completed'},
            {'source':'unload_shaker_plate','dest':'unscrew_caps', 'conditions':'are_req_robot_ops_completed'},
            {'source':'unscrew_caps','dest':'pick_lot', 'conditions':'are_req_robot_ops_completed'},
            {'source':'pick_lot','dest':'final_state', 'conditions':'are_req_robot_ops_completed'}
        ]

    ''' states callbacks '''

    def request_load_shaker_plate(self):
        batch_1 = self.lot.batches[0]
        robot_op = RobotTaskOpDescriptor.from_args(name='invertAndLoadShakerPlate', target_robot="FixedRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_shake_op(self):
        batch_1 = self.lot.batches[0]
        current_op = self.generate_operation("shake_samples", target_batch=batch_1)
        self.request_station_op(current_op)

    def request_unload_shaker_plate(self):
        batch_1 = self.lot.batches[0]
        robot_op = RobotTaskOpDescriptor.from_args(name='invertAndLoadWellPlate', target_robot="FixedRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_unscrew_caps(self):
        batch_1 = self.lot.batches[0]
        robot_op = RobotTaskOpDescriptor.from_args(name='unscrewCaps', target_robot="FixedRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_lot_pickup(self):
        # pickup the 8-well rack
        params_dict = {}
        batch_1 = self.lot.batches[0]
        params_dict["perform_6p_calib"] = False
        robot_op_1 = CollectBatchOpDescriptor.from_args(name='UnloadEightWRackYumiStation', target_robot="MobileRobot",
                                                       params=params_dict, target_batch=batch_1)

        # pickup the pxrd rack
        batch_2 = self.lot.batches[1]
        params_dict = {}
        params_dict["perform_6p_calib"] = True
        robot_op_2 = CollectBatchOpDescriptor.from_args(name='UnloadPXRDRackYumiStation', target_robot="MobileRobot",
                                                       params=params_dict, target_batch=batch_2)
        
        self.request_robot_ops([robot_op_1, robot_op_2])
