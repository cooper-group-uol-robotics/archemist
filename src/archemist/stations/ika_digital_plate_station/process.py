from transitions import State
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.robot_op import DropBatchOpDescriptor, RobotTaskOpDescriptor
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import Union


class PXRDWorkflowStirringProcess(StationProcess):
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [ State(name='init_state'),
            State(name='prep_state'), 
            State(name='place_lot', on_enter=['request_place_lot']),
            State(name='load_stir_plate', on_enter=['request_load_stir_plate']),
            State(name='stir', on_enter=['request_stir_op']),
            State(name='final_state')]

        ''' Transitions '''
        self.TRANSITIONS = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state','dest':'place_lot'},
            {'source':'place_lot', 'dest': 'load_stir_plate', 'conditions':'are_req_robot_ops_completed'},
            {'source':'load_stir_plate','dest':'stir', 'conditions':'are_req_robot_ops_completed'},
            {'source':'stir','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
        ]

    ''' states callbacks '''

    def request_place_lot(self):      
        # place the 8-well rack
        batch_1 = self.lot.batches[0]
        params_dict = {}
        params_dict["perform_6p_calib"] = True
        robot_op_1 = DropBatchOpDescriptor.from_args(name='LoadEightWRackYumiStation', target_robot="MobileRobot",
                                                       params=params_dict, target_batch=batch_1)
        # place the pxrd rack
        batch_2 = self.lot.batches[1]
        params_dict["perform_6p_calib"] = False
        robot_op_2 = DropBatchOpDescriptor.from_args(name='LoadPXRDRackYumiStation', target_robot="MobileRobot",
                                                       params=params_dict, target_batch=batch_2)
        
        self.request_robot_ops([robot_op_1, robot_op_2])

    def request_load_stir_plate(self):
        batch_1 = self.lot.batches[0]
        robot_op = RobotTaskOpDescriptor.from_args(name='loadIKAPlate', target_robot="FixedRobot",
                                                   target_batch=batch_1)
        
        self.request_robot_ops([robot_op])

    def request_stir_op(self):
        batch_1 = self.lot.batches[0]
        current_op = self.generate_operation("stir_samples", target_batch=batch_1)
        self.request_station_op(current_op)
