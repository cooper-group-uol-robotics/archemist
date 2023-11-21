from typing import Union
from transitions import State
from .state import WatersLCMSStation, LCMSInsertRackOp, LCMSEjectRackOp, LCMSAnalysisOp
from archemist.core.state.lot import Lot
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.robot_op import RobotTaskOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import List, Dict, Any

class APCLCMSAnalysisProcess(StationProcess):
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [State(name='init_state'), 
            State(name='prep_state'),
            State(name='place_vial', on_enter=['request_vial_placement']),
            State(name='insert_rack', on_enter=['request_rack_insertion']),
            State(name='run_analysis', on_enter=['request_analysis']),
            State(name='eject_rack', on_enter=['request_rack_ejection']),
            State(name='dispose_vial', on_enter=['request_vial_disposal']),
            State(name='final_state')]            

        ''' Transitions '''
        self.TRANSITIONS = [
            { 'source':'init_state', 'dest': 'prep_state'},
            { 'source':'prep_state','dest':'place_vial'},
            { 'source':'place_vial','dest':'insert_rack', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'insert_rack','dest':'run_analysis', 'conditions':'are_req_station_ops_completed'},
            { 'source':'run_analysis','dest':'eject_rack', 'conditions':'are_req_station_ops_completed'},
            { 'source':'eject_rack','dest':'dispose_vial', 'conditions':'are_req_station_ops_completed'},
            { 'source':'dispose_vial','dest':'final_state', 'conditions':'are_req_robot_ops_completed'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  operations: List[Dict[str, Any]] = None,
                  is_subprocess: bool=False,
                  skip_robot_ops: bool=False,
                  skip_station_ops: bool=False,
                  skip_ext_procs: bool=False
                  ):
        model = StationProcessModel()
        cls._set_model_common_fields(model,
                                     WatersLCMSStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.save()
        return cls(model)
        
    ''' states callbacks '''
    def request_vial_placement(self):
        robot_task = RobotTaskOp.from_args(name='PlaceLCMSVial',
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_vial_disposal(self):
        robot_task = RobotTaskOp.from_args(name='DisposeLCMSVial',
                                           target_robot="KMRIIWARobot")
        self.request_robot_ops([robot_task])

    def request_analysis(self):
        batch = self.lot.batches[0]
        analysis_op = LCMSAnalysisOp.from_args(target_batch=batch)
        self.request_station_op(analysis_op)

    def request_rack_insertion(self):
        insert_op = LCMSInsertRackOp.from_args()
        self.request_station_op(insert_op)

    def request_rack_ejection(self):
        eject_op = LCMSEjectRackOp.from_args()
        self.request_station_op(eject_op)