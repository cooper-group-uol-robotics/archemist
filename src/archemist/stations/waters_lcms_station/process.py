from typing import Union
from transitions import State
from .state import WatersLCMSStation, LCMSInsertRackOp, LCMSEjectRackOp, LCMSSampleAnalysisOp, LCMSPrepAnalysisOp
from archemist.core.state.lot import Lot
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.robot_op import RobotTaskOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import List, Dict, Any
from archemist.core.util import Location
class APCLCMSAnalysisProcess(StationProcess):
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)

        ''' States '''
        self.STATES = [State(name='init_state'), 
            State(name='prep_state', on_enter=['request_prep_analysis']),
            State(name='collect_vial', on_enter=['request_collect_vial']),
            State(name='place_vial', on_enter=['request_vial_placement']),
            State(name='insert_rack', on_enter=['request_rack_insertion']),
            State(name='run_analysis', on_enter=['request_analysis']),
            State(name='eject_rack', on_enter=['request_rack_ejection']),
            State(name='dispose_vial', on_enter=['request_vial_disposal']),
            State(name='increment_sample_index', on_enter=['request_increment_sample_index']),
            State(name='final_state')]            

        ''' Transitions '''
        self.TRANSITIONS = [
            { 'source':'init_state', 'dest': 'prep_state'},
            { 'source':'prep_state','dest':'collect_vial', 'conditions':'are_req_station_ops_completed'},
            { 'source':'collect_vial','dest':'place_vial', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'place_vial','dest':'insert_rack', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'insert_rack','dest':'run_analysis', 'conditions':'are_req_station_ops_completed'},
            { 'source':'run_analysis','dest':'eject_rack', 'conditions':'are_req_station_ops_completed'},
            { 'source':'eject_rack','dest':'dispose_vial', 'conditions':'are_req_station_ops_completed'},
            { 'source':'dispose_vial','dest':'increment_sample_index', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'increment_sample_index','dest':'final_state', 'conditions':'are_req_station_ops_completed'},
        ]

    @classmethod
    def from_args(cls, lot: Lot,
                  target_batch_index: int,
                  target_sample_index: int,
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
        model.data["target_batch_index"] = target_batch_index
        model.data["target_sample_index"] = target_sample_index
        model.save()
        return cls(model)
        
    ''' states callbacks '''
    def request_prep_analysis(self):
        prep_op = LCMSPrepAnalysisOp.from_args()
        self.request_station_op(prep_op)
        
    def request_collect_vial(self):
        robot_task = RobotTaskOp.from_args(
            name = "collectSample",
            target_robot = "KMRIIWARobot",
            task_type = 2,
            lbr_program_name = "collectSample"
            )
        self.request_robot_ops([robot_task])

    def request_vial_placement(self):
        location_dict = {"coordinates": [35, 8], "descriptor": "LCMS station"}
        target_loc = Location.from_dict(location_dict)
        robot_task = RobotTaskOp.from_args(
            name="loadLCMS",
            target_robot="KMRIIWARobot",
            target_location=target_loc,
            params={},  
            lbr_program_name="loadLCMS",
            lbr_program_params=[],
            fine_localization=True,
            task_type=2
        )
        self.request_robot_ops([robot_task])

    def request_vial_disposal(self):
        lcms_station: WatersLCMSStation = self.get_assigned_station()
        location_dict = {"coordinates": [35, 8], "descriptor": "LCMS station"}
        target_loc = Location.from_dict(location_dict)
        robot_task = RobotTaskOp.from_args(
            name="unLoadLCMS",
            target_robot="KMRIIWARobot",
            target_location=target_loc,
            params={},  
            lbr_program_name="unLoadLCMS",
            lbr_program_params=[str(lcms_station.sample_index)],
            fine_localization=True,
            task_type=2
        )
        self.request_robot_ops([robot_task])

    def request_increment_sample_index(self):
        lcms_station: WatersLCMSStation = self.get_assigned_station()
        lcms_station.sample_index += 1

    def request_analysis(self):
        batch_index = self.data["target_batch_index"]
        sample_index = self.data["target_sample_index"]
        sample = self.lot.batches[batch_index].samples[sample_index]
        analysis_op = LCMSSampleAnalysisOp.from_args(target_sample=sample)
        self.request_station_op(analysis_op)

    def request_rack_insertion(self):
        insert_op = LCMSInsertRackOp.from_args()
        self.request_station_op(insert_op)

    def request_rack_ejection(self):
        eject_op = LCMSEjectRackOp.from_args()
        self.request_station_op(eject_op)