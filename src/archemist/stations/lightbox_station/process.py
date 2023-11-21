from typing import Union
from transitions import State
from archemist.core.state.lot import Lot
from .state import LightBoxStation
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.robot_op import RobotTaskOp, RobotWaitOp
from archemist.core.state.station_process import StationProcess, StationProcessModel
from typing import List, Dict, Any

class LBSampleAnalysisProcess(StationProcess):
    
    def __init__(self, process_model: Union[StationProcessModel, ModelProxy]) -> None:
        super().__init__(process_model)
        ''' States '''
        self.STATES = [State(name='init_state'), 
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='load_sample', on_enter=['request_load_sample_job']),
            State(name='unload_sample', on_enter=['request_unload_sample_job']),
            State(name='station_process', on_enter=['request_process_data_job']),
            State(name='update_batch_index', on_enter=['request_batch_index_update']),
            State(name='update_sample_index', on_enter=['request_sample_index_update']),
            State(name='final_state')]            

        ''' Transitions '''
        self.TRANSITIONS = [
            { 'source':'init_state', 'dest': 'prep_state'},
            { 'source':'prep_state','dest':'load_sample'},
            { 'source':'load_sample','dest':'station_process', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'station_process','dest':'unload_sample', 'conditions':'are_req_station_ops_completed'},
            { 'source':'unload_sample','dest':'update_sample_index', 'conditions':'are_req_robot_ops_completed'},
            { 'source':'update_sample_index','dest':'load_sample', 'unless':'are_all_samples_loaded'},
            { 'source':'update_sample_index','dest':'update_batch_index', 'conditions':'are_all_samples_loaded'},
            { 'source':'update_batch_index','dest':'load_sample', 'unless':'are_all_batches_processed'},
            { 'source':'update_batch_index','dest':'final_state', 'conditions':'are_all_batches_processed'},
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
                                     LightBoxStation.__name__,
                                     lot,
                                     operations,
                                     is_subprocess,
                                     skip_robot_ops,
                                     skip_station_ops,
                                     skip_ext_procs)
        model.save()
        return cls(model)
        
    ''' states callbacks '''

    def initialise_process_data(self):
        self.data['batch_index'] = 0
        self.data['sample_index'] = 0

    def request_load_sample_job(self):
        params_dict = {}
        params_dict["sample_index"] = self.data['sample_index'] + 1
        params_dict["batch_index"] = self.data['batch_index'] + 1
        params_dict["perform_6p_calib"] = False
        params_dict["allow_auto_func"] = False
        target_batch = self.lot.batches[self.data['batch_index']]
        robot_task = RobotTaskOp.from_args(name='PresentVial',
                                                     target_robot="KMRIIWARobot",
                                                     params=params_dict,
                                                     target_batch=target_batch)
        robot_wait_task = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, robot_wait_task])

    def request_unload_sample_job(self):
        params_dict = {}
        params_dict["sample_index"] = self.data['sample_index'] + 1
        params_dict["batch_index"] = self.data['batch_index'] + 1
        params_dict["perform_6p_calib"] = False
        params_dict["allow_auto_func"] = False
        target_batch = self.lot.batches[self.data['batch_index']]
        robot_task = RobotTaskOp.from_args(name='ReturnVial',
                                                     target_robot="KMRIIWARobot",
                                                     params=params_dict,
                                                     target_batch=target_batch)
        robot_wait_task = RobotWaitOp.from_args("KMRIIWARobot", 3)
        self.request_robot_ops([robot_task, robot_wait_task])

    def request_process_data_job(self):
        sample_index = self.data['sample_index']
        batch_index = self.data['batch_index']
        sample = self.lot.batches[batch_index].samples[sample_index]
        
        current_op = self.generate_operation("analyse_op", target_sample=sample)
        self.request_station_op(current_op)

    def request_sample_index_update(self):
        self.data['sample_index'] += 1

    def request_batch_index_update(self):
        self.data['batch_index'] += 1
        self.data['sample_index'] = 0

    ''' transition callbacks '''

    def are_all_samples_loaded(self):
        sample_index = self.data['sample_index']
        batch_index = self.data['batch_index']
        return sample_index == self.lot.batches[batch_index].num_samples

    def are_all_batches_processed(self):
        return self.data['batch_index'] == self.lot.num_batches


