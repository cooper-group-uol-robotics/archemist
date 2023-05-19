
from datetime import datetime, timedelta
from transitions import State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaLBRMaintenanceTask
from .state import WaitingOpDescriptor
from archemist.core.util.enums import TimeUnit
from archemist.core.state.station_process import StationProcess, StationProcessData

class WaitingStationProcess(StationProcess):
    
    def __init__(self, station: Station, process_data: StationProcessData, **kwargs):
        ''' States '''
        states = [ State(name='init_state'),
            State(name='prep_state', on_enter='initialise_process_data'),
            State(name='waiting_process', on_enter=['request_process_operation']),
            State(name='load_batch', on_enter=['request_load_batch']),
            State(name='added_batch_update', on_enter=['update_loaded_batch']),
            State(name='unload_batch', on_enter=['request_unload_batch']),
            State(name='removed_batch_update', on_enter=['update_unloaded_batch']),
            State(name='final_state', on_enter=['finalize_batch_processing'])]

        ''' Transitions '''
        transitions = [
            {'source':'init_state', 'dest': 'prep_state'},
            {'source':'prep_state', 'dest': 'load_batch'},
            {'source':'load_batch','dest':'added_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'added_batch_update','dest':'load_batch', 'unless':'are_all_batches_loaded'},
            {'source':'added_batch_update','dest':'waiting_process', 'conditions':'are_all_batches_loaded'},
            {'source':'waiting_process','dest':'unload_batch', 'conditions':'is_timer_done', 'before':'process_batches'},
            {'source':'unload_batch','dest':'removed_batch_update', 'conditions':'are_req_robot_ops_completed'},
            {'source':'removed_batch_update','dest':'unload_batch', 'unless':'are_all_batches_unloaded'},
            {'source':'removed_batch_update','dest':'final_state', 'conditions':'are_all_batches_unloaded'},
        ]
        super().__init__(station, process_data, states, transitions)

        ''' states callbacks '''

    def initialise_process_data(self):
        self._process_data.status['batch_index'] = 0
        self._process_data.status['timer_expiry_datetime'] = ""
        self._process_data.status['stored_op'] = None

    def request_load_batch(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='LoadWaitingStation',params=[True,batch_index+1], 
                                            type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index].id
        self.request_robot_op(robot_op,current_batch_id)

    def update_loaded_batch(self):
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_station(batch_index)
        self._process_data.status['batch_index'] += 1

    def request_unload_batch(self):
        batch_index = self._process_data.status['batch_index']
        robot_op = KukaLBRTask.from_args(name='UnloadWaitingStation',params=[False,batch_index],
                                type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
        current_batch_id = self._process_data.batches[batch_index - 1].id
        self.request_robot_op(robot_op,current_batch_id)

    def update_unloaded_batch(self):
        self._process_data.status['batch_index'] -= 1
        batch_index = self._process_data.status['batch_index']
        self._update_batch_loc_to_robot(batch_index)

    def request_process_operation(self):
        current_op = self._process_data.batches[-1].recipe.get_current_task_op()
        current_op.add_start_timestamp()
        expiry_date = datetime.now()
        if current_op.time_unit == TimeUnit.SECONDS:
            expiry_date += timedelta(seconds=current_op.duration)
        elif current_op.time_unit == TimeUnit.MINUTES:
            expiry_date += timedelta(minutes=current_op.duration)
        elif current_op.time_unit == TimeUnit.HOURS:
            expiry_date += timedelta(hours=current_op.duration)
        self._process_data.status['timer_expiry_datetime'] = expiry_date.isoformat()
        self._process_data.status['stored_op'] = current_op.model


    ''' transition callbacks '''

    def is_timer_done(self) -> bool:
        expiry_datetime = datetime.fromisoformat(self._process_data.status['timer_expiry_datetime'])
        return datetime.now() > expiry_datetime
    
    def are_all_batches_loaded(self):
        return self._process_data.status['batch_index'] == len(self._process_data.batches)
    
    def are_all_batches_unloaded(self):
        return self._process_data.status['batch_index'] == 0
        
    def process_batches(self):
        op_model = self._process_data.status['stored_op']
        op = WaitingOpDescriptor(op_model)
        completed_op = self._complete_station_op(op)
        for batch in self._station.assigned_batches:
            for _ in range(0, batch.num_samples):
                    batch.add_station_op_to_current_sample(completed_op)
                    batch.process_current_sample()

    def finalize_batch_processing(self):
        for batch in self._process_data.batches:
            self._station.process_assinged_batch(batch)

    def _complete_station_op(self, op: WaitingOpDescriptor) -> WaitingOpDescriptor:
        op_uuid = str(op.uuid)
        self._station.assign_station_op(op)
        self._station.update_assigned_op()
        self._station.complete_assigned_station_op(True)
        return self._station.completed_station_ops[op_uuid]



