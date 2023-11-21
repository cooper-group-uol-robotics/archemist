from typing import Dict
import time
from transitions import State
from archemist.core.state.station import Station
from .state import SynthesisStation, OptimaxTempStirringOpDescriptor, OptimaxTempOpDescriptor, OptimaxStirringOpDescriptor, OptimaxSamplingOpDescriptor, LcmsOpDescriptor, ParacetamolSynthesisOpDescriptor
from archemist.core.state.robot import RobotTaskType
from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaNAVTask, KukaLBRMaintenanceTask
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.util import Location


class SynthesisStationSm(StationProcessFSM):

    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if 'loaded_samples' not in self._status.keys():
            self._status['loaded_samples'] = 0

        ''' States '''
        states = [State(name='init_state'),
                  State(name='optimax_heating_process',
                        on_enter=['request_heating_process']),
                  State(name='optimax_sampling_process',
                        on_enter=['request_sampling_process']),
                  State(name='optimax_cooling_process',
                        on_enter=['request_cooling_process']),
                  State(name='disable_auto_functions', on_enter=[
                        'request_disable_auto_functions']),
                  State(name='enable_auto_functions', on_enter=[
                        'request_enable_auto_functions']),
                  State(name='unload_batch', on_enter=[
                        'request_unload_batch']),
                  State(name='load_batch', on_enter=['request_load_batch']),
                  State(name='navigate_to_optimax', on_enter=[
                        'request_robot_navigation_to_optimax']),
                  State(name='navigate_to_LCMS', on_enter=[
                        'request_robot_navigation_to_LCMS']),
                  State(name='LCMS_process', on_enter=[
                        'request_LCMS_process']),
                  State(name='Drain_process', on_enter=[
                        'request_Drain_process']),
                  State(name='paracetamol_synthesis', on_enter=['request_paracetamol_synthesis']),
                #   State(name='added_batch_update',
                #         on_enter=['update_loaded_batch']),
                #   State(name='removed_batch_update',
                #         on_enter=['update_unloaded_batch']),
                  State(name='final_state', on_enter='finalize_batch_processing')]

        ''' Transitions '''
        # transitions = [
        #     {'trigger': self._trigger_function, 'source': 'init_state',
        #         'dest': 'optimax_heating_process', 'conditions': 'all_batches_assigned'},
        #     {'trigger': self._trigger_function, 'source': 'optimax_heating_process',
        #         'dest': 'optimax_sampling_process', 'conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'optimax_sampling_process',
        #         'dest': 'disable_auto_functions', 'conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'disable_auto_functions',
        #         'dest': 'navigate_to_optimax', 'conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'navigate_to_optimax',
        #         'dest': 'unload_batch', 'conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'unload_batch',
        #         'dest': 'navigate_to_LCMS', 'conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'navigate_to_LCMS',
        #         'dest': 'load_batch', 'conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'load_batch',
        #         'dest': 'enable_auto_functions', 'conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'enable_auto_functions',
        #         'dest': 'LCMS_process', 'conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'LCMS_process',
        #         'dest': 'optimax_heating_process', 'unless': 'is_LCMS_result_positive','conditions': 'is_station_job_ready'},
        #     {'trigger': self._trigger_function, 'source': 'LCMS_process',
        #         'dest': 'optimax_cooling_process', 'conditions': ['is_station_job_ready','is_LCMS_result_positive']},
        #     {'trigger': self._trigger_function, 'source': 'optimax_cooling_process',
        #         'dest': 'final_state', 'conditions': 'is_station_job_ready', 'before':'process_sample'},
        # ]

        transitions = [
            {'trigger': self._trigger_function, 'source': 'init_state',
                'dest': 'paracetamol_synthesis', 'conditions': 'all_batches_assigned'},
            # {'trigger': self._trigger_function, 'source': 'optimax_heating_process',
            #     'dest': 'optimax_sampling_process', 'conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'optimax_sampling_process',
            #     'dest': 'disable_auto_functions', 'conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'disable_auto_functions',
            #     'dest': 'navigate_to_optimax', 'conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'navigate_to_optimax',
            #     'dest': 'unload_batch', 'conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'unload_batch',
            #     'dest': 'navigate_to_LCMS', 'conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'navigate_to_LCMS',
            #     'dest': 'load_batch', 'conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'load_batch',
            #     'dest': 'enable_auto_functions', 'conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'enable_auto_functions',
            #     'dest': 'LCMS_process', 'conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'LCMS_process',
            #     'dest': 'optimax_heating_process', 'unless': 'is_LCMS_result_positive','conditions': 'is_station_job_ready'},
            # {'trigger': self._trigger_function, 'source': 'LCMS_process',
            #     'dest': 'optimax_cooling_process', 'conditions': ['is_station_job_ready','is_LCMS_result_positive']},
            {'trigger': self._trigger_function, 'source': 'paracetamol_synthesis',
                'dest': 'final_state', 'conditions': 'is_station_job_ready', 'before':'process_sample'},
        ]

        

        self.init_state_machine(states=states, transitions=transitions)


#################################################

# temporary station process to be modified 
    def request_paracetamol_synthesis(self):
        self._station.assign_station_op(ParacetamolSynthesisOpDescriptor.from_args())



##################################################

    def request_heating_process(self):
        current_op = self._station.assigned_batches[0].recipe.get_current_task_op(
        )
        self.temperature = current_op.temperature
        self.temp_duration = current_op.temp_duration
        self.stir_speed = current_op.stir_speed
        self.stir_duration = current_op.stir_duration
        self.dilution = 5
        
        self._station.assign_station_op(OptimaxTempStirringOpDescriptor.from_args(temperature=current_op.temperature, temp_duration= current_op.temp_duration, stir_speed=self.stir_speed, stir_duration=self.stir_duration))

    def request_sampling_process(self):       
        # self._station.assign_station_op(
        #     OptimaxSamplingOpDescriptor.from_args(dilution = self.dilution))
        pass

    def request_LCMS_process(self):
        # self._station.assign_station_op(LcmsOpDescriptor.from_args())
        pass

    def request_cooling_process(self):
        _temperature = 10
        _temp_duration = 5
        self._station.assign_station_op(OptimaxTempOpDescriptor.from_args(temperature=_temperature, temp_duration= _temp_duration))

    # robot process

    def request_unload_batch(self):
        # robot_job = KukaLBRTask.from_args(name='UnloadOptimax',params=[False,self._status['batch_index']+1],
        #                         type=RobotTaskType.MANIPULATION, location=self._station.location)
        # current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        # self._station.request_robot_op(robot_job,current_batch_id)
        pass


    def request_load_batch(self):
        # robot_job = KukaLBRTask.from_args(name='LoadLCMS',params=[True,self._status['batch_index']+1],
        #                                      type=RobotTaskType.UNLOAD_FROM_ROBOT, location=self._station.location)
        # current_batch_id = self._station.assigned_batches[self._status['batch_index']].id
        # self._station.request_robot_op(robot_job,current_batch_id)
        pass

    def request_disable_auto_functions(self):
        # self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('DiableAutoFunctions',[False]))
        pass
        

    def request_enable_auto_functions(self):
        # self._station.request_robot_op(KukaLBRMaintenanceTask.from_args('EnableAutoFunctions',[False]))
        pass
        

    def request_robot_navigation_to_optimax(self):
        # self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO change the node id and graph id for Optimax
        pass
        

    def request_robot_navigation_to_LCMS(self):
        # self._station.request_robot_op(KukaNAVTask.from_args(Location(26,1,''), False)) #TODO change the node id and graph id for LCMS
        pass
    

    def is_LCMS_result_positive(self):
        return True


    # def request_batch_index_update(self):
    #     self._status['batch_index'] += 1
    #     self._status['batches_count'] += 1

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['batch_index'] = 0
        self._status['batches_count'] = 0
        self._status['loaded_samples'] = 0
        self.to_init_state()

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[self._status['batch_index']].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[self._status['batch_index']].process_current_sample()


    def timer(self, seconds):
        start_time = time.time()
        end_time = start_time + seconds
        
        while time.time() < end_time:
            remaining_time = int(end_time - time.time())
            print(f"Time remaining: {remaining_time} seconds", end="\r", flush=True)
            time.sleep(1)
        
