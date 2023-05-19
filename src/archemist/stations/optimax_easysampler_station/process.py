from typing import Dict
from transitions import State
from archemist.core.state.station import Station
from .state import SynthesisStation, OptimaxTempStirringOpDescriptor, OptimaxTempOpDescriptor, OptimaxStirringOpDescriptor, OptimaxSamplingOpDescriptor, LcmsOpDescriptor
# from archemist.core.state.robot import RobotTaskType
# from archemist.robots.kmriiwa_robot.state import KukaLBRTask, KukaNAVTask, KukaLBRMaintenanceTask
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
                  State(name='added_batch_update',
                        on_enter=['update_loaded_batch']),
                  State(name='removed_batch_update',
                        on_enter=['update_unloaded_batch']),
                  State(name='final_state', on_enter='finalize_batch_processing')]

        ''' Transitions '''
        transitions = [
            {'trigger': self._trigger_function, 'source': 'init_state',
                'dest': 'optimax_heating_process', 'conditions': 'all_batches_assigned'},
            {'trigger': self._trigger_function, 'source': 'optimax_heating_process',
                'dest': 'optimax_sampling_process', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'optimax_sampling_process',
                'dest': 'disable_auto_functions', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'disable_auto_functions',
                'dest': 'navigate_to_optimax', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'navigate_to_optimax',
                'dest': 'unload_batch', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'unload_batch',
                'dest': 'added_batch_update', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'added_batch_update', 'dest': 'unload_batch',
                'unless': 'are_all_batches_unloaded', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'added_batch_update',
                'dest': 'navigate_to_LCMS', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'navigate_to_LCMS',
                'dest': 'load_batch', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'load_batch',
                'dest': 'removed_batch_update', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'removed_batch_update', 'dest': 'load_batch',
                'unless': 'are_all_batches_loaded', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'removed_batch_update',
                'dest': 'enable_auto_functions', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'enable_auto_functions',
                'dest': 'LCMS_process', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'LCMS_process',
                'dest': 'optimax_heating_process', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'LCMS_process',
                'dest': 'optimax_cooling_process', 'conditions': 'is_station_job_ready'},
            {'trigger': self._trigger_function, 'source': 'optimax_cooling_process',
                'dest': 'final_state', 'conditions': 'is_station_job_ready'},
        ]

        self.init_state_machine(states=states, transitions=transitions)

    # station process

    def request_heating_process(self):
        current_op = self._station.assigned_batches[0].recipe.get_current_task_op(
        )
        self.temperature = current_op.temperature
        self.temp_duration = current_op.temp_duration
        self.stir_speed = current_op.stir_speed
        self.stir_duration = current_op.stir_duration
        
        self._station.assign_station_op(OptimaxTempStirringOpDescriptor.from_args(
            temperature=self.temperature, temp_duration=self.temp_duration, stir_speed=self.stir_speed, stir_duration=self.stir_duration))

    def request_sampling_process(self):
        current_op = self._station.assigned_batches[0].recipe.get_current_task_op()
        self.stir_dilution = current_op.dilution
        self._station.assign_station_op(
            OptimaxSamplingOpDescriptor.from_args(dilution = self.stir_dilution))

    def request_LCMS_process(self):
        self._station.assign_station_op(LcmsOpDescriptor.from_args())

    def request_cooling_process(self):
        self._station.assign_station_op(OptimaxTempOpDescriptor.from_args())

    # robot process

    def request_unload_batch(self):
        # robot_job = KukaLBRTask.from_args(name='UnloadOptimax',params=[False,self._status['batch_index']+1],
        #                         type=RobotTaskType.LOAD_TO_ROBOT, location=self._station.location)
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

    def request_batch_index_update(self):
        self._status['batch_index'] += 1
        self._status['batches_count'] += 1

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status['batch_index'] = 0
        self._status['batches_count'] = 0
        self._status['loaded_samples'] = 0
        self.to_init_state()
