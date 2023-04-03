from typing import Dict
from transitions import Machine, State
from archemist.core.state.station import Station
from archemist.core.state.robot import RobotTaskType, RobotTaskOpDescriptor
from archemist.robots.kmriiwa_robot.state import KukaLBRTask
from archemist.core.persistence.object_factory import StationFactory
from archemist.core.processing.station_process_fsm import StationProcessFSM


class StationLoadingSm(StationProcessFSM):
    def __init__(self, station: Station, params_dict: Dict):
        super().__init__(station, params_dict)
        if "operation_complete" not in self._status.keys():
            self._status["operation_complete"] = False

        if "loaded_samples" not in self._status.keys():
            self._status["loaded_samples"] = 0

        self.batch_mode = params_dict["batch_mode"]
        self._batch_load_task = params_dict["batch_load_task"]
        self._batch_unload_task = params_dict["batch_unload_task"]
        self._sample_load_task = params_dict["sample_load_task"]
        self._sample_unload_task = params_dict["sample_unload_task"]

        """ States """
        states = [
            State(name="init_state"),
            State(name="load_sample", on_enter=["request_load_sample_job"]),
            State(name="added_sample_update", on_enter=["update_sample_addition"]),
            State(name="load_batch", on_enter=["request_load_batch"]),
            State(name="added_batch_update", on_enter=["update_batch_loc_to_station"]),
            State(name="station_process", on_enter=["request_operation"]),
            State(name="unload_sample", on_enter=["request_unload_sample_job"]),
            State(name="removed_sample_update", on_enter=["update_sample_removal"]),
            State(name="unload_batch", on_enter=["request_unload_batch_job"]),
            State(name="removed_batch_update", on_enter=["update_batch_removal"]),
            State(name="final_state", on_enter=["finalize_batch_processing"]),
        ]

        self.machine = Machine(self, states=states, initial="init_state")

        """ Transitions """
        transitions = [
            {
                "trigger": self._trigger_function,
                "source": "init_state",
                "dest": "load_batch",
                "conditions": "all_batches_assigned",
            },
            {
                "trigger": self._trigger_function,
                "source": "load_batch",
                "dest": "added_batch_update",
                "conditions": "is_station_job_ready",
            },
            {
                "trigger": self._trigger_function,
                "source": "added_batch_update",
                "dest": "load_sample",
                "conditions": "is_station_job_ready",
            },
        ]
        if self.batch_mode:
            transitions += [
                {
                    "trigger": self._trigger_function,
                    "source": "load_sample",
                    "dest": "added_sample_update",
                    "conditions": "is_station_job_ready",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "added_sample_update",
                    "dest": "load_sample",
                    "conditions": "is_station_job_ready",
                    "unless": "are_all_samples_loaded",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "added_sample_update",
                    "dest": "station_process",
                    "conditions": ["are_all_samples_loaded", "is_station_job_ready"],
                },
                {
                    "trigger": self._trigger_function,
                    "source": "station_process",
                    "dest": "unload_sample",
                    "conditions": "is_station_job_ready",
                    "before": "process_batch",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "unload_sample",
                    "dest": "removed_sample_update",
                    "conditions": "is_station_job_ready",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "removed_sample_update",
                    "dest": "unload_sample",
                    "conditions": "is_station_job_ready",
                    "unless": "are_all_samples_unloaded",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "removed_sample_update",
                    "dest": "unload_batch",
                    "conditions": ["are_all_samples_unloaded", "is_station_job_ready"],
                },
            ]
        else:
            transitions += [
                {
                    "trigger": self._trigger_function,
                    "source": "load_sample",
                    "dest": "station_process",
                    "conditions": "is_station_job_ready",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "station_process",
                    "dest": "unload_sample",
                    "conditions": "is_station_job_ready",
                    "before": "process_sample",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "unload_sample",
                    "dest": "added_sample_update",
                    "conditions": "is_station_job_ready",
                    "unless": "are_all_samples_loaded",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "added_sample_update",
                    "dest": "load_sample",
                    "conditions": "is_station_job_ready",
                    "unless": "are_all_samples_loaded",
                },
                {
                    "trigger": self._trigger_function,
                    "source": "added_sample_update",
                    "dest": "unload_batch",
                    "conditions": ["are_all_samples_loaded", "is_station_job_ready"],
                    "before": "reset_samples",
                },
            ]

        transitions += [
            {
                "trigger": self._trigger_function,
                "source": "unload_batch",
                "dest": "removed_batch_update",
                "conditions": "is_station_job_ready",
            },
            {
                "trigger": self._trigger_function,
                "source": "removed_batch_update",
                "dest": "load_batch",
                "unless": "are_all_batches_processed",
                "conditions": "is_station_job_ready",
            },
            {
                "trigger": self._trigger_function,
                "source": "removed_batch_update",
                "dest": "final_state",
                "conditions": ["is_station_job_ready", "are_all_batches_processed"],
                "before": "reset_batches",
            },
        ]

        self.init_state_machine(states=states, transitions=transitions)

    def is_station_job_ready(self):
        return (
            not self._station.has_assigned_station_op()
            and not self._station.has_requested_robot_op()
        )

    def are_all_samples_loaded(self):
        return (
            self._status["loaded_samples"]
            == self._station.assigned_batches[self._status["batch_index"]].num_samples
        )

    def are_all_samples_unloaded(self):
        return self._status["loaded_samples"] == 0

    def are_all_batches_processed(self):
        return self._status["batches_count"] == self._station.batch_capacity

    def update_batch_removal(self):
        self.update_batch_loc_to_robot()
        self._status["batches_count"] += 1
        if self._status["batches_count"] == self._station.batch_capacity:
            self._status["batch_index"] = 0
        else:
            self._status["batch_index"] += 1

    def reset_samples(self):
        self._status["loaded_samples"] = 0

    def reset_batches(self):
        self._status["batch_index"] = 0
        self._status["batches_count"] = 0

    def request_load_sample_job(self):
        sample_index = self._status["loaded_samples"]
        robot_job = RobotTaskOpDescriptor.from_args(
            name=self._sample_load_task, params=[sample_index]
        )
        current_batch_id = self._station.assigned_batches[
            self._status["batch_index"]
        ].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_load_batch(self):
        robot_job = KukaLBRTask.from_args(
            name=self._batch_load_task,
            params=[False, self._status["batch_index"]],
            type=RobotTaskType.UNLOAD_FROM_ROBOT,
            location=self._station.location,
        )
        current_batch_id = self._station.assigned_batches[
            self._status["batch_index"]
        ].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_unload_batch_job(self):
        robot_job = KukaLBRTask.from_args(
            name=self._batch_unload_task,
            params=[False, self._status["batch_index"]],
            type=RobotTaskType.UNLOAD_FROM_ROBOT,
            location=self._station.location,
        )
        current_batch_id = self._station.assigned_batches[
            self._status["batch_index"]
        ].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def request_operation(self):
        current_op = self._station.assigned_batches[
            self._status["batch_index"]
        ].recipe.get_current_task_op()
        self._station.assign_station_op(current_op)

    def request_unload_sample_job(self):
        sample_index = self._status["loaded_samples"]
        robot_job = RobotTaskOpDescriptor.from_args(
            name=self._sample_unload_task, params=[sample_index]
        )
        current_batch_id = self._station.assigned_batches[
            self._status["batch_index"]
        ].id
        self._station.request_robot_op(robot_job, current_batch_id)

    def update_sample_addition(self):
        self._status["loaded_samples"] += 1

    def update_sample_removal(self):
        self._status["loaded_samples"] -= 1

    def process_batch(self):
        last_operation_op = self._station.station_op_history[-1]
        for _ in range(
            0, self._station.assigned_batches[self._status["batch_index"]].num_samples
        ):
            self._station.assigned_batches[
                self._status["batch_index"]
            ].add_station_op_to_current_sample(last_operation_op)
            self._station.assigned_batches[
                self._status["batch_index"]
            ].process_current_sample()
        self._status["operation_complete"] = True

    def process_sample(self):
        last_operation_op = self._station.station_op_history[-1]
        self._station.assigned_batches[
            self._status["batch_index"]
        ].add_station_op_to_current_sample(last_operation_op)
        self._station.assigned_batches[
            self._status["batch_index"]
        ].process_current_sample()

    def finalize_batch_processing(self):
        self._station.process_assigned_batches()
        self._status["operation_complete"] = False
        self._status["loaded_samples"] = 0
        self._status["batch_index"] = 0
        self._status["batches_count"] = 0
        self.to_init_state()
