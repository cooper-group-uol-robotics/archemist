from __future__ import annotations
from typing import Union, Type, List
from datetime import datetime
from archemist.core.persistence.models_proxy import ModelProxy, ListProxy
from archemist.core.persistence.object_factory import OpResultFactory
from archemist.core.models.station_op_model import (StationOpDescriptorModel,
                                                    StationLotOpDescriptorModel,
                                                    StationBatchOpDescriptorModel,
                                                    StationSampleOpDescriptorModel)
from archemist.core.state.lot import Lot
from archemist.core.state.batch import Batch
from archemist.core.state.sample import Sample
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.util.enums import OpOutcome
from bson.objectid import ObjectId

class StationOpDescriptor:
    def __init__(self, station_op_model: Union[StationOpDescriptorModel, ModelProxy]) -> None:
        if isinstance(station_op_model, ModelProxy):
            self._model_proxy = station_op_model
        else:
            self._model_proxy = ModelProxy(station_op_model)

    @classmethod
    def _set_model_common_fields(cls, op_model: StationOpDescriptorModel, associated_station: str):
        op_model.associated_station = associated_station
        op_model._type = cls.__name__
        op_model._module = cls.__module__

    @classmethod
    def from_args(cls):
        model = StationOpDescriptorModel()
        cls._set_model_common_fields(model, associated_station="Station")
        model.save()
        return cls(model)

    @property
    def model(self):
        return self._model_proxy.model
    
    @property
    def object_id(self):
        return self._model_proxy.object_id
    
    @property
    def associated_station(self) -> str:
        return self._model_proxy.associated_station
    
    @property
    def requested_by(self) -> ObjectId:
        return self._model_proxy.requested_by

    @requested_by.setter
    def requested_by(self, station_id: ObjectId):
        self._model_proxy.requested_by = station_id

    @property
    def outcome(self) -> OpOutcome:
        return self._model_proxy.outcome

    @property
    def results(self) -> List[Type[StationOpResult]]:
        return ListProxy(self._model_proxy.results, OpResultFactory.create_from_model)

    @property
    def start_timestamp(self) -> datetime:
        return self._model_proxy.start_timestamp
    
    @start_timestamp.setter
    def start_timestamp(self, new_start_timestamp: datetime):
        self._model_proxy.start_timestamp = new_start_timestamp

    @property
    def end_timestamp(self) -> datetime:
        return self._model_proxy.end_timestamp
    
    @end_timestamp.setter
    def end_timestamp(self, new_end_timestamp: datetime):
        self._model_proxy.end_timestamp = new_end_timestamp

    def add_start_timestamp(self):
        self._model_proxy.start_timestamp = datetime.now()

    def complete_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        self._model_proxy.outcome = outcome
        if results:
            self.results.extend(results)
        self._model_proxy.end_timestamp = datetime.now()

    def __eq__(self, __value: object) -> bool:
        return self.object_id == __value.object_id
    
class StationLotOpDescriptor(StationOpDescriptor):
    def __init__(self, station_op_model: Union[StationLotOpDescriptorModel, ModelProxy]):
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls, target_lot: Lot):
        model = StationLotOpDescriptorModel()
        model.target_lot = target_lot.model
        cls._set_model_common_fields(model, associated_station="Station")
        model.save()
        return cls(model)
    
    @property
    def target_lot(self) -> Lot:
        return Lot(self._model_proxy.target_lot)

    def complete_op(self, outcome: OpOutcome, results: List[type[StationOpResult]]):
        super().complete_op(outcome, results)
        if not results:
            print("[Warning] Station lot op completed with no results")
        elif len(results) == 1:
            for batch in self.target_lot.batches:
                for sample in batch.samples:
                    sample.add_result_op(results[0])
        elif len(results) == self.target_lot.num_batches:
            for index, batch in enumerate(self.target_lot.batches):
                for sample in batch.samples:
                    sample.add_result_op(results[index])
        else:
            index = 0
            for batch in self.target_lot.batches:
                for sample in batch.samples:
                    sample.add_result_op(results[index])
                    index += 1  
    
class StationBatchOpDescriptor(StationOpDescriptor):
    def __init__(self, station_op_model: Union[StationBatchOpDescriptorModel, ModelProxy]):
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls, target_batch: Batch):
        model = StationBatchOpDescriptorModel()
        model.target_batch = target_batch.model
        cls._set_model_common_fields(model, associated_station="Station")
        model.save()
        return cls(model)
    
    @property
    def target_batch(self) -> Batch:
        return Batch(self._model_proxy.target_batch)
    
    def complete_op(self, outcome: OpOutcome, results: List[type[StationOpResult]]):
        super().complete_op(outcome, results)
        if not results:
            print("[Warning] Station lot op completed with no results")
        elif len(results) == 1:
            for sample in self.target_batch.samples:
                sample.add_result_op(results[0])
        else:
            index = 0
            for sample in self.target_batch.samples:
                sample.add_result_op(results[index])
                index += 1
    
class StationSampleOpDescriptor(StationOpDescriptor):
    def __init__(self, station_op_model: Union[StationSampleOpDescriptorModel, ModelProxy]):
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpDescriptorModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station="Station")
        model.save()
        return cls(model)
    
    @property
    def target_sample(self) -> Sample:
        return Sample(self._model_proxy.target_sample)
    
    def complete_op(self, outcome: OpOutcome, results: List[type[StationOpResult]]):
        super().complete_op(outcome, results)
        if not results:
            print("[Warning] Station lot op completed with no results")
        else:
            self.target_sample.add_result_op(results[0])