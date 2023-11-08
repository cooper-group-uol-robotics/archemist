from bson.objectid import ObjectId
from .model import PXRDStationModel, PXRDJobStatus, PXRDAnalysisResultModel
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station
from archemist.core.state.batch import Batch
from archemist.core.state.station_op import StationOp, StationBatchOp
from archemist.core.models.station_op_model import StationOpModel, StationBatchOpModel
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.util.enums import OpOutcome
from typing import List, Dict, Union, Type
from datetime import datetime

''' ==== Station Description ==== '''
class PXRDStation(Station):
    def __init__(self, station_model: Union[PXRDStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = PXRDStationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)

    @property
    def job_status(self) -> PXRDJobStatus:
        return self._model_proxy.job_status

    @job_status.setter
    def job_status(self, new_status: PXRDJobStatus):
        self._model_proxy.job_status = new_status

    @property
    def door_closed(self) -> bool:
        return self._model_proxy.door_closed
    
    @door_closed.setter
    def door_closed(self, closed: bool):
        self._model_proxy.door_closed = closed

    def update_assigned_op(self):
        super().update_assigned_op()
        current_op = self.assigned_op
        if isinstance(current_op, PXRDAnalysisOp):
            self.job_status = PXRDJobStatus.RUNNING_JOB

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        current_op = self.assigned_op
        if isinstance(current_op, PXRDOpenDoorOp):
            self.door_closed = False
        elif isinstance(current_op, PXRDCloseDoorOp):
            self.door_closed = True
        elif isinstance(current_op, PXRDAnalysisOp):
            self.job_status = PXRDJobStatus.JOB_COMPLETE
        super().complete_assigned_op(outcome, results)

''' ==== Station Operation Descriptors ==== '''

class PXRDOpenDoorOp(StationOp):
    def __init__(self, station_op_model: Union[StationOpModel,ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=PXRDStation.__name__)
        model.save()
        return cls(model)


class PXRDCloseDoorOp(StationOp):
    def __init__(self, station_op_model: Union[StationOpModel,ModelProxy]) -> None:
        super().__init__(station_op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=PXRDStation.__name__)
        model.save()
        return cls(model)

class PXRDAnalysisOp(StationBatchOp):
    def __init__(self, op_model: StationBatchOpModel):
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_batch: Batch):
        model = StationBatchOpModel()
        model.target_batch = target_batch.model
        cls._set_model_common_fields(model, associated_station=PXRDStation.__name__)
        model.save()
        return cls(model)
    
class PXRDAnalysisResult(StationOpResult):
    def __init__(self, result_model: Union[PXRDAnalysisResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls, origin_op: ObjectId, result_filename: str):
        model = PXRDAnalysisResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.result_filename = result_filename
        model.save()
        return cls(model)

    @property
    def result_filename(self) -> str:
        return self._model_proxy.result_filename