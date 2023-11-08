from .model import (WatersLCMSStationModel,
                    LCMSAnalysisStatus,
                    LCMSAutoLoaderStatus,
                    LCMSBayOccupiedOpModel,
                    LCMSBayFreedOpModel,
                    LCMSInsertBatchOpModel,
                    LCMSEjectBatchOpModel,
                    LCMSAnalysisResultModel)
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station
from archemist.core.state.batch import Batch
from archemist.core.models.station_op_model import StationOpModel
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.station_op import StationOp, StationBatchOp
from typing import Dict, Union, List, Optional
from archemist.core.util.enums import OpOutcome
from bson.objectid import ObjectId

''' ==== Station Description ==== '''
class WatersLCMSStation(Station):
    def __init__(self, station_model: Union[WatersLCMSStationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = WatersLCMSStationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)

    @property
    def analysis_status(self) -> LCMSAnalysisStatus:
        return self._model_proxy.analysis_status

    @analysis_status.setter
    def analysis_status(self, new_status: LCMSAnalysisStatus):
        self._model_proxy.analysis_status = new_status

    @property
    def auto_loader_status(self) -> LCMSAutoLoaderStatus:
        return self._model_proxy.auto_loader_status

    @auto_loader_status.setter
    def auto_loader_status(self, new_status: LCMSAutoLoaderStatus):
        self._model_proxy.auto_loader_status = new_status

    def update_assigned_op(self):
        super().update_assigned_op()
        current_op = self.assigned_op
        if isinstance(current_op, LCMSAnalysisOp):
            self.analysis_status = LCMSAnalysisStatus.RUNNING_ANALYSIS

    def complete_assigned_op(self, outcome: OpOutcome, results: Optional[List[LCMSAnalysisResultModel]]):
        current_op = self.assigned_op
        if isinstance(current_op, LCMSBayOccupiedOp):
            self.auto_loader_status = LCMSAutoLoaderStatus.BAY_OCCUPIED
        elif isinstance(current_op, LCMSBayFreedOp):
            self.auto_loader_status = LCMSAutoLoaderStatus.BAY_FREE
        elif isinstance(current_op, LCMSInsertBatchOp):
            self.auto_loader_status = LCMSAutoLoaderStatus.BAY_UNAVAILABLE
        elif isinstance(current_op, LCMSEjectBatchOp):
            self.auto_loader_status = LCMSAutoLoaderStatus.BAY_OCCUPIED
        elif isinstance(current_op, LCMSAnalysisOp):
            self.analysis_status = LCMSAnalysisStatus.ANALYSIS_COMPLETE
        super().complete_assigned_op(outcome, results)


''' ==== Station Operation Descriptors ==== '''
class LCMSBayOccupiedOp(StationOp):
    def __init__(self, op_model: Union[LCMSBayOccupiedOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, bay_index: int):
        model = LCMSBayOccupiedOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.bay_index = bay_index
        model.save()
        return cls(model)

    @property
    def bay_index(self) -> int:
        return self._model_proxy.bay_index

class LCMSBayFreedOp(StationOp):
    def __init__(self, op_model: Union[LCMSBayFreedOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, bay_index: int):
        model = LCMSBayFreedOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.bay_index = bay_index
        model.save()
        return cls(model)

    @property
    def bay_index(self) -> int:
        return self._model_proxy.bay_index

class LCMSInsertBatchOp(StationBatchOp):
    def __init__(self, op_model: Union[LCMSInsertBatchOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_batch: Batch, bay_index: int):
        model = LCMSInsertBatchOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.target_batch = target_batch.model
        model.bay_index = bay_index
        model.save()
        return cls(model)

    @property
    def bay_index(self) -> int:
        return self._model_proxy.bay_index

class LCMSEjectBatchOp(StationBatchOp):
    def __init__(self, op_model: Union[LCMSEjectBatchOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_batch: Batch, bay_index: int):
        model = LCMSEjectBatchOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.target_batch = target_batch.model
        model.bay_index = bay_index
        model.save()
        return cls(model)

    @property
    def bay_index(self) -> int:
        return self._model_proxy.bay_index

class LCMSAnalysisOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.save()
        return cls(model)

class LCMSAnalysisResult(StationOpResult):
    def __init__(self, result_model: Union[LCMSAnalysisResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls,
                  origin_op: ObjectId,
                  result_filename: str):
        model = LCMSAnalysisResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.result_filename = result_filename
        model.save()
        return cls(model)

    @property
    def result_filename(self) -> str:
        return self._model_proxy.result_filename