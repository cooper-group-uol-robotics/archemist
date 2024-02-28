from .model import (WatersLCMSStationModel,
                    LCMSAnalysisStatus,
                    LCMSAnalysisResultModel)
from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station
from archemist.core.state.sample import Sample
from archemist.core.models.station_op_model import StationOpModel, StationSampleOpModel
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.station_op import StationOp, StationSampleOp
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
    def sample_index(self) -> int:
        return self._model_proxy.sample_index
    
    @sample_index.setter
    def sample_index(self, index: int):
        self._model_proxy.sample_index = index

    @property
    def batch_inserted(self) -> bool:
        return self._model_proxy.batch_inserted

    @batch_inserted.setter
    def batch_inserted(self, is_inserted: bool):
        self._model_proxy.batch_inserted = is_inserted

    def update_assigned_op(self):
        super().update_assigned_op()
        current_op = self.assigned_op
        if isinstance(current_op, LCMSSampleAnalysisOp):
            self.analysis_status = LCMSAnalysisStatus.RUNNING_ANALYSIS

    def complete_assigned_op(self, outcome: OpOutcome, results: Optional[List[LCMSAnalysisResultModel]]):
        current_op = self.assigned_op
        if isinstance(current_op, LCMSInsertRackOp):
            self.batch_inserted = True
        elif isinstance(current_op, LCMSEjectRackOp):
            self.batch_inserted = False
        elif isinstance(current_op, LCMSSampleAnalysisOp):
            self.analysis_status = LCMSAnalysisStatus.ANALYSIS_COMPLETE
        super().complete_assigned_op(outcome, results)


''' ==== Station Operation Descriptors ==== '''
class LCMSInsertRackOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.save()
        return cls(model)

class LCMSEjectRackOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.save()
        return cls(model)

class LCMSPrepAnalysisOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.save()
        return cls(model)

class LCMSSampleAnalysisOp(StationSampleOp):
    def __init__(self, op_model: Union[StationSampleOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpModel()
        cls._set_model_common_fields(model, associated_station=WatersLCMSStation.__name__)
        model.target_sample = target_sample.model
        model.save()
        return cls(model)

class LCMSAnalysisResult(StationOpResult):
    def __init__(self, result_model: Union[LCMSAnalysisResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls,
                  origin_op: ObjectId,
                  concentration: float, 
                  result_filename: str):
        model = LCMSAnalysisResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.concentration = concentration
        model.result_filename = result_filename
        model.save()
        return cls(model)

    @property
    def concentration(self) -> float:
        return self._model_proxy.concentration

    @property
    def result_filename(self) -> str:
        return self._model_proxy.result_filename