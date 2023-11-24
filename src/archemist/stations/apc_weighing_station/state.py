from typing import Dict, Union, List, Literal, Type
from archemist.core.persistence.models_proxy import ModelProxy
from .model import APCWeighResultModel, APCWeighingStationModel
from archemist.core.models.station_op_model import StationSampleOpModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOp, StationSampleOp, StationOpModel
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.sample import Sample
from archemist.core.util.enums import OpOutcome
from bson.objectid import ObjectId


''' ==== Station Description ==== '''
class APCWeighingStation(Station):
    def __init__(self, weighing_station_model: Union[APCWeighingStationModel, ModelProxy]) -> None:
        super().__init__(weighing_station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = APCWeighingStationModel()
        cls._set_model_common_fields(model, station_dict)
        model.funnel_storage_capacity = station_dict['properties']['funnel_storage_capacity']
        model.save()
        return cls(model)

    @property
    def balance_doors_open(self) -> bool:
        return self._model_proxy.balance_doors_open

    @balance_doors_open.setter
    def balance_doors_open(self, new_state: bool):
        self._model_proxy.balance_doors_open = new_state

    @property
    def vertical_doors_open(self) -> bool:
        return self._model_proxy.vertical_doors_open

    @vertical_doors_open.setter
    def vertical_doors_open(self, new_state: bool):
        self._model_proxy.vertical_doors_open = new_state

    @property
    def funnel_storage_capacity(self) -> int:
        return self._model_proxy.funnel_storage_capacity

    @property
    def funnel_storage_index(self) -> int:
        return self._model_proxy.funnel_storage_index

    @funnel_storage_index.setter
    def funnel_storage_index(self, new_value: int):
        self._model_proxy.funnel_storage_index = new_value

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        current_op = self.assigned_op
        if isinstance(current_op, APCWeighingOpenVDoorOp):
            self.vertical_doors_open = True
        elif isinstance(current_op, APCWeighingCloseVDoorOp):
            self.vertical_doors_open = False
        elif isinstance(current_op, APCOpenBalanceDoorOp):
            self.balance_doors_open = True
        elif isinstance(current_op, APCCloseBalanceDoorOp):
            self.balance_doors_open = False
        super().complete_assigned_op(outcome, results)



''' ==== Station Operation Descriptors ==== '''

class APCWeighingOpenVDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCWeighingStation.__name__)
        model.save()
        return cls(model)
    
class APCWeighingCloseVDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCWeighingStation.__name__)
        model.save()
        return cls(model)

class APCOpenBalanceDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCWeighingStation.__name__)
        model.save()
        return cls(model)

class APCCloseBalanceDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCWeighingStation.__name__)
        model.save()
        return cls(model)
    
class APCTareOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=APCWeighingStation.__name__)
        model.save()
        return cls(model)
    
class APCWeighingOp(StationSampleOp):
    def __init__(self, op_model: Union[StationSampleOpModel, ModelProxy]):
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=APCWeighingStation.__name__)
        model.save()
        return cls(model)

class APCWeighResult(StationOpResult):
    def __init__(self, result_model: Union[APCWeighResultModel, ModelProxy]):
        super().__init__(result_model)

    @classmethod
    def from_args(cls,
                  origin_op: ObjectId,
                  reading_value: float,
                  ):
        model = APCWeighResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.reading_value = reading_value
        model.save()
        return cls(model)

    @property
    def reading_value(self) -> float:
        return self._model_proxy.reading_value

    @property
    def unit(self) -> Literal["g"]:
        return self._model_proxy.unit
    
    

    
