from typing import Dict, Union, Any, List, Literal, Type
from archemist.core.persistence.models_proxy import ModelProxy, EmbedModelProxy, ListProxy
from .model import WeighResultModel, WeighingStationModel
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationSampleOpModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOp, StationSampleOp, StationOpModel
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.sample import Sample
from archemist.core.util.enums import OpOutcome
from bson.objectid import ObjectId


''' ==== Station Description ==== '''
class WeighingStation(Station):
    def __init__(self, weighing_station_model: Union[WeighingStationModel, ModelProxy]) -> None:
        self._model = weighing_station_model

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = WeighingStationModel()
        cls._set_model_common_fields(model, station_dict)
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
    def funnel_loaded(self) -> bool:
        return self._model_proxy.funnel_loaded

    @funnel_loaded.setter
    def funnel_loaded(self, new_state: bool):
        self._model_proxy.funnel_loaded = new_state

    def update_assigned_op(self):
        super().update_assigned_op()
        current_op = self.assigned_op
        # TODO maybe more needed in here?

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        current_op = self.assigned_op
        super().complete_assigned_op(outcome, results)
        # TODO maybe more needed in here?


''' ==== Station Operation Descriptors ==== '''

class WeighingVOpenDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WeighingStation.__name__)
        model.vertical_doors_open = True
        model.save()
        return cls(model)
    
class WeighingVCloseDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WeighingStation.__name__)
        model.vertical_doors_open = False
        model.save()
        return cls(model)

class BalanceOpenDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WeighingStation.__name__)
        balance_doors_open = True
        model.save()
        return cls(model)

class BalanceCloseDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WeighingStation.__name__)
        balance_doors_open = False
        model.save()
        return cls(model)
    
class LoadFunnelOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, cartridge_index: int):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WeighingStation.__name__)
        model.funnel_loaded = True
        model.save()
        return cls(model)
    
class UnloadFunnelOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls, cartridge_index: int):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WeighingStation.__name__)
        model.funnel_loaded = False
        model.save()
        return cls(model)
    
class TareOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=WeighingStation.__name__)
        model.save()
        return cls(model)
    
class WeighingOp(StationSampleOp):
    def __init__(self, op_model: StationSampleOpModel):
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=WeighingStation.__name__)
        model.save()
        return cls(model)

class WeighResult(StationOpResult):
    def __init__(self, result_model: Union[WeighResultModel, ModelProxy]): #TODO need this model
        super().__init__(result_model)

    @classmethod
    def from_args(cls,
                  origin_op: ObjectId,
                  reading_value: float,
                  unit: Literal["g"]):
        model = WeighResultModel()
        cls._set_model_common_fields(model, origin_op)
        model.reading_value = reading_value
        model.unit = unit
        model.save()
        return cls(model)

    @property
    def reading_value(self) -> float:
        return self._model_proxy.reading_value

    @property
    def unit(self) -> Literal["g"]:
        return self._model_proxy.unit
    
    

    
