from typing import Dict, Union, Any, List, Literal, Type
from archemist.core.persistence.models_proxy import ModelProxy, EmbedModelProxy, ListProxy
from .model import ApcWeighResultModel, ApcWeighingStationModel
from archemist.core.models.station_model import StationModel
from archemist.core.models.station_op_model import StationSampleOpModel
from archemist.core.state.station import Station
from archemist.core.state.station_op import StationOp, StationSampleOp, StationOpModel
from archemist.core.state.station_op_result import StationOpResult
from archemist.core.state.sample import Sample
from archemist.core.util.enums import OpOutcome
from bson.objectid import ObjectId


''' ==== Station Description ==== '''
class ApcWeighingStation(Station):
    def __init__(self, weighing_station_model: Union[ApcWeighingStationModel, ModelProxy]) -> None:
        self._model = weighing_station_model

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = ApcWeighingStationModel()
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

    def complete_assigned_op(self, outcome: OpOutcome, results: List[Type[StationOpResult]]):
        current_op = self.assigned_op
        if isinstance(current_op, ApcWeighingVOpenDoorOp):
            self.vertical_doors_open = True
        elif isinstance(current_op, ApcWeighingVCloseDoorOp):
            self.vertical_doors_open = False
        elif isinstance(current_op, ApcBalanceOpenDoorOp):
            self.balance_doors_open = True
        elif isinstance(current_op, ApcBalanceCloseDoorOp):
            self.balance_doors_open = False
        super().complete_assigned_op(outcome, results)



''' ==== Station Operation Descriptors ==== '''

class ApcWeighingVOpenDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=ApcWeighingStation.__name__)
        model.save()
        return cls(model)
    
class ApcWeighingVCloseDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=ApcWeighingStation.__name__)
        model.save()
        return cls(model)

class ApcBalanceOpenDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=ApcWeighingStation.__name__)
        model.save()
        return cls(model)

class ApcBalanceCloseDoorOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=ApcWeighingStation.__name__)
        model.save()
        return cls(model)
    
class ApcTareOp(StationOp):
    def __init__(self, op_model: Union[StationOpModel, ModelProxy]) -> None:
        super().__init__(op_model)

    @classmethod
    def from_args(cls):
        model = StationOpModel()
        cls._set_model_common_fields(model, associated_station=ApcWeighingStation.__name__)
        model.save()
        return cls(model)
    
class ApcWeighingOp(StationSampleOp):
    def __init__(self, op_model: Union[StationSampleOpModel, ModelProxy]):
        super().__init__(op_model)

    @classmethod
    def from_args(cls, target_sample: Sample):
        model = StationSampleOpModel()
        model.target_sample = target_sample.model
        cls._set_model_common_fields(model, associated_station=ApcWeighingStation.__name__)
        model.save()
        return cls(model)

class ApcWeighResult(StationOpResult):
    def __init__(self, result_model: Union[ApcWeighResultModel, ModelProxy]): #TODO need this model
        super().__init__(result_model)

    @classmethod
    def from_args(cls,
                  origin_op: ObjectId,
                  reading_value: float,
                  ):
        model = ApcWeighResultModel()
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
    
    

    
