from archemist.core.persistence.models_proxy import ModelProxy
from archemist.core.state.station import Station, StationModel

from typing import Union, Dict

class APCMetaStation(Station):
    def __init__(self, station_model: Union[StationModel, ModelProxy]) -> None:
        super().__init__(station_model)

    @classmethod
    def from_dict(cls, station_dict: Dict):
        model = StationModel()
        cls._set_model_common_fields(model, station_dict)
        model.save()
        return cls(model)