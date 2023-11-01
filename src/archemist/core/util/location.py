from archemist.core.persistence.models_proxy import EmbedModelProxy
from typing import Union, Tuple, Dict
from mongoengine import EmbeddedDocument, fields

class LocationModel(EmbeddedDocument):
    coordinates = fields.PointField(null=True)
    descriptor = fields.StringField(default="unknown")

class Location:
    def __init__(self, location_model: Union[LocationModel, EmbedModelProxy]):
        self._model_proxy = location_model

    @classmethod
    def from_args(cls, coordinates: Tuple[int, int]=(), descriptor: str="unknown"):
        model = LocationModel()
        if coordinates:
            coordinates_dict = {"type": "Point",
                                "coordinates": [coordinates[0], coordinates[1]]}
        else:
            coordinates_dict = None
        model.coordinates =  coordinates_dict
        model.descriptor = descriptor
        return cls(model)
    
    @classmethod
    def from_dict(cls, location_dict: Dict):
        model = LocationModel()
        coordinates = location_dict.get("coordinates")
        if coordinates:
            coordinates_dict = {"type": "Point",
                                "coordinates": [coordinates[0], coordinates[1]]}
        else:
            coordinates_dict = None
        model.coordinates = coordinates_dict
        model.descriptor = location_dict.get("descriptor", "unknown")
        return cls(model)
    
    @property
    def model(self) -> LocationModel:
        if isinstance(self._model_proxy, EmbedModelProxy):
            return self._model_proxy.model
        else:
            return self._model_proxy
    
    @property
    def coordinates(self) -> Tuple[int, int]:
        if self._model_proxy.coordinates:
            return tuple(self._model_proxy.coordinates["coordinates"])
        else:
            return ()
    
    @property
    def descriptor(self) -> str:
        return self._model_proxy.descriptor
    
    def is_unspecified(self) -> bool:
        return self.coordinates == () and self.descriptor == "unknown"
    
    def __eq__(self, __value: object) -> bool:
        return self.coordinates == __value.coordinates and self.descriptor == __value.descriptor
    
    def __str__(self) -> str:
        return f"coordinates:{self.coordinates} - descriptor: {self.descriptor}"
    
