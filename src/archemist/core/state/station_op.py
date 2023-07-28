from datetime import datetime
from archemist.core.models.station_op_model import StationOpDescriptorModel

class StationOpDescriptor:
    def __init__(self, stationOpModel: StationOpDescriptorModel) -> None:
        self._model = stationOpModel

    @property
    def model(self):
        return self._model
    
    @property
    def type(self):
        return self._model._type

    @property
    def has_result(self):
        return self._model.has_result

    @property
    def was_successful(self):
        return self._model.was_successful

    @property
    def start_timestamp(self):
        return self._model.start_timestamp

    @property
    def end_timestamp(self):
        return self._model.end_timestamp

    def add_start_timestamp(self):
        self._model.start_timestamp = datetime.now()

    def complete_op(self, success: bool, **kwargs):
        self._model.has_result = True
        self._model.was_successful = success
        self._model.end_timestamp = datetime.now()