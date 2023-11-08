from mongoengine import Document, fields
from archemist.core.models.station_op_result_model import StationOpResultModel
from archemist.core.models.lot_model import LotModel
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.sample_model import SampleModel
from archemist.core.util.enums import OpOutcome

class StationOpModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    requested_by = fields.ObjectIdField(null=True) # station that generated the op
    associated_station = fields.StringField(required=True) # station associated with the op

    outcome = fields.EnumField(OpOutcome, null=True)
    results = fields.ListField(fields.ReferenceField(StationOpResultModel), default=[])
    
    start_timestamp = fields.ComplexDateTimeField(null=True)
    end_timestamp = fields.ComplexDateTimeField(null=True)

    meta = {'collection': 'station_ops', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class StationLotOpModel(StationOpModel):
    target_lot = fields.ReferenceField(LotModel)

class StationBatchOpModel(StationOpModel):
    target_batch = fields.ReferenceField(BatchModel)

class StationSampleOpModel(StationOpModel):
    target_sample = fields.ReferenceField(SampleModel)