from mongoengine import Document, fields
from archemist.core.models.op_result_model import OpResultModel
from archemist.core.models.lot_model import LotModel
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.sample_model import SampleModel
from archemist.core.util.enums import OpOutcome

class StationOpDescriptorModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    requested_by = fields.ObjectIdField(null=True) # station that generated the op
    associated_station = fields.StringField(required=True) # station associated with the op

    outcome = fields.EnumField(OpOutcome, null=True)
    results = fields.ListField(fields.ReferenceField(OpResultModel), default=[])
    
    start_timestamp = fields.ComplexDateTimeField(null=True)
    end_timestamp = fields.ComplexDateTimeField(null=True)

    meta = {'collection': 'station_ops', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class StationLotOpDescriptorModel(StationOpDescriptorModel):
    target_lot = fields.ReferenceField(LotModel)

class StationBatchOpDescriptorModel(StationOpDescriptorModel):
    target_batch = fields.ReferenceField(BatchModel)

class StationSampleOpDescriptorModel(StationOpDescriptorModel):
    target_sample = fields.ReferenceField(SampleModel)