from mongoengine import Document, EmbeddedDocument, fields
from archemist.core.util.enums import StationState
from archemist.core.models.batch_model import BatchModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.models.robot_op_model import RobotOpDescriptorModel

class StationProcessDataModel(EmbeddedDocument):
    uuid = fields.UUIDField(binary=False)
    batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    req_robot_ops = fields.EmbeddedDocumentListField(RobotOpDescriptorModel,default=[])
    req_station_ops = fields.EmbeddedDocumentListField(StationOpDescriptorModel,default=[])
    status = fields.DictField(default={})

class StationModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    location = fields.DictField()
    batch_capacity = fields.IntField(min_value=1, default=1)
    process_batch_capacity = fields.IntField(min_value=1, default=1)
    process_state_machine = fields.DictField(required=True)
    process_data_map = fields.MapField(fields.EmbeddedDocumentField(StationProcessDataModel), default={})
    selected_handler = fields.StringField(required=True)
    operational = fields.BooleanField(default=True)
    state = fields.EnumField(StationState, default=StationState.IDLE)
    loaded_samples = fields.IntField(default=0) # cannot use min value, breaks dec__ operator
    assigned_batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    processed_batches = fields.ListField(fields.ReferenceField(BatchModel), default=[])
    requested_robot_op = fields.EmbeddedDocumentField(RobotOpDescriptorModel,null=True)
    assigned_station_op = fields.EmbeddedDocumentField(StationOpDescriptorModel,null=True)
    station_op_history = fields.EmbeddedDocumentListField(StationOpDescriptorModel,default=[])
    requested_robot_op_history = fields.EmbeddedDocumentListField(RobotOpDescriptorModel, default=[])

    meta = {'collection': 'stations', 'db_alias': 'archemist_state', 'allow_inheritance': True}