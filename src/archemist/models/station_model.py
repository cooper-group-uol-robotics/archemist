from mongoengine import Document, fields
from archemist.util.enums import StationState
from archemist.models.batch_model import BatchModel
from archemist.models.station_op_model import StationOpDescriptorModel
from archemist.models.robot_op_model import RobotOpDescriptorModel

class StationModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    location = fields.DictField()
    batch_capacity = fields.IntField(min_value=1, default=1)
    process_state_machine = fields.DictField(required=True)
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