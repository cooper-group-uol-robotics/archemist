from mongoengine import Document, fields
from archemist.core.util.enums import RobotTaskType
from archemist.core.models.batch_model import BatchModel

class RobotOpDescriptorModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    uuid = fields.UUIDField(binary=False, required=True)
    requested_by = fields.ObjectIdField(null=True) # station that generated the op
    executed_by = fields.ObjectIdField(null=True) # robot that executed the op
    related_batch = fields.ReferenceField(BatchModel, null=True)

    has_result = fields.BooleanField(default=False)
    was_successful = fields.BooleanField(default=False)
    start_timestamp = fields.ComplexDateTimeField()
    end_timestamp = fields.ComplexDateTimeField()

    meta = {'collection': 'robot_ops', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class RobotTaskOpDescriptorModel(RobotOpDescriptorModel):
    name = fields.StringField(required=True)
    task_type = fields.EnumField(RobotTaskType, required=True)
    params = fields.ListField(fields.StringField(), default=True)
    location = fields.DictField()