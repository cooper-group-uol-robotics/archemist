from mongoengine import EmbeddedDocument, fields
from archemist.util.enums import RobotTaskType

class RobotOpDescriptorModel(EmbeddedDocument):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    origin_station = fields.ObjectIdField(null=True)
    related_batch_id = fields.IntField(null=True)

    has_result = fields.BooleanField(default=False)
    was_successful = fields.BooleanField(default=False)
    robot_stamp = fields.StringField()
    start_timestamp = fields.ComplexDateTimeField()
    end_timestamp = fields.ComplexDateTimeField()

    meta = {'allow_inheritance':True}

class RobotTaskOpDescriptorModel(RobotOpDescriptorModel):
    name = fields.StringField(required=True)
    task_type = fields.EnumField(RobotTaskType, required=True)
    params = fields.ListField(fields.StringField(), default=True)
    location = fields.DictField()