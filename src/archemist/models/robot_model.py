from mongoengine import Document, fields
from archemist.models.robot_op_model import RobotOpDescriptorModel
from archemist.util.enums import RobotState


class RobotModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    exp_id = fields.IntField(required=True)
    operational = fields.BooleanField(default=True)
    batch_capacity = fields.IntField(min_value=1, default=1)
    location = fields.DictField()
    assigned_op = fields.EmbeddedDocumentField(RobotOpDescriptorModel, null=True)
    complete_op = fields.EmbeddedDocumentField(RobotOpDescriptorModel, null=True)
    state = fields.EnumField(RobotState, default=RobotState.IDLE)
    robot_op_history = fields.EmbeddedDocumentListField(RobotOpDescriptorModel, default=[])

    meta = {'collection': 'robots', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class MobileRobotModel(RobotModel):
    onboard_batches = fields.ListField(fields.IntField(),default=[])