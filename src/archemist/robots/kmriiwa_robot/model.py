from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from mongoengine import fields


class KukaNAVTaskModel(RobotOpDescriptorModel):
    target_location = fields.DictField(required=True)
    fine_localisation = fields.BooleanField(default=False)
