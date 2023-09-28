from archemist.core.models.robot_op_model import RobotOpDescriptorModel, RobotTaskOpDescriptorModel
from archemist.core.models.robot_model import MobileRobotModel
from mongoengine import fields

class KukaLBRIIWAModel(MobileRobotModel):
    locked_to_station = fields.BooleanField(default=False)

class KukaNAVTaskModel(RobotOpDescriptorModel):
    target_location = fields.DictField(required=True)
    fine_localisation = fields.BooleanField(default=False)

class KukaLBRTaskModel(RobotTaskOpDescriptorModel):
    lock_robot = fields.StringField(default="", choices=["", "lock", "unlock"])
