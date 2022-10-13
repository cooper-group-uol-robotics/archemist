from archemist.state.robot import Robot, RobotModel
from bson.objectid import ObjectId


class PandaFranka(Robot):
    def __init__(self, robot_model: RobotModel) -> None:
        self._model = robot_model

    @classmethod
    def from_dict(cls, robot_document: dict):
        model = RobotModel()
        model._type = cls.__name__
        cls._set_model_common_fields(robot_document, model)
        model.save()
        return cls(model)

    @classmethod
    def from_object_id(cls, object_id: ObjectId):
        model = RobotModel.objects.get(id=object_id)
        return cls(model)