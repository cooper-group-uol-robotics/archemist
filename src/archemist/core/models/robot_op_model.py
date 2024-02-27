from mongoengine import Document, fields
from archemist.core.models.batch_model import BatchModel
from archemist.core.util.enums import OpOutcome
from archemist.core.util.location import LocationModel

class RobotOpModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    
    target_robot = fields.StringField(required=True)
    requested_by = fields.ObjectIdField(null=True) # station that generated the op
    executed_by = fields.ObjectIdField(null=True) # robot that executed the op

    outcome = fields.EnumField(OpOutcome, null=True)
    
    start_timestamp = fields.ComplexDateTimeField()
    end_timestamp = fields.ComplexDateTimeField()

    meta = {'collection': 'robot_ops', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class RobotTaskOpModel(RobotOpModel):
    name = fields.StringField(required=True)
    params = fields.DictField(default={})
    target_batch = fields.ReferenceField(BatchModel, null=True)
    task_type = fields.IntField(required=True)
    target_location = fields.EmbeddedDocumentField(LocationModel, default=LocationModel())
    fine_localization = fields.BooleanField(default=True)
    lbr_program_name = fields.StringField(required=True)
    lbr_program_params = fields.ListField(default=[])


class CollectBatchOpModel(RobotTaskOpModel):
    target_onboard_slot = fields.IntField(null=True)

class DropBatchOpModel(RobotTaskOpModel):
    onboard_collection_slot = fields.IntField(null=True)

class RobotMaintenanceOpModel(RobotOpModel):
    target_robot_id = fields.IntField(required=True)
    name = fields.StringField(required=True)
    params = fields.DictField(default={})

class RobotNavOpModel(RobotOpModel):
    name = fields.StringField(required=True)
    target_location = fields.EmbeddedDocumentField(LocationModel, default=LocationModel())
    params = fields.DictField(default={})

class RobotWaitOpModel(RobotOpModel):
    timeout = fields.IntField(min_value=0)

