from mongoengine import Document, fields
from archemist.core.models.batch_model import BatchModel
from archemist.core.util.enums import OpOutcome

class RobotOpDescriptorModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    
    target_robot = fields.StringField(required=True)
    requested_by = fields.ObjectIdField(null=True) # station that generated the op
    executed_by = fields.ObjectIdField(null=True) # robot that executed the op

    outcome = fields.EnumField(OpOutcome, null=True)
    
    start_timestamp = fields.ComplexDateTimeField()
    end_timestamp = fields.ComplexDateTimeField()

    meta = {'collection': 'robot_ops', 'db_alias': 'archemist_state', 'allow_inheritance': True}

class RobotTaskOpDescriptorModel(RobotOpDescriptorModel):
    name = fields.StringField(required=True)
    params = fields.DictField(default={})
    target_batch = fields.ReferenceField(BatchModel, null=True)
    target_location = fields.DictField()

class CollectBatchOpDescriptorModel(RobotTaskOpDescriptorModel):
    target_onboard_slot = fields.IntField(null=True)

class DropBatchOpDescriptorModel(RobotTaskOpDescriptorModel):
    onboard_collection_slot = fields.IntField(null=True)

class RobotMaintenanceOpDescriptorModel(RobotOpDescriptorModel):
    target_robot_id = fields.IntField(required=True)
    name = fields.StringField(required=True)
    params = fields.DictField(default={})

class RobotNavOpDescriptorModel(RobotOpDescriptorModel):
    name = fields.StringField(required=True)
    target_location = fields.DictField(required=True)
    params = fields.DictField(default={})

class RobotWaitOpDescriptorModel(RobotOpDescriptorModel):
    timeout = fields.IntField(min_value=0)