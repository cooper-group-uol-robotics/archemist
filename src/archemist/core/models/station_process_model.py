from mongoengine import Document, EmbeddedDocument, fields
from archemist.core.models.lot_model import LotModel
from archemist.core.models.robot_op_model import RobotOpDescriptorModel
from archemist.core.models.station_op_model import StationOpDescriptorModel
from archemist.core.util.enums import ProcessStatus

class keyOpDetailsModel(EmbeddedDocument):
    op_type = fields.StringField(required=True)
    repeat_for_all_batches = fields.BooleanField(required=True)
    parameters = fields.ListField(default=[])

class StationProcessModel(Document):

    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    ''' general '''
    uuid = fields.UUIDField(binary=False)
    requested_by = fields.ObjectIdField(null=True) # station that requested the process
    associated_station = fields.StringField(required=True) # station associated with the process
    status = fields.EnumField(enum=ProcessStatus, default=ProcessStatus.INACTIVE)
    state = fields.StringField(default="init_state")
    data = fields.DictField(default={})
    processing_slot = fields.IntField(min_value=0, default=None, null=True)
    skip_robot_ops = fields.BooleanField(default=False)
    skip_station_ops = fields.BooleanField(default=False)
    skip_ext_procs = fields.BooleanField(default=False)

    ''' lot '''
    lot = fields.ReferenceField(LotModel)

    ''' robot ops '''
    req_robot_ops = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel),default=[])
    robot_ops_history = fields.ListField(fields.ReferenceField(RobotOpDescriptorModel),default=[])
    
    '''station ops'''
    key_ops_dict = fields.MapField(field=fields.EmbeddedDocumentField(keyOpDetailsModel), default={})
    req_station_ops = fields.ListField(fields.ReferenceField(StationOpDescriptorModel), default=[])
    station_ops_history = fields.ListField(fields.ReferenceField(StationOpDescriptorModel), default=[])

    '''station processes'''
    req_station_procs = fields.ListField(fields.ReferenceField("self"), default=[])
    station_procs_history = fields.ListField(fields.ReferenceField("self"), default=[])

    meta = {'collection': 'station_processes', 'db_alias': 'archemist_state'}