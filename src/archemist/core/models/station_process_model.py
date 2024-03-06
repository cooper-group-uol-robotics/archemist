from mongoengine import Document, EmbeddedDocument, fields
from archemist.core.models.lot_model import LotModel
from archemist.core.models.robot_op_model import RobotOpModel
from archemist.core.models.station_op_model import StationOpModel
from archemist.core.util.enums import ProcessStatus


class OperationSpecsModel(EmbeddedDocument):
    op_type = fields.StringField(required=True)
    parameters = fields.DictField(default={})


class StationProcessModel(Document):

    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    ''' general '''
    requested_by = fields.ObjectIdField(null=True)  # station that requested the process
    assigned_to = fields.ObjectIdField(null=True)  # station that will be running the process
    associated_station = fields.StringField(required=True)  # station associated with the process
    status = fields.EnumField(enum=ProcessStatus, default=ProcessStatus.INACTIVE)
    state = fields.StringField(default="init_state")
    data = fields.DictField(default={})
    lot_slot = fields.IntField(min_value=0, default=None, null=True)
    is_subprocess = fields.BooleanField(default=False)
    skip_robot_ops = fields.BooleanField(default=False)
    skip_station_ops = fields.BooleanField(default=False)
    skip_ext_procs = fields.BooleanField(default=False)

    ''' lot '''
    lot = fields.ReferenceField(LotModel)

    ''' robot ops '''
    req_robot_ops = fields.ListField(fields.ReferenceField(RobotOpModel), default=[])
    robot_ops_history = fields.ListField(fields.ReferenceField(RobotOpModel), default=[])

    '''station ops'''
    operation_specs_map = fields.MapField(field=fields.EmbeddedDocumentField(OperationSpecsModel), default={})
    req_station_ops = fields.ListField(fields.ReferenceField(StationOpModel), default=[])
    station_ops_history = fields.ListField(fields.ReferenceField(StationOpModel), default=[])

    '''station processes'''
    req_station_procs = fields.ListField(fields.ReferenceField("self"), default=[])
    station_procs_history = fields.ListField(fields.ReferenceField("self"), default=[])

    meta = {'collection': 'station_processes', 'db_alias': 'archemist_state'}
