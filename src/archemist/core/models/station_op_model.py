from mongoengine import Document, fields

class StationOpDescriptorModel(Document):
    _type = fields.StringField(required=True)
    _module = fields.StringField(required=True)
    uuid = fields.UUIDField(binary=False, required=True)
    requested_by = fields.ObjectIdField(null=True) # station that generated the op
    associated_station = fields.StringField(required=True) # station associated with the op

    has_result = fields.BooleanField(default=False)
    was_successful = fields.BooleanField(default=False)
    start_timestamp = fields.ComplexDateTimeField(null=True)
    end_timestamp = fields.ComplexDateTimeField(null=True)

    meta = {'collection': 'station_ops', 'db_alias': 'archemist_state', 'allow_inheritance': True}