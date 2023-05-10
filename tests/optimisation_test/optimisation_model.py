from mongoengine import Document, fields

class OptimisationModel(Document):
    optimizer_module = fields.StringField(required=True)
    optimizer_class = fields.StringField(required=True)
    optimizer_hyperparameters = fields.DictField()


    meta = {'collection': 'optimisation', 'db_alias': 'archemist_state'}