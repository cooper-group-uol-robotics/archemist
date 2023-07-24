from mongoengine import connect, Document, fields
from archemist.core.models.batch_model import BatchModel

connect('algae_bot_workflow', host='127.0.0.1', port=27017)


class OptimisationModel(Document):
    optimizer_module = fields.StringField(required=True)
    optimizer_class = fields.StringField(required=True)
    optimizer_wrapper_module = fields.StringField(required=True)
    optimizer_wrapper_class = fields.StringField(required=True)
    optimizer_hyperparameters = fields.DictField()
    max_recipe_count = fields.IntField()
    batches_seen = fields.ListField()

    


    # meta = {'collection': 'optimisation', 'db_alias': 'optimization_state'}