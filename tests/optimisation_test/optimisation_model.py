from mongoengine import connect, Document, fields

connect('my_test_db', host='127.0.0.1', port=27017)

class OptimisationModel(Document):
    optimizer_module = fields.StringField(required=True)
    optimizer_class = fields.StringField(required=True)
    optimizer_hyperparameters = fields.DictField()
    allowed_recipe_count = fields.IntField()


    # meta = {'collection': 'optimisation', 'db_alias': 'optimization_state'}