from mongoengine import connect, Document, fields

class OptimisationModel(Document):
    optimizer_module = fields.StringField(required=True)
    optimizer_class = fields.StringField(required=True)
    objective_variable = fields.DictField(required=True)
    decision_variables = fields.ListField(required=True)
    optimizer_hyperparameters = fields.DictField()
    recipe_template_name = fields.StringField(required=True)
    generated_recipes_suffix = fields.StringField(default="recipe")
    generated_recipes_prefix = fields.StringField(default="recipe")
    max_recipe_count = fields.IntField()
    batches_seen = fields.ListField()

    meta = {'collection': 'optimisation', 'db_alias': 'archemist_state'}