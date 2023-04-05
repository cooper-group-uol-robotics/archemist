import os

HERE = os.path.dirname(os.path.abspath(__file__))
FILES = os.path.join(HERE, "resources")

TEST_RECIPE_YAMLS = os.path.join(FILES, "testing_recipe.yaml")


class TestFiles:
    recipe_yaml = TEST_RECIPE_YAMLS
