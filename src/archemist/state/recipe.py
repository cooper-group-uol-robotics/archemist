class Recipe:
    def __init__(self, name, id):
        self.name = name
        self.id = id
        self.stationRecipes = list()
    
class StationRecipe:
    def __init__(self, name, stationAssoc, id):
        self.name = name
        self.id = id
        self.stationAssoc = stationAssoc