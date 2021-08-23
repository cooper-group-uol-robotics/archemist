from pydoc import locate
from state import batch, material, recipe, station, result
import persistance.fsHandler
from datetime import date, timedelta
import datetime
class Parser:

    def loadRecipeYaml(self):
        handler = persistance.fsHandler.FSHandler()
        self.loadRecipeYaml(handler.loadYamlFile("recipe.yaml"))

    def loadRecipeYaml(self, recipeDictionary):
        self.recipe = recipe.Recipe(recipeDictionary["Workflow"]["name"], recipeDictionary["Workflow"]["ID"])
        
    
    def loadConfigYaml(self, configDictionaryInput):
        configDictionary = configDictionaryInput["workflow"]
        configList = list()
        
        batches = list()
        for batchn in configDictionary["Batches"]:
            batches.append(batch.Batch(configDictionary["Batches"][batchn]["name"], configDictionary["Batches"][batchn]["id"]))
        configList.append(batches)

        liquids = list()
        for liquid in configDictionary["Materials"]["liquids"]:
            exp_date = date.today() + timedelta(days=21)
            if "expiry_date" in configDictionary["Materials"]["liquids"][liquid]:
                exp_date = datetime.strptime(configDictionary["Materials"]["liquids"][liquid]["expiry_date"], '%d/%m/%y %H:%M:%S') 
            vol = 0
            mass = 0
            dens = configDictionary["Materials"]["liquids"][liquid]["density"]
            if configDictionary["Materials"]["liquids"][liquid]["unit"] == "mg":
                mass = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]/1000
                vol = mass/dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "g":
                mass = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]
                vol = mass/dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "ug":
                mass = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]/1000/1000
                vol = mass/dens    
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "ml":
                print("ml")
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]/1000
                mass = vol*dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "l":
                print("ml")
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]
                mass = vol*dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "ul":
                print("ml")
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]/1000/1000
                mass = vol*dens
            liquids.append(material.Liquid(liquid, configDictionary["Materials"]["liquids"][liquid]["id"], exp_date, mass, dens, vol))
        configList.append(liquids)

        solids = list()
        for solid in configDictionary["Materials"]["solids"]:
            exp_date = date.today() + timedelta(days=21)
            if "expiry_date" in configDictionary["Materials"]["solids"][solid]:
                exp_date = datetime.tstrptime(configDictionary["Materials"]["solids"][solid]["expiry_date"], '%d/%m/%y %H:%M:%S') 
            mass = 0
            if configDictionary["Materials"]["solids"][solid]["unit"] == "mg":
                mass = configDictionary["Materials"]["solids"][solid]["amount_to_dispense"]/1000
            elif configDictionary["Materials"]["solids"][solid]["unit"] == "g":
                mass = configDictionary["Materials"]["solids"][solid]["amount_to_dispense"]
            elif configDictionary["Materials"]["solids"][solid]["unit"] == "ug":
                mass = configDictionary["Materials"]["solids"][solid]["amount_to_dispense"]/1000/1000
            solids.append(material.Solid(solid, configDictionary["Materials"]["solids"][solid]["id"], exp_date, mass, configDictionary["Materials"]["solids"][solid]["dispense_method"]))
        configList.append(solids)

        locations = list()
        for location in configDictionary["Locations"]:
            locations.append(station.Location(location, configDictionary["Locations"][location]["node_id"], configDictionary["Locations"][location]["graph_id"], configDictionary["Locations"][location]["map_id"]))
        configList.append(locations)

        stations = list()
        for stationN in configDictionary["Stations"]:
            newstation = type(stationN, (station.Station, ), configDictionary["Stations"][stationN])
            locationOf = None
            for x in locations:
                if x.name == configDictionary["Stations"][stationN]["location"]:
                    locationOf = x.name
                    break
                else:
                    x = None
            newStationObj = newstation(stationN, configDictionary["Stations"][stationN]["id"], locationOf)
            stations.append(newStationObj)
        configList.append(stations)

        results = list()
        results.append(result.Result(configDictionary["output"]["characteristic"], locate(configDictionary["output"]["type"])))
        configList.append(results)

        return configList