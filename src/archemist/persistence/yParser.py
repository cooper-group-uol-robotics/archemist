from pydoc import locate
from src.archemist.state import material, station
from src.archemist.state.result import Result
from src.archemist.state.recipe import StationFlow, StationFlowNode, Recipe
from src.archemist.persistence.dbHandler import dbHandler
from src.archemist.persistence.fsHandler import FSHandler
from src.archemist.state.robot import mobileRobot, armRobot
import src.archemist.state.robots
from datetime import date, timedelta
import datetime
from multipledispatch import dispatch
import os
import sys

class Parser:

    @dispatch()
    def loadRecipeYaml(self):
        __location__ = os.path.realpath(
            os.path.join(os.getcwd(), os.path.dirname(__file__)))
        handler = FSHandler()
        return self.loadRecipeYaml(handler.loadYamlFile(os.path.join(__location__, 'workflowConfigs\\recipes\\recipe.yaml')))

    @dispatch(dict)
    def loadRecipeYaml(self, recipeDictionary):
        config = self.loadConfigYaml()

        liquidsList = list()
        for liquid in recipeDictionary["recipe"]["materials"]["liquids"]:
            for liquidMaterial in config[0]:
                if str(liquidMaterial.name) == liquid:
                    liquidsList.append(liquidMaterial)


        solidsList = list()
        for solid in recipeDictionary["recipe"]["materials"]["solids"]:
            for solidMaterial in config[1]:
                if str(solidMaterial.name) == solid:
                    solidsList.append(solidMaterial)

        stationFlowList = list()
        for state, stateList in recipeDictionary["recipe"]["stationFlow"].items():
            for stateData in stateList:
                stationF = None
                for flowStation in config[4]:
                    if str(flowStation.name) == stateList["station"]:
                        stationF = flowStation
                ##todo: add stationOpDescriptor matching from config
            stationFlowList.append(StationFlowNode(
                state, stationF, stateList["task"], stateList["onsuccess"], stateList["onfail"]))


        outcomeDescriptors = list()
        for outcome in recipeDictionary["recipe"]["outcomeDescriptors"]:
            for outcomeDescriptor in config[5]:
                if str(outcomeDescriptor.name) == outcome:
                    outcomeDescriptors.append(outcomeDescriptor)

        stationflow = StationFlow(stationFlowList)

        newRecipe = Recipe(self, recipeDictionary["recipe"]["name"], recipeDictionary["recipe"]["id"], stationflow, solidsList, liquidsList, outcomeDescriptors)
        return newRecipe

    @dispatch()
    def loadConfigYaml(self):
        handler = dbHandler()
        config = dict()
        config = {'workflow': handler.getConfig()}
        return self.loadConfigYaml(config)
    
    

    @dispatch(dict)
    def loadConfigYaml(self, configDictionaryInput):
        # Get rid of top-level workflow key, only interested in everything below
        configDictionary = configDictionaryInput["workflow"]
        configList = list()  # list of lists, for each category of config

        liquids = list()  # list of liquids
        # loop through each key under "liquids"
        for liquid in configDictionary["Materials"]["liquids"]:
            # Set default expiration date to 3 weeks from now, in case one isn't set
            exp_date = date.today() + timedelta(days=21)
            # if expiry date is set then parse string to date type
            if "expiry_date" in configDictionary["Materials"]["liquids"][liquid]:
                exp_date = datetime.strptime(
                    configDictionary["Materials"]["liquids"][liquid]["expiry_date"], '%d/%m/%y %H:%M:%S')
            # convert between volume and mass using density. Density must be set
            vol = 0
            mass = 0
            dens = configDictionary["Materials"]["liquids"][liquid]["density"]
            # Also convert for every possible unit, mg, g, ug, ml, l and ul
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
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]/1000
                mass = vol*dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "l":
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]
                mass = vol*dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "ul":
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_to_dispense"]/1000/1000
                mass = vol*dens

            # other liquid properties added directly from yaml
            liquids.append(material.Liquid(
                liquid, configDictionary["Materials"]["liquids"][liquid]["id"], exp_date, mass, dens, vol))
        configList.append(liquids)

        solids = list()  # list of solids to be used
        for solid in configDictionary["Materials"]["solids"]:
            # default expiration date if not set
            exp_date = date.today() + timedelta(days=21)
            # if set, parse string to date
            if "expiry_date" in configDictionary["Materials"]["solids"][solid]:
                exp_date = datetime.tstrptime(
                    configDictionary["Materials"]["solids"][solid]["expiry_date"], '%d/%m/%y %H:%M:%S')

            mass = 0  # convert between mass units to universal grams
            if configDictionary["Materials"]["solids"][solid]["unit"] == "mg":
                mass = configDictionary["Materials"]["solids"][solid]["amount_to_dispense"]/1000
            elif configDictionary["Materials"]["solids"][solid]["unit"] == "g":
                mass = configDictionary["Materials"]["solids"][solid]["amount_to_dispense"]
            elif configDictionary["Materials"]["solids"][solid]["unit"] == "ug":
                mass = configDictionary["Materials"]["solids"][solid]["amount_to_dispense"]/1000/1000

            # Add solid to list of solids
            solids.append(material.Solid(solid, configDictionary["Materials"]["solids"][solid]["id"],
                          exp_date, mass, configDictionary["Materials"]["solids"][solid]["dispense_method"]))
        configList.append(solids)

        locations = list()  # list of possible locations
        # simply add each location to list directly from yaml
        for location in configDictionary["Locations"]:
            locations.append(station.Location(location, configDictionary["Locations"][location]["node_id"],
                             configDictionary["Locations"][location]["graph_id"], configDictionary["Locations"][location]["map_id"]))
        configList.append(locations)

        
        robots = list()  # list of batches
        for robot in configDictionary["Robots"]:
            robotObj = self.str_to_class_robot(configDictionary["Robots"][robot]["type"])
            if (issubclass(robotObj, mobileRobot)):
                locationOf = None
                for x in locations:
                    if x.name == configDictionary["Robots"][robot]["startLocation"]:
                        locationOf = x  # if it does then use this object
                        break
                robotObj = robotObj(robot, configDictionary["Robots"][robot]["id"], locationOf)
            else:
                robotObj = robotObj(robot, configDictionary["Robots"][robot]["id"])
            robots.append(robotObj)
        configList.append(robots)

        stations = list()  # list of stations
        for stationN in configDictionary["Stations"]:
            # See if location string matches up with a location object in the list of locations
            locationOf = None
            for x in locations:
                if x.name == configDictionary["Stations"][stationN]["location"]:
                    locationOf = x  # if it does then use this object
                    break
                else:
                    x = None
            # set dictionary entry to the location object (instead of name string)
            configDictionary["Stations"][stationN]["location"] = locationOf
            # dynamically create new station class for this station with its properties from yaml (with station as base class)
            newstation = type(stationN, (station.Station, ),
                              configDictionary["Stations"][stationN])
            # create object of new station class, passing parameters from yaml and also using location object
            newStationObj = newstation(
                stationN, configDictionary["Stations"][stationN]["id"], locationOf)
            stations.append(newStationObj)
        configList.append(stations)  # add to list of lists

        ##todo: parse station op descriptors from config, to be applied in stationflownodes in recipe

        results = list()  # create list of result/output specification from yaml
        for resultI in configDictionary["OutputDescriptors"]:
            resultN = Result(resultI, configDictionary["OutputDescriptors"][resultI]["characteristic"], locate(
                configDictionary["OutputDescriptors"][resultI]["type"]))
            results.append(resultN)
        configList.append(results)  # add to list of lists

        return configList  # finally return list

    def str_to_class_robot(self, classname):
      return getattr(src.archemist.state.robots, classname)