from archemist.state import material
from archemist.state.recipe import StationFlow, StationFlowNode, Recipe
from archemist.persistence.dbHandler import dbHandler
from archemist.persistence.fsHandler import FSHandler
import archemist.state.robots
import archemist.state.stations
import archemist.processing.stationSMs

from datetime import date, timedelta
import datetime
from multipledispatch import dispatch
import os
import sys
from archemist.util.location import Location

class Parser:

    @dispatch()
    def loadRecipeYaml(self):
        __location__ = os.path.realpath(
            os.path.join(os.getcwd(), os.path.dirname(__file__)))
        handler = FSHandler()
        return self.loadRecipeYaml(handler.loadYamlFile(os.path.join(__location__, 'workflowConfigs/recipes/recipe.yaml')))

    @dispatch(dict)
    def loadRecipeYaml(self, recipeDictionary):
        config = self.loadConfigYaml()

        liquidsList = list()
        for liquid in recipeDictionary["recipe"]["materials"]["liquids"]:
            for liquidMaterial in config[1]:
                if str(liquidMaterial.name) == liquid:
                    liquidsList.append(liquidMaterial)


        solidsList = list()
        for solid in recipeDictionary["recipe"]["materials"]["solids"]:
            for solidMaterial in config[2]:
                if str(solidMaterial.name) == solid:
                    solidsList.append(solidMaterial)


        stationOpDescriptors = list()
        for station in recipeDictionary["recipe"]["stations"]:
            for stationOpDesc in recipeDictionary["recipe"]["stations"][station]["stationOp"]:
                stationOpObj = self.str_to_class_station(stationOpDesc)
                # if (stationOpDesc == "PeristalticPumpOpDescriptor"):
                #     liquid = None
                #     for liq in liquidsList:
                #         if liq.__class__.__name__ == recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]["liquid"]:
                #             liquid = liq
                #     stationOpObj = stationOpObj(liquid)
                # elif (stationOpDesc == "IKAHeatingStirringOpDescriptor"):
                #     stationOpObj = stationOpObj(recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]["temp"],
                #     recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]["rpm"],
                #     recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]["duration"])
                # elif (stationOpDesc == "IKAHeatingOpDescriptor"):
                #     stationOpObj = stationOpObj(recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]["temp"],
                #     recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]["duration"])
                # elif (stationOpDesc == "IKAStirringOpDescriptor"):
                #     stationOpObj = stationOpObj(recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]["rpm"],
                #     recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]["duration"])
                input_properties = recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["properties"]
                stationOutputObj = self.str_to_class_station(recipeDictionary["recipe"]["stations"][station]["stationOp"][stationOpDesc]["output"]["name"])
                stationOpDescriptors.append(stationOpObj(input_properties, stationOutputObj(stationOpDesc)))

        stationFlowList = list()
        for state in recipeDictionary["recipe"]["stationFlow"]:
            if (state == "start"):
                stationFlowList.append(StationFlowNode(
                    state, None, None, recipeDictionary["recipe"]["stationFlow"][state]["onSuccess"], recipeDictionary["recipe"]["stationFlow"][state]["onFail"]))
            elif (state == "end"):
                stationFlowList.append(StationFlowNode(
                    state, None, None, "end", "end"))
            else:
                stationFlowList.append(StationFlowNode(state, recipeDictionary["recipe"]["stationFlow"][state]["station"], recipeDictionary["recipe"]["stationFlow"][state]["task"], recipeDictionary["recipe"]["stationFlow"][state]["onSuccess"], recipeDictionary["recipe"]["stationFlow"][state]["onFail"]))

        stationflow = StationFlow(stationFlowList)
        newRecipe = Recipe(recipeDictionary["recipe"]["name"], recipeDictionary["recipe"]["id"], stationOpDescriptors,
        stationflow, solidsList, liquidsList)
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

        robots = list()  # list of batches
        for robot in configDictionary["Robots"]:
            robotObj = self.str_to_class_robot(robot)
            robotObj = robotObj(configDictionary["Robots"][robot]["id"])
            robots.append(robotObj)
        configList.append(robots)

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
                mass = configDictionary["Materials"]["liquids"][liquid]["amount_stored"]/1000
                vol = mass/dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "g":
                mass = configDictionary["Materials"]["liquids"][liquid]["amount_stored"]
                vol = mass/dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "ug":
                mass = configDictionary["Materials"]["liquids"][liquid]["amount_stored"]/1000/1000
                vol = mass/dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "ml":
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_stored"]/1000
                mass = vol*dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "l":
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_stored"]
                mass = vol*dens
            elif configDictionary["Materials"]["liquids"][liquid]["unit"] == "ul":
                vol = configDictionary["Materials"]["liquids"][liquid]["amount_stored"]/1000/1000
                mass = vol*dens

            # other liquid properties added directly from yaml
            liquids.append(material.Liquid(
                liquid, configDictionary["Materials"]["liquids"][liquid]["pump_id"], exp_date, mass, dens, vol))
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
                mass = configDictionary["Materials"]["solids"][solid]["amount_stored"]/1000
            elif configDictionary["Materials"]["solids"][solid]["unit"] == "g":
                mass = configDictionary["Materials"]["solids"][solid]["amount_stored"]
            elif configDictionary["Materials"]["solids"][solid]["unit"] == "ug":
                mass = configDictionary["Materials"]["solids"][solid]["amount_stored"]/1000/1000

            # Add solid to list of solids
            solids.append(material.Solid(solid, configDictionary["Materials"]["solids"][solid]["cartridge_id"],
                          exp_date, mass, configDictionary["Materials"]["solids"][solid]["dispense_method"]))
        configList.append(solids)

        stations = list()  # list of stations
        for stationN in configDictionary["Stations"]:
            station_cls = self.str_to_class_station(stationN)
            station_sm_cls = self.str_to_class_state_machine(configDictionary["Stations"][stationN]["process_state_machine"])
            parameters = configDictionary["Stations"][stationN]["parameters"]
            station_sm_obj = station_sm_cls()
            station_obj = station_cls(configDictionary["Stations"][stationN]["id"], station_sm_obj, parameters, liquids, solids)
            stations.append(station_obj)
        configList.append(stations)  # add to list of lists


        return configList  # finally return list

    def str_to_class_robot(self, classname):
      return getattr(archemist.state.robots, classname)

    def str_to_class_station(self, classname):
      return getattr(archemist.state.stations, classname)

    def str_to_class_state_machine(self, classname):
      return getattr(archemist.processing.stationSMs, classname)
