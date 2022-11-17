# ARCHemist
## Description

ARCHemist is an open source architecture and application for the orchestration and management of robotic chemistry workflows. It is a reconfigurable architecture that allows different robots and lab instruments to work together to execute chemistry workflows. This architecture was designed with and for chemists, where it allows them to setup their experiments using human readable configuration and recipe files.

![ARCHemist operation overview](file:///C:/Users/hatem/Documents/archemist_gif.gif)

For more information on the architecture, please refer to our [published paper](https://arxiv.org/pdf/2204.13571.pdf)

## Installation
To run the ARChemist application and use the available developed robots and stations handlers, it need to be run in an environment that has ROS1 installed. This can be achieved by either running it:
- In a environment that have ROS1 globally installed, e.g., Ubuntu 20.04 with [ROS noetic](http://wiki.ros.org/noetic) installed.
- In a [RoboStack](https://robostack.github.io/) conda environment that has ROS1 installed inside the environment.

Please refer to the appropriate documentation to install ROS1 on your system.

Furthermore, the application requires [MongoDb](https://www.mongodb.com/) to be installed in order for its persistence layer to function. Please refer to the appropriate section on their website for installation instruction.

After installing ROS1, MongoDb and the appropriate ROS stations' and robots' drivers and handlers (more info [to follow](#ros-drivers-and-handlers)), clone the ARCHemist repository into your system and install the needed dependencies using the **requirements.txt** file using:

```
$ pip install -r requirements.txt
```
afterwards install the ARCHemist package using:
```
python setup.py install
```

## Usage
To use the ARCHemist to run chemistry experiments, first the user need to create a directory for their workflow using the script [create_workflow_dir.py](file://../scripts/create_workflow_dir.py). This script will create a directory with three subdirectories inside: 

1. **config_files** directory where the user will need add their workflow configuration and server settings files. To elaborate, these are:
   - *workflow_config.yaml*: this file provides the details on all the stations, robots and materials used in the workflow.
   - *server_settings.yaml*: this files stores MongoDb database information. These include the host address and the name of the database where the workflow information will be stored.  

2. **recipes** directory where the user will be adding recipe files to be executed. This folder is monitored by the application when running and every time a new recipe file is added, it will be added to the recipe execution queue.  
3. **templates** directory where templates for workflow configuration, server settings and recipe files are available. The user should complete the *workflow_config.yaml* and *server_settings.yaml* templates and add them to the *config_files* folder in order for the application to run. As for the sample recipe file, the user should create their own recipe files based on the template and add them to the recipes folder for execution. 

For more information on these files, check their documentation and refer to the [examples](files://examples/) folder for a number of example workflows.

After creating the workflow directory, run the ARCHemist server using:
```
python scripts/run_server.py --path path/to/workflow/dir
```
In case starting with an already existing workflow where its database entry exists use:
```
python scripts/run_server.py --path path/to/workflow/dir --exists
```
Next launch all the station and robot handlers using:
```
python scripts/launch_handlers.py
```
This will launch all the robots/stations handlers as specified in the configuration file. Note that, this assumes that all the robots and stations drivers are up and running and reachable by their respective handlers.

Alternatively, for testing and debugging purposes, simulated stations/robots handlers can be launched that won't connect to any physical equipment using:
```
python scripts/launch_handlers.py --sim
```
Finally, the ARCHemist command line interface can be launched using:
```
python scripts/run_cli.py
```
This will allow the user to start/pause the workflow processor, add clean batches to the workflow and access some basic robots/stations controls. 

**Important** the workflow processor starts in paused state by default so the user need to start it via the appropriate command. Furthermore, the workflow starts with no batches to process and thus any new added recipes won't be executed till clean batches are added. Finally, a station won't start processing any added batches till its batch capacity is reached so make sure to have enough batches in the workflow for the stations to operate.

## Adding new station
To add a new station to the ARCHemist system, a new module for it need to be created inside *archemist/stations* folder. This module should have the following structure:
```
└─ new_station_module
   |  __init__.py
   |  state.py
   |  model.py
   |  handler.py
   |  process.py
```
- *\_\_init__.py* to add the station as a python module
- 

## Adding new robot

## ROS drivers and handlers