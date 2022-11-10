# ARCHemist
## Description

ARCHemist is an open source architecture and application for the orchestration and management of robotic chemistry workflows. It is a reconfigurable architecture that allows different robots and lab instruments to work together to execute chemistry workflows. This architecture was designed with and for chemists, where it allows them to setup their experiments using human readable configuration and recipe files.

![ARCHemist operation overview](file:///C:/Users/hatem/Documents/archemist_gif.gif)

For more information on the architecture, please refer to our [published paper](https://arxiv.org/pdf/2204.13571.pdf)

## Installation
To run the ARChemist system and use the available developed robot and station handlers, it need to be run in an environment that has ROS1 installed. This can be achieved by either running it:
- In a environment that have ROS1 globally installed, e.g., Ubuntu 20.04 with ROS noetic installed.
- In a [RoboStack](https://robostack.github.io/) conda environment that has ROS1 installed inside the environment.

Please refer to the appropriate documentation to install ROS1 on your system. 

After installing ROS1 and the appropriate ROS stations' and robots' drivers and handlers (more info in [this section](#ros-drivers-and-handlers)), clone the ARCHemist repository into your system and install the needed dependencies using the **requirements.txt** file:

```
$ pip install -r requirements.txt
```
afterwards install the ARCHemist package using:
```
python setup.py install
```

## Usage
To use the ARCHemist system to run your chemistry workflow, first you need to create a directory for your workflow using the script [create_workflow.py](file:///scripts/create_workflow.py). This script will create a directory with two subdirectories inside: 

1. a *config_files* directory with a template workflow configuration file, which you will need to complete. 

1. *recipes* directory where you will be adding recipes files to be executed. 

create a *yaml* config file that describes the materials, stations, robots and operations involved in the process. Here is an example of the file format.

```yaml
workflow:
  # This section provides general details on the workflow, which include its name, number of samples per batch and the batch default input location.
  General:
    name: my_cool_workflow
    samples_per_batch: 2
    default_batch_input_location:
      node_id: 25
      graph_id: 1
  # This section details all the materials used in the workflow, which include the various solids and liquids.
  Materials:
    solids:
      - name: NaCl
        id: 123
        amount_stored: 1000
        dispense_src: quantos
        cartridge_id: 31
        unit: mg
        expiry_date: 2025-02-11
    liquids:
      - name: H2O
        id: 145
        amount_stored: 400
        unit: ml
        density: 997 #g/l
        pump_id: p1
        expiry_date: 2025-02-11
  # This section describes all the robot used in the workflow.
  Robots:
    - type: KukaLBRIIWA
      id: 1
      batch_capacity: 2
  # This section provides details on the stations used in the workflow and their details. Note that the process state machine described the operations need to be carried out by the station to fulfill its role.
  Stations:
    - type: QuantosSolidDispenserQS2
      id: 12
      location:
        node_id: 27
        graph_id: 1
      batch_capacity: 2
      process_state_machine:
        type: StationLoadingSm
        args:
          batch_mode: true
      parameters:
        catridges:
            id: 31
            hotel_index: 1
            remaining_dosages: 98
```
After de

## Adding new station

## Adding new robot

## ROS drivers and handlers