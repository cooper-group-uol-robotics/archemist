# ARChemist
## Description

ARChemist is an open source architecture and application for the orchestration and management of robotic chemistry workflows. It is a reconfigurable architecture that allows different robots and lab instruments to work together to execute chemistry workflows. This architecture was designed with and for chemists, where it allows them to setup their experiments using human readable configuration and recipe files.

![archemist_gif](https://user-images.githubusercontent.com/13589969/202526574-b4121573-72c4-4b14-9d12-1f46d9a3eef3.gif)

For more information on the architecture, please refer to our [published paper](https://ieeexplore.ieee.org/document/9811996)

## Installation
To run the ARChemist application and use the available developed robots and stations handlers, it need to be run in an environment that has ROS1 installed. This can be achieved by either running it:
- In a environment that have ROS1 globally installed, e.g., Ubuntu 20.04 with [ROS noetic](http://wiki.ros.org/noetic) installed.
- In a [RoboStack](https://robostack.github.io/) conda environment that has ROS1 installed inside the environment.

Please refer to the appropriate documentation to install ROS1 on your system.

Furthermore, the application requires [MongoDb](https://www.mongodb.com/) to be installed in order for its persistence layer to function. Please refer to the appropriate section on their website for installation instruction.

After installing ROS1, MongoDb and the appropriate ROS stations' and robots' drivers and handlers (more info [to follow](#ros-drivers-and-handlers)), clone the ARChemist repository into your system and install the package using:

```
$ pip install .
```
or alternatively using:
```
$ python setup.py install
```

## Usage
### Command line Interface
To use the ARChemist to run chemistry experiments, first the user need to create a directory for their workflow using the script [create_workflow_dir](./scripts/create_workflow_dir.py). This script will create a directory with three subdirectories inside: 

1. **config_files** directory where the user will need add their workflow configuration and server settings files. To elaborate, these are:
   - *workflow_config.yaml*: this file provides the details on all the stations, robots and materials used in the workflow.
   - *server_settings.yaml*: this files stores MongoDb database information. These include the host address and the name of the database where the workflow information will be stored.  

2. **recipes** directory where the user will be adding recipe files to be executed. This folder is monitored by the application when running and every time a new recipe file is added, it will be added to the recipe execution queue.  
3. **templates** directory where templates for workflow configuration, server settings and recipe files are available. The user should complete the *workflow_config.yaml* and *server_settings.yaml* templates and add them to the *config_files* folder in order for the application to run. As for the sample recipe file, the user should create their own recipe files based on the template and add them to the recipes folder for execution. 

For more information on these files, check their documentation and refer to the [examples](./examples/) folder for a number of example workflows.

After creating the workflow directory, start the ARChemist application server using:
```
$ archemist start_server --path path/to/workflow/dir
```
In case starting with an already existing workflow where its database entry exists use:
```
$ archemist start_server --path path/to/workflow/dir --exists
```
Next launch all the station and robot handlers using:
```
$ archemist launch_handlers --path path/to/workflow/dir
```
This will launch all the robots/stations handlers as specified in the configuration file. Note that, this assumes that all the robots and stations drivers are up and running and reachable by their respective handlers.

Alternatively, for testing and debugging purposes, simulated stations/robots handlers can be launched that won't connect to any physical equipment using:
```
$ archemist launch_handlers --path path/to/workflow/dir --sim
```
Finally, the ARChemist command line interface can be launched using:
```
$ archemist start_cli
```
This will allow the user to start/pause the workflow processor, add clean batches to the workflow and access some basic robots/stations controls. 

**Important** the workflow processor starts in paused state by default so the user need to start it via the appropriate command. Furthermore, the workflow starts with no batches to process and thus any new added recipes won't be executed till clean batches are added. Finally, a station won't start processing any added batches till its batch capacity is reached so make sure to have enough batches in the workflow for the stations to operate.

### Web user interface
this will be added later

## Adding new stations and robots
Before explaining the process of adding new stations and robots to the ARChemist application, the diagram below illustrates the dataflow inside the applictaion and how a station/robot represented inside the application interacts with its real-world counterpart.

<p align="center">
  <img width="640" height="370" src="https://user-images.githubusercontent.com/13589969/202523533-976611d3-a0ab-4d0b-bd61-2421b8dd1eca.png">
</p>

### Adding new station
To add a new station to the ARChemist system, a new module for it need to be created inside [archemist/stations](./src/archemist/stations/) folder. This module should have the following structure:
```
archemist/stations
└─ new_station_module
   |  __init__.py
   |  state.py
   |  model.py
   |  handler.py
   |  process.py
```
- **\_\_init__.py** : is needed to use the station as a python module
- **model.py** : defines the database models for the station and all of its operations. For the station model, it needs to inherit from *StationModel* class found in [station_model.py](./src/archemist/core/model/station_model.py) and then new fields can be added to describe the station. Similarly, to define a new station operation model, it needs to inherit from *StationOpDescriptorModel* found [station_op_model.py](./src/archemist/core/model/station_op_model.py) so its news fields can be defined.
- **state.py** : define classes that represent the station and its operation in the ARChemist application, that wraps their models and provide methods that allow to retrieve and alter their states inside the application. The station state class needs to inherit from *Station* class found [station.py](./src/archemist/core/state/station.py) and define any new methods needed to access and alter its database model. Similarly, the station operation state class needs to inherit from *StationOpDescriptor* class found [station_op.py](./src/archemist/core/state/station_op.py).
- **handler.py** : define a handler that handles the interaction between the station application representation and its real world counterpart. This is achieved by communicating with its driver using the appropriate medium so that its operations can be executed and its state can be updated in real time. The station handler class needs to inherit from *StationHandler* found [handler.py](./src/archemist/core/processing/handler.py) and implement the methods: 1) execute_op, 2) is_op_execution_complete, 3) get_op_result and 4) run. Note that many handlers can be defined for a single station, in case it has multiple drivers that use different communication channels. When running the workflow, the appropriate handler needs to be specified in the workflow config file, as only a single handler per station can run.
- **process.py** : defines the flow of operation that the station will go through in order to execute its operations and perform its function. This include both station operation and required robot tasks to proceed. Currently, the process is represented using a [finite state machine](https://en.wikipedia.org/wiki/Finite-state_machine), where its states represent the various station operations and robot task requests required to complete the process. To define a new process state machine, it needs to inherit from *StationProcessFSM* found [station_process_fsm.py](./src/archemist/core/processing/station_process_fsm.py) and define all its states and transitions. Note that many process state machines can be defined for a station since its utilisation depends on the workflow requirements, e.g., two processes where each use a different robot. When running the workflow, the appropriate process needs to be specified in the workflow config file, as only a single process per station can be executing.

To better understand how to define a new station and add it to the ARChemist application, please refer to any of the station modules defined inside [archemist/stations](./src/archemist/stations/) folder.

### Adding new robot
To add a new robot to the ARChemist system, a new module for it need to be created inside [archemist/robots](./src/archemist/robots/) folder. This module should have the following structure:
```
archemist/robots
└─ new_station_module
   |  __init__.py
   |  state.py
   |  model.py
   |  handler.py
```
- **\_\_init__.py** : is needed to use the robot as a python module
- **model.py** : defines the database models for the robot and its tasks. For the robot model, it needs to inherit from *RobotModel* or *MobileRobotModel* classes found [robot_model.py](./src/archemist/core/model/robot_model.py) and then any new fields can be added as needed. Similarly, to define a new specific robot task model, it needs to inherit from *RobotOpDescriptorModel* or *RobotTaskOpDescriptorModel* found [robot_op_model.py](./src/archemist/core/model/robot_op_model.py).
- **state.py** : define classes that represent the robot and its specific tasks in the ARChemist application, that wraps their models and provide methods that allow to retrieve and alter their states inside the application. The robot state class needs to inherit from *Robot* class found [robot.py](./src/archemist/core/state/robot.py) and define any new methods needed to access and alter its database model. Similarly, the robot operation state class needs to inherit from *RobotOpDescriptor* or *RobotTaskOpDescriptor* classes found [robot_op.py](./src/archemist/core/state/robot_op.py).
- **handler.py** : define a handler that handles the interaction between the robot application representation and its real world counterpart. This is achieved by communicating with its driver using the appropriate medium so that its tasks can be executed and its state can be updated in real time. The robot handler class needs to inherit from *RobotHandler* found [handler.py](./src/archemist/core/processing/handler.py) and implement the methods: 1) execute_op, 2) is_op_execution_complete, 3) get_op_result and 4) run. Note that many handlers can be defined for a single robot, in case it has multiple drivers that use different communication channels. When running the workflow, the appropriate handler needs to be specified in the workflow config file, as only a single handler per robot can run.

To better understand how to define a new robot and add it to the ARChemist application, please refer to any of the robot modules defined inside [archemist/robots](./src/archemist/robots/) folder.

## ROS drivers and handlers
In the current implementation, the handlers developed for the all the defined stations and robots are implemented using ROS1, since their drivers use ROS1 for communication. All these drivers are available on our organisation [github page](https://github.com/cooper-group-uol-robotics). To clone the currently developed repositories, use:
```
git clone https://github.com/cooper-group-uol-robotics/ika_plate_rct_digital.git
git clone https://github.com/cooper-group-uol-robotics/chemspeed_flex_driver.git
git clone https://github.com/cooper-group-uol-robotics/kmriiwa_chemist_msgs.git
git clone https://github.com/cooper-group-uol-robotics/pi4_peristaltic_pump.git
git clone https://github.com/cooper-group-uol-robotics/fisherbrand_pps4102_balance.git
git clone https://github.com/cooper-group-uol-robotics/mettler_toledo_quantos_q2.git
git clone https://github.com/cooper-group-uol-robotics/kern_pcb_balance.git
```


Note that, other robot/station handlers that don't use ROS1 can be also added and utilised in the ARChemist application as long as they allow the execution of their respective station/robot operations and retrieving their data.
