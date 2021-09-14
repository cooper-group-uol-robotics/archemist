from archemist.exceptions.exception import Error
from archemist.state.state import State
from archemist.persistence.persistenceManager import persistenceManager, Parser
from archemist.state.station import Station
from archemist.state.robot import Robot, VialMoveOpDescriptor, TransportBatchOpDescriptor, RackPlaceOpDescriptor
from archemist.state.batch import Batch
from archemist.state.robots.pandaFranka import PandaFranka, PandaMoveOpDescriptor
from archemist.state.robots.kukaLBRIIWA import KukaLBRIIWA, KukaMoveBaseOpDescriptor, KukaPickOpDescriptor, KukaRackPlaceOpDescriptor,KukaVialMoveOpDescriptor, RackPickOpDescriptor
from archemist.util.location import Location
from archemist.util.sm import States
import rospy
import roslaunch
from time import sleep


class WorkflowManager:
    def __init__(self):
        self._persistanceManager = persistenceManager()
        self._state = None
        
        self._robot_batch_queue = []
        self._unassigned_batches = []
        self._completed_batches = []

    def initializeWorkflow(self):
        self._state = State()
        self._state.initializeState(reset_db=True)
        kuka1 = self._state.getRobot('KukaLBRIIWA',1) 
        kuka1.location = Location(5,7,'drive_frame')
        self._state.modifyObjectDB(kuka1)
        panda = self._state.getRobot('PandaFranka',1)
        panda.location = Location(8,7,'neutral')
        self._state.modifyObjectDB(panda)
        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()
        # for station in self._state.stations:
        #     stationHandlerName = station.__class__.__name__ + "_Handler"
        #     try:
        #         node = roslaunch.core.Node('archemist', stationHandlerName)
        #         process = launch.launch(node)
        #     except:
        #         print("Couldn't launch node: " + stationHandlerName)

    def createBatch(self, batch_id, rows, cols, location):
        parser = Parser()
        self._unassigned_batches.append(Batch(batch_id, parser.loadRecipeYaml(), rows, cols, location))

    def process(self):
        while (True):
            self._state.updateFromDB()
            while self._unassigned_batches: # single vial mode
                batch = self._unassigned_batches.pop()
                current_station_name = batch.getCurrentStation().station
                current_station = self._state.getStation(current_station_name)
                #print('processing batch at ' + str(batch.location) + ' heading to ' + next_station_name) 
                sample = batch.getCurrentSample()
                if (current_station_name == 'InputStation'):
                    robotOp = RackPickOpDescriptor('', current_station.rack_holder_loc, None)
                    print('Picking rack from {0}'.format(str(robotOp.start_pos)))
                    batchOp_tup = (batch,robotOp)
                    self._robot_batch_queue.append(batchOp_tup)
                elif batch.processSM.state == States.TRANSPORTING_RACK:
                    print('inside ' + str(batch.processSM.state))
                    if sample.location.get_map_coordinates() !=  current_station.load_loc.get_map_coordinates():
                        print('navigating '+ str(current_station.load_loc))
                        robotOp = TransportBatchOpDescriptor('',current_station.load_loc,None)
                        batchOp_tup = (batch,robotOp)
                        self._robot_batch_queue.append(batchOp_tup)
                    else:
                        batch.advanceProcessState()
                elif batch.processSM.state == States.PLACING_RACK:
                    print('inside ' + str(batch.processSM.state))
                    if current_station.rack_holder_loc is not None:
                        if batch.location != current_station.rack_holder_loc:
                            robotOp = RackPlaceOpDescriptor('', current_station.rack_holder_loc, None)
                            print('Placing rack to {0}'.format(str(robotOp.end_pos)))
                            batchOp_tup = (batch,robotOp)
                            self._robot_batch_queue.append(batchOp_tup)
                        else:
                            batch.advanceProcessState()
                    else:
                         batch.advanceProcessState()
                elif batch.processSM.state == States.PRE_LOAD_VIAL:
                    print('inside ' + str(batch.processSM.state))
                    if current_station.pre_load_loc is not None:
                        if sample.location != current_station.pre_load_loc:
                            robotOp = VialMoveOpDescriptor('',sample.location, current_station.pre_load_loc, None)
                            print('moving sample from {0} to pre load {1}'.format(robotOp.start_pos,robotOp.end_pos))
                            batchOp_tup = (batch,robotOp)
                            self._robot_batch_queue.append(batchOp_tup)
                        else:
                             batch.advanceProcessState()
                    else:
                        batch.advanceProcessState()
                elif batch.processSM.state == States.LOAD_VIAL:
                    print('inside ' + str(batch.processSM.state))
                    if sample.location != current_station.load_loc:
                        robotOp = VialMoveOpDescriptor('',sample.location, current_station.load_loc, None)
                        print('moving sample from {0} to load {1}'.format(robotOp.start_pos,robotOp.end_pos))
                        batchOp_tup = (batch,robotOp)
                        self._robot_batch_queue.append(batchOp_tup)
                    else:
                        batch.advanceProcessState()
                elif batch.processSM.state == States.PROCESS_VIAL:
                    print('inside ' + str(batch.processSM.state))
                    print('processing sample at {0}'.format(current_station.__class__.__name__))
                    statinoOp = batch.recipe.getCurrentTaskOp()
                    batch.addOpeationOp(statinoOp)
                    current_station.add_batch(batch)
                    batch.assigned = True
                    self._state.modifyObjectDB(current_station)
                elif batch.processSM.state == States.POST_LOAD_VIAL:
                    print('inside ' + str(batch.processSM.state))
                    if current_station.post_load_loc is not None:
                        if sample.location != current_station.post_load_loc:
                            robotOp = VialMoveOpDescriptor('',sample.location, current_station.post_load_loc, None)
                            print('moving sample from {0} to post load {1}'.format(robotOp.start_pos,robotOp.end_pos))
                            batchOp_tup = (batch,robotOp)
                            self._robot_batch_queue.append(batchOp_tup)
                        else:
                            batch.advanceProcessState()
                    else:
                        batch.advanceProcessState()
                elif batch.processSM.state == States.RETURN_VIAL:
                    print('inside ' + str(batch.processSM.state))
                    upcoming_station_name = batch.getUpcomingStation().station
                    upcoming_station = self._state.getStation(upcoming_station_name)
                    if sample.location.get_map_coordinates() !=  upcoming_station.load_loc.get_map_coordinates():
                        if sample.location != batch.location:
                            robotOp = VialMoveOpDescriptor('',sample.location, batch.location, None)
                            print('moving sample from {0} to rack {1}'.format(robotOp.start_pos,robotOp.end_pos))
                            batchOp_tup = (batch,robotOp)
                            self._robot_batch_queue.append(batchOp_tup)
                        else:
                            batch.advanceProcessState()
                    else:
                        batch.advanceProcessState()
                elif batch.processSM.state == States.PICKING_RACK:
                    print('inside ' + str(batch.processSM.state))
                    upcoming_station_name = batch.getUpcomingStation().station
                    upcoming_station = self._state.getStation(upcoming_station_name)
                    if batch.location.get_map_coordinates() !=  upcoming_station.load_loc.get_map_coordinates():
                        if current_station.rack_holder_loc is not None:
                            robotOp = RackPickOpDescriptor('', batch.location, None)
                            print('Picking rack from {0}'.format(str(robotOp.start_pos)))
                            batchOp_tup = (batch,robotOp)
                            self._robot_batch_queue.append(batchOp_tup)
                        else:
                            batch.advanceProcessState()
                    else:
                        batch.advanceProcessState()
                elif batch.processSM.state == States.RESET:
                    print('inside ' + str(batch.processSM.state))
                    batch.advanceRecipeState(True)   
                    batch.resetProcessState()
                
                self.scheduleRobotJobs()
            if batch.complete():
                self._completed_batches.append(batch)
            elif not batch.assigned:
                self._unassigned_batches.append(batch)
            robot_batches = self.getCompletedRobotObjects()
            self._unassigned_batches.extend(robot_batches)
            station_batches = self.getCompleteStationBatches()
            self._unassigned_batches.extend(station_batches)
            sleep(1)

    def getCompletedRobotObjects(self):
        processed_batches = list()
        for robot in self._state.robots:
            if robot.has_processed_batch():
                unassigned_batch = robot.get_processed_batch()
                unassigned_batch.assigned = False
                processed_batches.append(unassigned_batch)
                self._state.modifyObjectDB(robot)
        return processed_batches

    def getCompleteStationBatches(self):
        processed_batches = list()
        for station in self._state.stations:
            if station.has_processed_batch():
                unassigned_batch = station.get_processed_batch()
                unassigned_batch.assigned = False
                processed_batches.append(unassigned_batch)
                self._state.modifyObjectDB(station)
        return processed_batches

    def scheduleRobotJobs(self):
        while self._robot_batch_queue:
            batchOp_tup = self._robot_batch_queue.pop()
            batch = batchOp_tup[0]
            robotOp = batchOp_tup[1]
            if isinstance(robotOp, TransportBatchOpDescriptor):
                robot = self._state.getRobot('KukaLBRIIWA',1)
                robotOp = KukaMoveBaseOpDescriptor(robot.id, robotOp.target_loc, True)
                batch.addOpeationOp(robotOp)
                self._commitBatch(robot, batch)
               # print('commanding robot {0} with command {1}'.format(robot.__class__.__name__,robotOp.__class__.__name__))
            elif isinstance(robotOp, RackPickOpDescriptor):
                robot = self._state.getRobot('KukaLBRIIWA',1) # here you can select a robot
                robotOp = KukaPickOpDescriptor(robotOp.start_pos)
                batch.addOpeationOp(robotOp)
                # if (robot.location != robotOp.start_pos):
                #     navRobotOp = KukaMoveBaseOpDescriptor(robot.id, robotOp.start_pos, True)
                #     batch.addOpeationOp(navRobotOp)
                self._commitBatch(robot, batch)
               # print('commanding robot {0} with command {1}'.format(robot.__class__.__name__,robotOp.__class__.__name__))
            elif isinstance(robotOp, RackPlaceOpDescriptor):
                robot = self._state.getRobot('KukaLBRIIWA',1)
                robotOp = KukaRackPlaceOpDescriptor(robotOp.end_pos)
                batch.addOpeationOp(robotOp)
                # if (robot.location != robotOp.end_pos):
                #     navRobotOp = KukaMoveBaseOpDescriptor(robot.id, robotOp.end_pos, True)
                #     batch.addOpeationOp(navRobotOp)
                self._commitBatch(robot, batch)
               # print('commanding robot {0} with command {1}'.format(robot.__class__.__name__,robotOp.__class__.__name__))
            elif isinstance(robotOp,VialMoveOpDescriptor):
                kuka_hard_frames = ['pump', 'ika', 'camera']
                panda_hard_frame = ['rack_robot']
                for robot in self._state.robots:
                    if robot.location.get_map_coordinates() == robotOp.start_pos.get_map_coordinates():
                        if isinstance(robot, KukaLBRIIWA):
                            if all((frame != robotOp.end_pos.frame_name and frame != robotOp.start_pos.frame_name) for frame in kuka_hard_frames):
                                robotOp = KukaVialMoveOpDescriptor(robotOp.start_pos, robotOp.end_pos)
                                batch.addOpeationOp(robotOp)
                                self._commitBatch(robot, batch)
                                print('commanding robot {0} with command {1}'.format(robot.__class__.__name__,robotOp.__class__.__name__))
                                return
                        elif isinstance(robot, PandaFranka):
                            if all((frame != robotOp.end_pos.frame_name and frame != robotOp.start_pos.frame_name) for frame in panda_hard_frame):
                                robotOp = PandaMoveOpDescriptor(robotOp.start_pos, robotOp.end_pos)
                                batch.addOpeationOp(robotOp)
                                self._commitBatch(robot, batch)
                                print('commanding robot {0} with command {1}'.format(robot.__class__.__name__,robotOp.__class__.__name__))
                                return
                # no robots nearby
                if all((frame != robotOp.end_pos.frame_name and frame != robotOp.start_pos.frame_name) for frame in kuka_hard_frames):
                                robot = self._state.getRobot('KukaLBRIIWA',1)
                                robotOp = KukaVialMoveOpDescriptor(robotOp.start_pos, robotOp.end_pos)
                                batch.addOpeationOp(robotOp)
                                self._commitBatch(robot, batch)


    def _commitBatch(self, robot, batch):
        batch.assigned = True
        robot.add_batch(batch)
        self._state.modifyObjectDB(robot)


