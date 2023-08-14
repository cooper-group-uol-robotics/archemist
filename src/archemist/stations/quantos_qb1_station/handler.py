import rospy
from typing import Tuple, Dict
from archemist.core.state.station import Station
from .state import OpenDoorOpDescriptor, CloseDoorOpDescriptor, DispenseOpDescriptor, MoveCarouselOpDescriptor, UnlockCartridgeOpDescriptor
from roslabware_msgs.msg import MettlerQuantosQB1Cmd, MettlerQuantosQB1Reading
from archemist.core.processing.handler import StationHandler
from threading import Thread



class QuantosSolidDispenserQB1ROSHandler(StationHandler):
    def __init__(self, station: Station):
        super().__init__(station)
        rospy.init_node(f'{self._station}_handler')
        self._quantos_pub = rospy.Publisher("/mettler_quantos_qB1_commands", MettlerQuantosQB1Cmd, queue_size=1)
        rospy.Subscriber('/mettler_quantos_qB1_info', data_class=MettlerQuantosQB1Reading, callback=self._quantos_callback)
        self._received_results = False
        self._op_results = {}
        self._results_wating = None
        
        
        rospy.sleep(1)
        
      
    def run(self):
        rospy.loginfo(f'{self._station}_handler is running')
        try:
            while not rospy.is_shutdown():
                self.handle() #The core handler loop
                rospy.sleep(2)
        except KeyboardInterrupt:
            rospy.loginfo(f'{self._station}_handler is terminating!!!')



    def execute_op(self):
        current_op = self._station.get_assigned_station_op()
        self._received_results = False
        self._op_results = {}
        if isinstance(current_op, DispenseOpDescriptor):
            #Dispense solid
                self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.DISPENSE, quantos_tolerance = current_op.tolerance, quantos_amount= current_op.target_mass,
                                       )
        elif isinstance(current_op, OpenDoorOpDescriptor):
            print("Opening all doors")
            self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.OPEN_ALL_DOORS)
        
        elif isinstance(current_op, CloseDoorOpDescriptor):
            print("Closing all doors")
            for i in range(3):
                self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.CLOSE_ALL_DOORS)
        
        elif isinstance(current_op, MoveCarouselOpDescriptor):
            self._quantos_pub.publish(quantos_command=MettlerQuantosQB1Cmd.MOVE_SAMPLER, quantos_int=current_op.carousel_pos)
        
        elif isinstance(current_op, UnlockCartridgeOpDescriptor):
            self._quantos_pub.publish(quantos_command= MettlerQuantosQB1Cmd.UNLOCK_PIN)

        else:
            rospy.logwarn(f'[{self.__class__.__name__}] Unknown operation was received')
        
        self._results_wating = Thread(target = self._wait_for_ros_message, daemon= True)
        self._results_wating.start()
    
    
    def is_op_execution_complete(self) -> bool:
        

        if self._results_wating.is_alive():
          
            return False
        else:

            rospy.sleep(5) # small wait time just to ensure all is clear

            print("Transitioning")
            
            return True
        

    def get_op_result(self) -> Tuple[bool, Dict]:
        return self._op_results.pop("success", False), self._op_results

    def _wait_for_ros_message(self):
        """Function that waits to recieve a ros message from the Quantos. This is designed to be used in a thread to control the timing of the
            station operations.
        
        """
        
        msg = rospy.wait_for_message('/mettler_quantos_qB1_info', MettlerQuantosQB1Reading, timeout=180)
      
      


    def _quantos_callback(self, msg:MettlerQuantosQB1Reading):
        self._received_results = True 
        
      
        self._op_results["success"] = msg.success
        self._op_results["command"] = msg.command_running
        self._op_results["output"] = msg.output
        self._op_results["front_door_open"] = msg.front_door_open
        self._op_results["side_door_open"] = msg.side_door_open
        self._op_results["autosampler_position"] = msg.sampler_pos
      