import threading
from pymavlink import mavutil
from library import utility_functions as helper
from library import auxiliary_functions as aux
from workers import workers
from .queue_manager import AbstractQueueManager
from .custom_exceptions import ExistingSerialConnectionException


class Rover:
    def __init__(self, queue_manager: AbstractQueueManager, mission_manager) -> None:
        self.queue_manager = queue_manager
        self.mission_manager = mission_manager
        self.thread_lock = threading.Lock()
        self.rover_serial = None
        self.rover_initiated = False
        self.threads_initiated = False
        self.worker_threads = None
        self.threads_terminate_event = None
        self.rover_armed = False
        self.rover_mode = None

    # Setter Injection 
    def set_rover_serial(self, rover_serial, force=False):
        with self.thread_lock:
            if not self.rover_serial or force:
                self.rover_serial = rover_serial
                self._inject_rover_serial_to_mission_manager()
            else:
                raise ExistingSerialConnectionException("Serial connection to the UGV already exists. Use `force` parameter if you must.")

    def _inject_rover_serial_to_mission_manager(self):
        self.mission_manager.set_rover_serial(self.rover_serial, force=True)

    def initiate(self, reinitiate=False) -> bool:
        # make initial connection here
        with self.thread_lock:
            if reinitiate and self.rover_initiated:
                self._cleanup_resources()
            self.mission_manager.load_mission()
            print("hwy not here?")
            self._initiate_threads()
            print("mission loaded")
            self.rover_initiated = True
        return True
    
    def arm_rover(self) -> bool:
        with self.thread_lock:
            mav_cmd = ( "COMMAND_LONG",
                self.rover_serial.target_system,
                self.rover_serial.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, # Confirmation
                1, # 0:disarm, 1:arm
                21196, # 0: arm-disarm unless prevented by safety checks, 21196: force arming or disarming
                0, # Reserved
                0, # Reserved
                0, # Reserved
                0, # Reserved
                0  # Reserved 
            )
            self.queue_manager.put("sync_cmd", block=True)
            success = self.queue_manager.get("sync_cmd_result")
            if success:
                self.rover_mode = "AUTO"
                print("Rover is armed")
                return True
            else:
                return False
        
    def disarm_rover(self) -> bool:
        with self.thread_lock:
            mav_cmd = ( "COMMAND_LONG",
                self.rover_serial.target_system,
                self.rover_serial.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, # Confirmation
                0, # 0:disarm, 1:arm
                21196, # 0: arm-disarm unless prevented by safety checks, 21196: force arming or disarming
                0, # Reserved
                0, # Reserved
                0, # Reserved
                0, # Reserved
                0  # Reserved 
            )
            self.queue_manager.put("sync_cmd", block=True)
            success = self.queue_manager.get("sync_cmd_result")
            if success:
                self.rover_mode = "AUTO"
                print("Rover is disarmed")
                return True
            else:
                return False
    
    def set_auto_mode(self) -> bool:
        with self.thread_lock:
            AUTO_MODE = 10 # refer to https://ardupilot.org/rover/docs/parameters.html#mode1
            mav_cmd = ( "COMMAND_LONG",
                self.rover_serial.target_system,
                self.rover_serial.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0, # Confirmation (default 0)
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, # 
                AUTO_MODE, 
                0, # Reserved
                0, # Reserved
                0, # Reserved
                0, # Reserved
                0  # Reserved
            )
            self.queue_manager.put("sync_cmd", mav_cmd, block=True)
            success = self.queue_manager.get("sync_cmd_result")
            if success:
                self.rover_mode = "AUTO"
                return True
            else:
                return False

    def ready_for_mission(self) -> bool:
        with self.thread_lock:
            if self.rover_serial and self.threads_initiated and self.mission_manager.is_mission_downloaded():
                return True 
            else:
                print(f"rover_serial: {self.rover_serial}, threads_initiated: {self.threads_initiated}, mission_manager: {self.mission_manager.is_mission_downloaded()}")
                return False
    
    def _initiate_threads(self):
        if self.threads_initiated:
            print("ERROR: worker threads are already initiated.")
            return
        self.worker_threads, self.threads_terminate_event = workers.run(self.rover_serial, self.queue_manager, self.mission_manager)
        for thread in self.worker_threads:
            thread.start()

        self.threads_initiated = True

    def _kill_threads(self):
        self.threads_terminate_event.set()
        while any(thread.is_alive() for thread in self.worker_threads):
            for thread in self.worker_threads:
                thread.join(timeout=1)
    
    # TODO: needs to be implemented
    def _cleanup_resources(self):
        # Call RTL function to put the rover back to the launch point
        if self.threads_initiated:
            self._kill_threads()
            self.threads_initiated = False
        pass
    
