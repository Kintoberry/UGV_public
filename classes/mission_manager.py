from classes.mission_blueprint import MissionBlueprint
import threading
from .custom_exceptions import MissionCompleteException, AlreadyInLastWaypointException, AlreadyInLastMissionItemException, SyncCommandFailedException
from .queue_manager import AbstractQueueManager
from .custom_exceptions import ExistingSerialConnectionException
from pymavlink import mavutil

class MissionManager:
    def __init__(self, queue_manager: AbstractQueueManager, mission_blueprint):
        self.rover_serial = None
        self.queue_manager = queue_manager
        self.mission_blueprint = mission_blueprint
        self.thread_lock = threading.Lock()
        self.target_waypoint = None
        self.current_waypoint = None
        self.current_mission_item = None
        self.final_waypoint = None
        self.mission_complete = False
        self.loitering = False
        self.loiter_time = None

    # Setter Injection 
    def set_rover_serial(self, rover_serial, force=False):
        with self.thread_lock:
            self.rover_serial = rover_serial

    def load_mission(self):
        with self.thread_lock:
            self.mission_blueprint.download_mission(self.rover_serial)
            self.current_waypoint = self.mission_blueprint.get_first_waypoint()
            self.final_waypoint = self.mission_blueprint.get_final_Waypoint()
            self.current_mission_item = self.mission_blueprint.get_first_mission_item()

    def is_mission_downloaded(self) -> bool:
        with self.thread_lock:
            return self.mission_blueprint.is_mission_downloaded()
    
    def update_status(self, message):
        with self.thread_lock:
            message_type = message.get_type()
            if message_type == "MISSION_CURRENT":
                if self.mission_blueprint.is_waypoint(message.seq):
                    self.target_waypoint = message.seq
                elif self.mission_blueprint.is_loiter_unlimited_cmd(message.seq):
                    self.current_mission_item = message.seq
                    self.loitering = True 
                    self.loiter_time = -1

            elif message_type == "MISSION_ITEM_RECHEAD":
                if self.mission_blueprint.is_waypoint(message.seq):
                    self.current_waypoint = message.seq
                    if message.seq == self.final_waypoint:
                        self.target_waypoint = self.final_waypoint
                        self.mission_complete = True
                elif self.mission_blueprint.is_loiter_unlimited_cmd(message.seq):
                    self.loitering = False 
                    self.loiter_time = None
                
    def move_to_next_waypoint(self) -> bool:
        with self.thread_lock:
            if self.mission_blueprint.is_mission_complete(self.current_waypoint):
                raise MissionCompleteException("Cannot move to the next waypoint. Mission already complete.")
            next_waypoint = self.mission_blueprint.get_next_waypoint(self.current_waypoint)
            if next_waypoint == self.mission_blueprint.get_final_waypoint():
                raise AlreadyInLastWaypointException("There is no next waypoint.")
            mav_cmd = ("COMMAND_LONG",
                self.rover_serial.target_system,
                self.rover_serial.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                0,
                next_waypoint,
                0,
                0, 0, 0, 0, 0
             )
            self.queue_manager.put("sync_cmd", mav_cmd)
            success = self.queue_manager.get("sync_cmd_result")
            if success:
                self.current_waypoint = next_waypoint
            else:
                raise SyncCommandFailedException("Couldn't send sync command MAV_CMD_DO_SET_MISSION_CURRENT")

            self.current_waypoint = next_waypoint
        

    def move_to_next_mission_item(self):
        with self.thread_lock:
            if self.mission_blueprint.is_mission_complete(self.current_waypoint):
                raise MissionCompleteException("Cannot move to the next waypoint. Mission already complete.")
            next_mission_item = self.mission_blueprint.get_next_mission_item(self.current_mission_item)
            if next_mission_item == self.mission_blueprint.get_final_mission_item():
                raise AlreadyInLastMissionItemException("There is no next mission item.")
            mav_cmd = ("COMMAND_LONG",
                self.rover_serial.target_system,
                self.rover_serial.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                0,
                next_mission_item,
                0,
                0, 0, 0, 0, 0
             )
            self.queue_manager.put("sync_cmd", mav_cmd)
            success = self.queue_manager.get("sync_cmd_result")
            if success:
                self.current_mission_item = next_mission_item
            else:
                raise SyncCommandFailedException("Couldn't send sync command MAV_CMD_DO_SET_MISSION_CURRENT")
            