from pymavlink import mavutil
# from library import auxiliary_functions as aux
import sys, os
from .custom_exceptions import MissionAlreadyDownloadedException

class MissionBlueprint:
    def __init__(self):
        self.mission_items = []
        self.waypoints = None
        self.mission_count = None
        self.mission_downloaded = False

    def _add_mission_item(self, item):
        self.mission_items.append(item)

    def _save_waypoints(self):
        self.waypoints = sorted([item for item in self.mission_items if item.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT], key=lambda wp: wp.seq)
    
    def _sort_mission_items(self):
        sorted(self.mission_items, key=lambda item: item.seq)
    

    def is_waypoint(self, seq) -> bool:
        for waypoint in self.waypoints:
            if waypoint.seq == seq:
                return True
        return False
    
    def is_loiter_unlimited_cmd(self, seq) -> bool:
        for item in self.mission_items:
            if item.seq == seq:
                if item.command == mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM:
                    return True
        return False

    def get_next_waypoint(self, seq) -> int:
        for waypoint in self.waypoints:
            if waypoint.seq > seq:
                return waypoint.seq
        return self.get_final_waypoint()
    
    def get_next_mission_item(self, seq) -> int:
        for item in self.mission_items:
            if item.seq > seq:
                return item.seq
        return self.get_final_mission_item()
    
    def get_first_waypoint(self) -> int:
        if len(self.waypoints) > 0:
            return self.waypoints[0].seq
        else:
            raise IndexError("The waypoints list is empty, no first waypoint available.")
    
    def get_first_mission_item(self) -> int:
        if len(self.mission_items) > 0:
            return self.mission_items[0].seq
        else:
            raise IndexError("The waypoints list is empty, no first waypoint available.")
        
    def get_final_waypoint(self) -> int:
        if len(self.waypoints) > 0:
            return self.waypoints[-1].seq
        else:
            raise IndexError("The waypoints list is empty, no final waypoint available.")
    
    def get_final_mission_item(self) -> int:
        if len(self.mission_items) > 0:
            return self.waypoints[-1].seq
        else:
            raise IndexError("The waypoints list is empty, no final waypoint available.")

    def download_mission(self, rover_serial, force=False):
        if not force and self.mission_downloaded:
            raise MissionAlreadyDownloadedException("Missions is already downloaded. Use `force` parameter for re-download.")
        
        rover_serial.mav.mission_request_list_send(rover_serial.target_system, rover_serial.target_component)
        self.mission_count = None

        while self.mission_count is None:
            msg = rover_serial.recv_match(type="MISSION_COUNT", blocking=True)
            self.mission_count = msg.count

        for i in range(self.mission_count):
            rover_serial.mav.mission_request_int_send(rover_serial.target_system, rover_serial.target_component, i)
            msg = rover_serial.recv_match(type="MISSION_ITEM_INT", blocking=True)  # Use MISSION_ITEM_INT instead of MISSION_ITEM
            mission_blueprint._add_mission_item(msg)

        rover_serial.mav.mission_ack_send(rover_serial.target_system, rover_serial.target_component, mavutil.mavlink.MAV_MISSION_ACCEPTED)
        self._save_waypoints()
        self.mission_downloaded = True

    def is_mission_downloaded(self) -> bool:
        return self.mission_downloaded

    def get_number_of_waypoints(self):
        return len(self.waypoints)
    
    def is_mission_complete(self, current_sequence):
        return current_sequence >= len(self.mission_items)
    

if __name__ == "__main__":
    # Add the project root directory to sys.path
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sys.path.append(project_root)
    from library import auxiliary_functions as aux
    from library import utility_functions as helper

    portname, baud_rate = aux.find_port_name()
    if portname is None:
        print("ERROR: Cannot find the port for the telemetry radio.")
        exit()
    
    rover_serial = helper.connect_flight_controller(portname, baud_rate)
    if rover_serial is None:
        print("ERROR: connect_flight_controller func returned None")
        exit()

    rover_serial.wait_heartbeat()

    mission_blueprint = MissionBlueprint(rover_serial)
    mission_blueprint.download_mission()

    print("waypoints number: ", mission_blueprint.get_number_of_waypoints())
    print(mission_blueprint.get_waypoints_summary())

