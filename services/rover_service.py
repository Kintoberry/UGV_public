import rpyc
from rpyc.utils.server import ThreadedServer
from classes import Rover, MissionManager, MissionBlueprint, QueueManager
from classes.custom_exceptions import ExistingSerialConnectionException, MissionCompleteException, AlreadyInLastMissionItemException, SyncCommandFailedException
from library import auxiliary_functions as aux


class RoverService(rpyc.Service):
    def __init__(self, rover_instance):
        super().__init__()
        self.rover_instance = rover_instance

    def on_connect(self, conn):
        pass
    
    def on_disconnect(self, conn):
        pass
    
    def exposed_initiate_rover(self) -> dict:
        isInitialized = self.rover_instance.initiate()
        if isInitialized:
            return {"status_code": 200, "message": "Rover is initiated."}
        return {"status_code": 400, "message": "Rover has filed to initiate."}
    
    def exposed_connect_to_rover(self) -> dict:
        try:
            rover_serial = aux.connect_to_rover()
            self.rover_instance.set_rover_serial(rover_serial)
            return {"status_code": 200, "message": "Connected to the rover."}
        except ExistingSerialConnectionException as e:
            return {"status_code": 400, "message": str(e)}  # Bad Request
        except Exception as e:
            return {"status_code": 500, "message": str(e)}  # Internal Server Error
        
    def exposed_conduct_mission(self):
        # if not self.rover_instance.ready_for_mission():
        #     print("why is it here?")
        #     return {"status_code": 400, "message": "You need to connect to the rover, downalod the mission, and initiate the rover."}
        if not self.rover_instance.set_auto_mode():
            return {"status_code": 400, "message": "Failed to set AUTO mode."}
        if not self.rover_instance.arm_rover():
            return {"status_code": 400, "message": "Rover cannot be armed."}
        print("conduct mission success")
        return {"status_code": 200, "message": "The mission has started"}
    
    def exposed_move_to_next_mission_item(self):
        try:
            self.rover_instance.mission_manager.move_to_next_mission_item()
            print("moving to the next mission item succeeded")
            return {"status_code": 200, "message": "The mission has started"}
        except MissionCompleteException as e:
            return {"status_code": 500, "message": str(e)}
        except AlreadyInLastMissionItemException as e:
            return {"status_code": 500, "message": str(e)}
        except MissionCompleteException as e:
            return {"status_code": 500, "message": str(e)}
        except SyncCommandFailedException as e:
            return {"status_code": 500, "message": str(e)}
        
    def exposed_report_mission_status(self):
        pass


def main():
    queue_manager = QueueManager()
    mission_blueprint = MissionBlueprint()
    mission_manager = MissionManager(queue_manager, mission_blueprint)
    rover = Rover(queue_manager, mission_manager)
    
    rover_service = RoverService(rover_instance=rover)
    server = ThreadedServer(service=rover_service, port=20000)
    print("Rover Service has started..")
    server.start()

if __name__ == "__main__":
    main()




