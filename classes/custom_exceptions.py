

class MissionCompleteException(Exception):
    pass

class AlreadyInLastWaypointException(Exception):
    pass

class AlreadyInLastMissionItemException(Exception):
    pass

class MissionAlreadyDownloadedException(Exception):
    pass

class ExistingSerialConnectionException(Exception):
    pass

class SyncCommandFailedException(Exception):
    pass
