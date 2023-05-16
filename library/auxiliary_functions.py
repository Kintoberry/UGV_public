from pymavlink import mavutil
import serial.tools.list_ports
import sys


def connect_to_rover():
    portname, baud_rate = find_port_name()
    if portname is None:
        print("ERROR: Cannot find the port for the telemetry radio.")
        return False
    
    rover_serial = connect_flight_controller(portname, baud_rate)
    if rover_serial is None:
        print("ERROR: connect_flight_controller func returned None")
        return False

    return rover_serial

def connect_flight_controller(connection_string, baud_rate, debug=False):
    try:
        return mavutil.mavlink_connection(connection_string, baud=baud_rate)
    except Exception as e:
            print("e: ", e)
            return None 

        
def find_port_name(debug=False) -> tuple[str, int]:
    # NOTE: it seems `vid` and `pid` and etc might changes depending on which OS you run this code. 
    # It might be due to firmware-level discrepancies. 
    telemetry_radio_identity = None
    pixhawk4_identity = None
    if sys.platform == "linux":
        # port: /dev/ttyUSB0 - FT230X Basic UART - FT230X Basic UART, vid: 1027, pid: 24597, serial_number: D308LTJD, port.device: /dev/ttyUSB0
        # NOTE: use hexadecimal value. For example, vid = 0x0403 is 1027 in decimal
        telemetry_radio_identity = {
            "vid": 0x0403,
            "pid": 0x6015,
            "serial_number": "D308LTJD"
        }
        # this is only checked on Linux system
        pixhawk4_identity = {
            "vid": 0x1209,
            "pid": 0x5740,
            "serial_number": "29003F001951383433393139"
        }
    elif sys.platform == "win32":
         # Telemetry hardware on Windows - VID:PID=0403:6015", "SER=D308LTJDA
         # So the last character 'A' is added to the `SER` compared to on Linux system
        telemetry_radio_identity = {
            "vid": 0x0403,
            "pid": 0x6015,
            "serial_number": "D308LTJDA"
        }
    
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if debug:
            # example output for Pixhawk4 serial connection: port: /dev/ttyACM0 - Pixhawk4, vid: 4617, pid: 22336, serial_number: 29003F001951383433393139, port.device: /dev/ttyACM0, port.hwid: USB VID:PID=1209:5740 SER=29003F001951383433393139 LOCATION=2-2:1.0
            print(f"port: {port}, vid: {port.vid}, pid: {port.pid}, serial_number: {port.serial_number}, port.device: {port.device}, port.hwid: {port.hwid}")
        
        if all(getattr(port, attr) == pixhawk4_identity[attr] for attr in pixhawk4_identity):
            """ 
                NOTE: we're returning the first device file string. But you should know that there can be
                multiple device file related to one serial port connection.

                For example, both /dev/ttyACM0 and /dev/ttyACM1 are created for a serial connection with Pixhawk4.
                If you have to choose the right file among the two, you should consult ardupilot documentation.
                One might be for MAVLink, while the other might be for logging and etc.
            """
            if debug:
                print("Pixhawk4 serial port")
            return port.device, 115200
        elif all(getattr(port, attr) == telemetry_radio_identity[attr] for attr in telemetry_radio_identity):
            if debug:
                print("Telemetry radio serial port")
            return port.device, 57600
    if debug:
        print("ERROR: Couldn't connect to the rover using a serial port.")
    return None, None

if __name__ == "__main__":
    portname, baud = find_port_name(debug=True)
    print("portname: ", portname)

