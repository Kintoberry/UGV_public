from flask import Flask, jsonify
import rpyc

ROVER_HOSTNAME = "localhost"

def setup_rover_connection():
    conn = rpyc.connect(ROVER_HOSTNAME, 20000)
    app.config['rpc_conn'] = conn

app = Flask(__name__)

@app.route('/connect-to-rover', methods=['GET'])
def connect_to_rover():
    try:
        rover_conn = app.config['rpc_conn']
        response = rover_conn.root.connect_to_rover()
        if response["status_code"] == 200:
            return jsonify({"message": response["message"]}, response["status_code"])
        else:
            return jsonify({"error": response["message"]}, response["status_code"])
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route('/initiate-rover', methods=['GET'])
def initiate_rover():
    try:
        rover_conn = app.config['rpc_conn']
        initialized = rover_conn.root.initiate_rover()
        if initialized:
            response = jsonify({"message": "Rover is successfully initialized."})
            response.status_code = 200
            return response
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
@app.route('/conduct-mission', methods=['GET'])
def conduct_mission():
    try:
        rover_conn = app.config['rpc_conn']
        response = rover_conn.root.conduct_mission()
        if response["status_code"] == 200:
            return jsonify({"message": response["message"]}, response["status_code"])
        else:
            return jsonify({"error": response["message"]}, response["status_code"])
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/move-to-next-mission-item', methods=['GET'])
def move_to_next_mission_item():
    try:
        rover_conn = app.config['rpc_conn']
        response = rover_conn.root.move_to_next_mission_item()
        if response["status_code"] == 200:
            return jsonify({"message": response["message"]}, response["status_code"])
        else:
            return jsonify({"error": response["message"]}, response["status_code"])
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/report-mission-status', methods=['GET'])
def report_mission_status():
    try:
        rover_conn = app.config['rpc_conn']
        response = rover_conn.root.report_mission_status()
        if response["status_code"] == 200:
            return jsonify({"message": response["message"]}, response["status_code"])
        else:
            return jsonify({"error": response["message"]}, response["status_code"])
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    pass 

@app.route('/abort-mission', methods=['GET'])
def abort_mission():
    pass 

@app.route('/return-to-launch', methods=['GET'])
def return_to_launch():
    pass 

def main():
    setup_rover_connection()
    app.run(debug=True, host='0.0.0.0', port=50000)

if __name__ == '__main__':
    main()
