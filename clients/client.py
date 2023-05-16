import requests

def initiate():
    try:
        response = requests.get('http://localhost:50000/initiate-rover', timeout=5)
        response.raise_for_status()
    except requests.exceptions.RequestException as e:
        print(f"Request error: {str(e)}")
    else:
        print(f"Response JSON: {response.json()}")

def connect():
    response = requests.get('http://localhost:50000/connect-to-rover', timeout=5)
    print(f"Response JSON: {response.json()}")

def conduct_mission():
    response = requests.get('http://localhost:50000/conduct-mission', timeout=5)
    print(f"status_code: {response.status_code}")
    print(f"Response JSON: {response.json()}")




if __name__ == "__main__":
    connect()
    initiate()
    conduct_mission()
    # initiate()
    # conduct_mission()