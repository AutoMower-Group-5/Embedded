import time
from rplidar import RPLidar
import asyncio
import websockets
import json
import threading

LIDAR_PATH = "/dev/ttyUSB0"
MIN_DISTANCE = 200 # min distance in mm
MAX_BUF_SIZE = 360 * 20
SERVER_IP = f"ws://192.168.0.207:8765"

scan_data_buf = []

async def websocket_client():
    global scan_data_buf
    uri = SERVER_IP
    async with websockets.connect(uri) as websocket:
        while True:
            if (len(scan_data_buf) >= MAX_BUF_SIZE):
                data = json.dumps(scan_data_buf)
                await websocket.send(data)
                print(f"Data sent: {data}")
                scan_data_buf.clear()
                
                response = await websocket.recv()
                print(f"Recv data: {response}")
        
            await asyncio.sleep(0.1)

def check_collision(angle, distance):
    if (distance < MIN_DISTANCE):
        print(f"Angle: {angle}. Distance {distance}, CLOSE!") # Rotate the mower to a different angle before moving
    else:
        print(f"Angle: {angle}. Distance {distance}") # Mower is in the clear

def process_scan(scan_data):
    print("Lidar Data (angles and distances)")
    
    for point in scan_data:
        angle = int(point[1])
        distance = point[2]
        scan_data_buf.append(point[1:3])
        check_collision(angle, distance)
        
def init_lidar():
    lidar = RPLidar(LIDAR_PATH)
    
    try:
        print("Starting RPLidar...")
        lidar.connect()
        time.sleep(1)
        lidar.start_motor()
        time.sleep(1)
        
        print("Scanning data...")
        for scan_data in lidar.iter_scans():
            process_scan(scan_data)
            time.sleep(0.1)
        
    except KeyboardInterrupt:
        print("Stopping RPLidar...")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        
def run_websocket_client():
    asyncio.run(websocket_client())

def main():
    client_thread = threading.Thread(target=run_websocket_client)
    client_thread.start()
    init_lidar()
    
        
if __name__ == "__main__":
    main()
