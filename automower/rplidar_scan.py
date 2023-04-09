import time
from rplidar import RPLidar

LIDAR_PATH = "/dev/ttyUSB0"
MIN_DISTANCE = 200 # min distance in mm
MAX_BUF_SIZE = 360 * 20

scan_data_buf = []

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
        scan_data_buf.append(point)
        check_collision(angle, distance)
        
    if (len(scan_data_buf) > MAX_BUF_SIZE):
        scan_data_buf.clear()
        
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
        
def main():
    init_lidar()
        
if __name__ == "__main__":
    main()