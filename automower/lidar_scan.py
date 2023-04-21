from rplidar import RPLidar
import time
import threading
import serial
import queue

LIDAR_MIN_ANGLE = 170
LIDAR_MAX_ANGLE = 190

def lidar_scan_data(stop_flag, scan_queue):
    print("Scanning data...")
    try:
        for scan_data in lidar.iter_scans():
            scan_queue.put(scan_data)
 
            # Check if the stop flag is set
            if stop_flag.is_set():
                break
    except Exception as e:
        print(f"Error during scanning: {e}")
 
def process_scan_data(stop_flag, scan_queue):
    global LIDAR_MIN_ANGLE
    global LIDAR_MAX_ANGLE 
    while not stop_flag.is_set():
        try:
            scan_data = scan_queue.get(block=True)
            #lidar_min_angle = 170 # Min angle for the lidar to detect an object
            #lidar_max_angle = 190 # Max angle for the lidar to detect an object
            lidar_min_dist = 200  # Min distance for the lidar to detect an object in mm
            # Process the scan data here
            for point in scan_data:
                quality = int(point[0])
                angle = int(point[1])
                distance = point[2]
                #check_collision(quality, angle, distance)
                print(quality, angle, distance)
                if (quality > 13 and
                    LIDAR_MIN_ANGLE <= angle <= LIDAR_MAX_ANGLE
                    and distance < lidar_min_dist):
                    write_to_arduino(angle, scan_queue)
                    break
            
        except queue.Empty:
            pass

# TODO: Change angle sent to the arduino
# TODO: If received angle from arduino starts with 0, remove it
# TODO: Send position to the backend
# TODO: Clean up the code and bugs
def write_to_arduino(angle, scan_queue):
    global LIDAR_MIN_ANGLE
    global LIDAR_MAX_ANGLE 
    # Object detected, sending stop command to the arduino
    data_send = "L:DET\n"
    ser.write(data_send.encode())
    print("Sending:", data_send)
    
    data_received = ser.readline().decode().strip('\n')
    print("Receiving:", data_received)
    
    if data_received == "A:STOP":
        while True:
            new_angle = str(abs(angle - 180)).zfill(3)
            
            # Send coords
            if angle < LIDAR_MAX_ANGLE:
                time.sleep(1)
                data_send = f"L:RHT:{new_angle}\n"
                angle = 0
            elif angle > LIDAR_MIN_ANGLE:
                time.sleep(1)
                data_send = f"L:LFT:{new_angle}\n"
                angle = 0
            
            
            # Get mower position
            if ":".join(data_received.split(":")[:2]) == "A:POS":
                x_pos = data_received.split(":")[2]
                y_pos = data_received.split(":")[3]
                print("Send mower position to the backend")
                print("x_pos", x_pos)
                print("y_pos", y_pos)
                data_send = "L:DON\n"
            
            if data_received == "A:OK":
                print("DONE")
                break
            
            print("Sending:", data_send)
            ser.write(data_send.encode())
            data_received = ser.readline().decode().strip('\n')
            print("Receiving:", data_received)
            
            with scan_queue.mutex:
                scan_queue.queue.clear()
 
if __name__ == "__main__":
    serial_port = "/dev/ttyUSB1" # Serial port for the arduino
    serial_baud_rate = 9600  # Baud rate for the arduino
    ser = serial.Serial(serial_port, serial_baud_rate) # Init serial
    
    lidar_port = "/dev/ttyUSB0"  # Replace with the correct port for your RPLidar
    lidar = RPLidar(lidar_port)
 
    stop_flag = threading.Event()
    scan_queue = queue.Queue()
 
    scan_thread = threading.Thread(target=lidar_scan_data, args=(stop_flag, scan_queue))
    process_thread = threading.Thread(target=process_scan_data, args=(stop_flag, scan_queue))
 
    scan_thread.start()
    process_thread.start()
 
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_flag.set()
        scan_thread.join()
        process_thread.join()
        lidar.stop()
        lidar.disconnect()
        print("Stopped scanning")
