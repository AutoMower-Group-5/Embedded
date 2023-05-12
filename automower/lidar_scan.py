from rplidar import RPLidar
import time
import threading
import serial
import queue
import importlib
import requests
import pi_camera

#time.sleep(10)

bt_script = importlib.import_module("bt_com")

LIDAR_MIN_ANGLE = 170
LIDAR_MAX_ANGLE = 190
bt_msg = "M:A"
data_send = "POS\n"
data_angle = 0

def lidar_scan_data(stop_flag, scan_queue):
    print("Scanning data...")
    try:
        for scan_data in lidar.iter_scans():
            #print(scan_data)
            scan_queue.put(scan_data)

            # Check if the stop flag is set
            if stop_flag.is_set():
                break
    except Exception as e:
        print(f"Error during scanning: {e}")
 
def process_scan_data(stop_flag, scan_queue, stop_pos, resume_pos):
    global LIDAR_MIN_ANGLE
    global LIDAR_MAX_ANGLE
    global bt_msg
    
    while not stop_flag.is_set():
        try:
            scan_data = scan_queue.get(block=True)
            lidar_min_dist = 250  # Min distance for the lidar to detect an object in mm
            # Process the scan data here
            for point in scan_data:
                quality = int(point[0])
                angle = int(point[1])
                distance = point[2]
                #check_collision(quality, angle, distance)
                #print(quality, angle, distance)
                if (quality > 13 and
                    LIDAR_MIN_ANGLE <= angle <= LIDAR_MAX_ANGLE
                    and distance < lidar_min_dist and bt_msg == "M:A"):
                    write_to_arduino(angle, scan_queue, stop_pos, resume_pos)
                    break
            
        except queue.Empty:
            pass

def write_to_arduino(angle, scan_queue, stop_pos, resume_pos): 
    global LIDAR_MIN_ANGLE
    global LIDAR_MAX_ANGLE
    global data_send
    global data_angle
    
    # Object detected, sending stop command to the arduino
    data_send = "L:DET\n"
    data_angle = angle
    data_scan_queue = scan_queue
    
                    
def run_bt_com():
    bt_script.run_bt_com()
    time.sleep(0.1)
    
def write_pos(stop_pos, resume_pos):
    global LIDAR_MIN_ANGLE
    global LIDAR_MAX_ANGLE
    global data_send
    global data_angle
    
    while True:
        data_received = ser.readline().decode().strip('\n')
        print(data_received)
        if data_received == "BOOTED":
            ser.write("B\n".encode())
            time.sleep(1)
            break
    
    while True:   
        ser.write(data_send.encode())
        print("Sending:", data_send)
        
        #ser.reset_input_buffer()
        data_received = ser.readline().decode().strip('\n')
        print("Receiving 1:", data_received)
        
        if data_received == "A:STOP":
            while True:
                new_angle = str(abs(data_angle - 180)).zfill(3)
                
                # Send coords
                if data_angle < LIDAR_MAX_ANGLE:
                    time.sleep(1)
                    data_send = f"L:RHT:{new_angle}\n"
                    angle = 0
                elif data_angle > LIDAR_MIN_ANGLE:
                    time.sleep(1)
                    data_send = f"L:LFT:{new_angle}\n"
                    angle = 0
                
                # Get mower position
                if ":".join(data_received.split(":")[:2]) == "A:POS":
                    x_pos = data_received.split(":")[2]
                    y_pos = data_received.split(":")[3]
                    url = "https://intense-stream-40056.herokuapp.com/collision/post"
                    pos_data = {"xCoordinate": x_pos, "yCoordinate": y_pos}
                    
                    response = requests.post(url, json=pos_data)
                    pi_camera.send_picture()
                    
                    if response == 200:
                        print("OK")
                    else:
                        print("Error:", response.json())
                    
                    data_send = "L:DON\n"
                
                if data_received == "A:OK":
                    data_send = "POS\n"
                    print("DONE")
                    break
                
                print("Sending:", data_send)
                ser.write(data_send.encode())
                data_received = ser.readline().decode().strip('\n')
                print("Receiving 2:", data_received)
                
                with scan_queue.mutex:
                    scan_queue.queue.clear()

        if ":".join(data_received.split(":")[:2]) == "P:POS":
            x_pos = data_received.split(":")[2]
            y_pos = data_received.split(":")[3]
            z_pos = data_received.split(":")[4]
            
            print("Pos:", x_pos, y_pos, z_pos)
            
            url = "https://intense-stream-40056.herokuapp.com/path/post/withsession"
            pos_data = {"xPath": x_pos, "yPath": y_pos, "angle": z_pos}
            response = requests.post(url, json=pos_data)
            
            if response == 200:
                print("OK")
            else:
                print("Error:", response.json())
                    
            time.sleep(0.5)

            
if __name__ == "__main__":
    serial_port = "/dev/ttyUSB0" # Serial port for the arduino
    serial_baud_rate = 115200  # Baud rate for the arduino
    ser = serial.Serial(serial_port, serial_baud_rate) # Init serial
    lidar_port = "/dev/ttyUSB1"  # Replace with the correct port for your RPLidar
    lidar = RPLidar(lidar_port)
    
    lidar.reset()
    
    stop_flag = threading.Event()
    stop_pos = threading.Event()
    resume_pos = threading.Event()
    scan_queue = queue.Queue()
    
    scan_thread = threading.Thread(target=lidar_scan_data, args=(stop_flag, scan_queue))
    process_thread = threading.Thread(target=process_scan_data, args=(stop_flag, scan_queue, stop_pos, resume_pos))
    bt_thread = threading.Thread(target=run_bt_com)
    pos_thread = threading.Thread(target=write_pos, args=(stop_pos, resume_pos))
 
    scan_thread.start()
    process_thread.start()
    bt_thread.start()
    pos_thread.start()
    
    try:
        while True:
            bt_msg = bt_script.data
            time.sleep(1)
    except KeyboardInterrupt:
        stop_flag.set()
        scan_thread.join()
        process_thread.join()
        bt_thread.join()
        pos_thread.join()
        lidar.stop()
        lidar.disconnect()
        print("Stopped scanning")
