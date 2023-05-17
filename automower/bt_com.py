import bluetooth
from bluetooth import BluetoothSocket, RFCOMM
import time
import serial

data = "M:A"

def run_bt_com():
    global data
    
    server_socket = BluetoothSocket(RFCOMM)
    server_socket.bind(("", bluetooth.PORT_ANY))
    server_socket.listen(1)

    port = server_socket.getsockname()[1]
    uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ab"

    arduino_port = "/dev/ttyUSB1"
    baud_rate = 115200

    bluetooth.advertise_service(server_socket, "AutoMower", service_id=uuid, service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS], profiles=[bluetooth.SERIAL_PORT_PROFILE])

    while True:
        print(f"Waiting for connection on RFCOMM channel {port}")
        client_socket, client_info = server_socket.accept()
        print(f"Accepted connection from {client_info}")

        ser = serial.Serial(arduino_port, baud_rate)
        time.sleep(2)

        try:
            while True:
                data_received = client_socket.recv(1024)
                if len(data_received) == 0:
                    break
                
                data_received = data_received.decode().strip()    
                ser.write(data_received.encode())
                
                if data_received == "M:M":
                    data = "M:M"
                if data_received == "M:A":
                    data = "M:A"
                
                print(data_received)

        except IOError:
            pass

        print("Disconnected")
        client_socket.close()

    server_socket.close()