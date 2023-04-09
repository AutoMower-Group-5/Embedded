import bluetooth
from bluetooth import BluetoothSocket, RFCOMM
import time

server_socket = BluetoothSocket(RFCOMM)
server_socket.bind(("", bluetooth.PORT_ANY))
server_socket.listen(1)

port = server_socket.getsockname()[1]
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ab"

bluetooth.advertise_service(server_socket, "AutoMower", service_id=uuid, service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS], profiles=[bluetooth.SERIAL_PORT_PROFILE])

print(f"Waiting for connection on RFCOMM channel {port}")

client_socket, client_info = server_socket.accept()
print(f"Accepted connection from {client_info}")

try:
    while True:
        data = client_socket.recv(1024)
        if len(data) == 0:
            break

        print(f"Received: {data.decode('utf-8')}")
        client_socket.send(data)

except IOError:
    pass

print("Disconnected")

client_socket.close()
server_socket.close()
