import base64
from picamera import PiCamera
from datetime import datetime
import requests

camera = PiCamera()
camera.resolution = (1920, 1080)

def get_date():
    current_datetime = datetime.now()
    date_str = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
    return date_str
 
 
def capture_picture():
    date = get_date()
    path = f"images/{date}.jpg"
    # Save the captured image
    camera.capture(path)
    print(path)
 
    return path
 
 
def send_picture():
    path = capture_picture()
 
    with open(f"{path}", "rb") as image_file:
        image_data = image_file.read()
 
    encoded_string = base64.b64encode(image_data).decode('utf-8')
 
    # print to check encoded image
    # print(encoded_string)
 
    url = "https://intense-stream-40056.herokuapp.com/image/session/post"
    data = {'encodedImg': encoded_string}
    
    # Send a POST request with the URL and data
    response = requests.post(url, json=data)
 
    if response.status_code == 200:
        # Print the response content
        print(response.json())
    else:
        print("Error:", response.status_code)
 
