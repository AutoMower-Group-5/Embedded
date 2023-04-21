import requests
import base64

with open("/Users/imsan/Desktop/huracan.jpeg", "rb") as image_file:
    image_data = image_file.read()

encoded_string = base64.b64encode(image_data).decode('utf-8')

# print to check encoded image
# print(encoded_string)


url = "https://intense-stream-40056.herokuapp.com/image/write"
data = {'encodedImg': encoded_string}

# Send a POST request with the URL and data
response = requests.post(url, json=data)

if response.status_code == 200:
    # Print the response content
    print(response.json())
else:
    print("Error:", response.status_code)
