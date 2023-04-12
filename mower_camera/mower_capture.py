from picamera import PiCamera
from time import sleep
from datetime import datetime
import keyboard


camera = PiCamera()
camera.resolution = (1920, 1080)


def get_date():
    current_datetime = datetime.now()
    date_str = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
    return date_str

def choose_from_list():
    options ="""
    choose a number from list blow:
    1. Run the camera in X seconds and capture pitures every Y seconds
    2. Run the camera until the program is intruppted and capture pictures in Y seconds
    3. Use key 'S' to start camera and 'E' to end it and capture pictures in Y seconds 
    """
    ans = int(input(options))
    
    if ans == 1:
        seconds = int(input("Insert the seconds: "))
        sleep_for_sec = int(input("For each seconds do you want to capture pictures: "))
        for x in range(seconds):
            capture_picture()
            sleep(sleep_for_sec)
        
        
    elif ans == 2:
        sleep_for_sec = int(input("For each seconds do you want to capture pictures: "))
        while True:
            #capture_picture()
            capture_picture()
            sleep(sleep_for_sec)
            
        
    elif ans == 3:
        sleep_for_sec = int(input("For each seconds do you want to capture pictures: "))
        key = input("Insert 's' to start and 'e' to end the program: ")
        run = 1
        
        while True:
            if key == 's':
                keyboard.wait('s')  # Wait for the 's' key to be pressed
                while run:
                    capture_picture()
                    sleep(sleep_for_sec)
                    if keyboard.is_pressed("e"):
                        run = 0 
            elif key == 'e':
                run = 0
            else:
                print("The insert key is not defined: Insert again: ")
                key = input("Insert 's' to start and 'e' to end the program: ")  

def capture_picture():
    date = get_date()
    camera.capture(f'/home/imsan00/Desktop/mover_capture_images/captured_images{date}.jpg')
    


if __name__ == '__main__':
    choose_from_list()
   