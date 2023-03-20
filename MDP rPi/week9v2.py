import serial
import time
import imagezmq
import socket
import requests
import json
from picamera2 import Picamera2, Preview
from libcamera import Transform

# Libraries
import RPi.GPIO as GPIO
import time

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Open both Serial ports of Tablet and STM32
try:
    TABLET_SER = serial.Serial("/dev/rfcomm0", baudrate=9600, timeout=1)
except serial.SerialException:
    print("Please connect the android tablet to RPi")
    exit(0)
try:
    STM_SER = serial.Serial(
        "/dev/ttyUSB0",
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=2,
    )
except serial.SerialException:
    print("Please connect the RPi to the STM")
    exit(0)

# Globals

START = False
TASK_FINISH = False
CV_RESPONSE = None


""" CV """
CV_ADDRESS = 'http://192.168.1.21:5000/get-confidence'
CAMERA_SETUP_TIME = 2

""" ULTRASONIC """
STOP_DISTANCE = 70
DISTANCE_TRAVELED = 0
ULTRASOUND_DISTANCE = 1000

""" pICAM """
# Configure picam
picam2 = Picamera2()
config = picam2.create_still_configuration(
        transform=Transform(hflip=True, vflip=True),main={"size":(640,480)})
picam2.configure(config)
picam2.start()


# CV CAM FUNCTION
def capture_and_send():
    try:
        nparray = picam2.capture_array()
        response = requests.post(url = CV_ADDRESS, data = nparray.tobytes())
        return response.text
    except Exception as e:
        print (e.__class__.__name__)


# ================================================
# Code for Ultrasonic
def get_distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance


# Generic send functions
def encode_to_tablet(message):
    print(f"Sent to tablet: {message}")
    TABLET_SER.write(message.encode())


def encode_to_stm(message):
    time.sleep(0.05)
    STM_SER.write(message.encode())
    print(f"Attempted to add to STM:{message}")
    print(f"{message.encode()}")

    # Wait for an acknowledgment from the STM32
    while True:
        response = STM_SER.readline().decode().strip()
        print(f"Received from STM: {response}")  # Add this line for debugging
        if response == "ACK":
            break
        else:
            print("Waiting for ACK...")  # Add this line for debugging
            time.sleep(0.1)


def movement_task():
    global START, DISTANCE_TRAVELED, ULTRASOUND_DISTANCE, CV_RESPONSE, TASK_FINISH

    while not START:
        tablet_data = TABLET_SER.readline().decode().strip()
        if tablet_data:
            print("========== RECEIVED START FROM ANDROID TABLET ==========")
            START = True


    while START and not TASK_FINISH:
        print("========== MOVEMENT TASK STARTED ==========")
        approach_obstacle_and_advance(25,80, True)
        DISTANCE_TRAVELED = 0
        current_response = get_image()
        if(current_response == "L"):
            encode_to_stm("nq042")
            encode_to_stm("ne043")
            time.sleep(0.1)

            approach_obstacle_and_advance(45,100, True)

            encode_to_stm("ne015")
            current_response = get_image()
            if current_response == "L":
                encode_to_stm("nc015")
                task_LL()
                task_endR()
            elif current_response == "R":
                task_LR()
                task_endL()
            else:
                print("No L or R recognized, retrying")
                encode_to_stm("ne015")
                current_response = get_image()
                if current_response == "L":
                    encode_to_stm("nc015")
                    encode_to_stm("nc015")
                    task_LL()
                    task_endR()
                elif current_response == "R":
                    encode_to_stm("nc015")
                    task_LR()
                    task_endL()
                

        elif (current_response == "R"):
            encode_to_stm("ne042")
            encode_to_stm("nq043")
            time.sleep(0.1)
            approach_obstacle_and_advance(45,100, True)
            encode_to_stm("nq015")
            current_response = get_image()
            if current_response == "L":
                task_RL()
                task_endR()
                break
            elif current_response == "R":
                encode_to_stm("nz015")
                task_RR()
                task_endL()
                break
            else:
                print("No L or R recognized, retrying")
                encode_to_stm("nq015")
                current_response = get_image()
                if current_response == "L":
                    encode_to_stm("nz015")
                    task_RL()
                    task_endR()
                    break
                elif current_response == "R":
                    encode_to_stm("nz015")
                    encode_to_stm("nz015")
                    task_RR()
                    task_endL()
                    break
                
        else:
            print("No L or R recognized")

# test passed
def task_LL():
    encode_to_stm("nq043")
    encode_to_stm("nw020")
    encode_to_stm("ne133")
    encode_to_stm("nw040")
    encode_to_stm("ne089")


# test passed
def task_LR():
    encode_to_stm("ne072")
    encode_to_stm("nw001")
    encode_to_stm("nq177")
    encode_to_stm("nw047")
    encode_to_stm("nq090")

# test passed
def task_RL():
    encode_to_stm("nq072")
    encode_to_stm("nw001")
    encode_to_stm("ne180")
    encode_to_stm("nw038")
    encode_to_stm("ne088")

# test passed
def task_RR():
    encode_to_stm("ne043")
    encode_to_stm("nw015")
    encode_to_stm("nq132")
    encode_to_stm("nw047")
    encode_to_stm("nq088")


def task_endL():

    global DISTANCE_TRAVELED, TASK_FINISH
    distance_command = DISTANCE_TRAVELED + 32

    encode_to_stm("nw{:03}".format(int(distance_command)))
    DISTANCE_TRAVELED = 0

    encode_to_stm("nq043")
    encode_to_stm("nw040")
    encode_to_stm("ne045")  
    encode_to_stm("ns000")
    time.sleep(0.5) 


    approach_obstacle_and_advance(18, 60, False)
    TASK_FINISH = True


def task_endR():
    global DISTANCE_TRAVELED, TASK_FINISH
    distance_command = DISTANCE_TRAVELED + 38
    encode_to_stm("nw{:03}".format(int(distance_command)))
    DISTANCE_TRAVELED = 0

    encode_to_stm("ne043")
    encode_to_stm("nw033")
    encode_to_stm("nq047")
    encode_to_stm("ns000")
    time.sleep(1)

    approach_obstacle_and_advance(18, 60, False)
    TASK_FINISH = True


def get_image():
    global CV_RESPONSE
    print("Getting response from CV")
    response = capture_and_send()
    response = int(response)
    if response == 39:
        response = "L"
        print("CV Response is:" + str(response))
        CV_RESPONSE = response
    elif response == 38:
        response = "R"
        print("CV Response is:" + str(response))
        CV_RESPONSE = response
    else:
        print("CV Response is not L or R, received:" + str(response))
        response = None
    return response


def approach_obstacle_and_advance(target_distance, dash_distance, back_conpensate):
    global DISTANCE_TRAVELED, ULTRASOUND_DISTANCE

    desired_stop = False
    while not desired_stop:
        print("========== APPROACHING OBSTACLE ==========")
        ULTRASOUND_DISTANCE = get_distance()
        print("Ulltrasonic reads: " + str(ULTRASOUND_DISTANCE))
        time.sleep(0.006)
        if ULTRASOUND_DISTANCE >= dash_distance:  # it's safe to dash
            encode_to_stm("nw{:03}".format(dash_distance-10))
            DISTANCE_TRAVELED += dash_distance-10
        elif (
            ULTRASOUND_DISTANCE <= dash_distance
            and ULTRASOUND_DISTANCE > target_distance
        ):
            encode_to_stm("nw{:03}".format(round(ULTRASOUND_DISTANCE - target_distance),0))
            desired_stop = True
            DISTANCE_TRAVELED += round((ULTRASOUND_DISTANCE - target_distance),0)
        elif ULTRASOUND_DISTANCE < target_distance and ULTRASOUND_DISTANCE > 2:

            if(back_conpensate):
                encode_to_stm("nx{:03}".format(round(target_distance - ULTRASOUND_DISTANCE),0))
                DISTANCE_TRAVELED -= round((ULTRASOUND_DISTANCE - target_distance),0)
            desired_stop = True
            
        else:
            print("========== Unexpected Behaviour! ==========")
            encode_to_stm("ns000")


def main():
    try:
        print("Sending dummy command")
        encode_to_stm("ns000")
        print("========== ROBOT RESETED, WAITING FOR START ==========")
        movement_task()

    except KeyboardInterrupt:
        print("Stopped by User - Keyboard Interrupt")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()  # yeet.
