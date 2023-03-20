import serial
import time
import imagezmq
import socket
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
FINISH = False

""" CV """
CV_IP = "tcp://192.168.1.21:5555"
# CV ADDRESSING CONFIGURATION
HOST = socket.gethostname()
DELAY = 2
sender = imagezmq.ImageSender(connect_to=CV_IP)


""" ULTRASONIC """
STOP_DISTANCE = 70
DISTANCE_TRAVELED = 0
ULTRASOUND_DISTANCE = 1000

""" pICAM """
# Configure picam
picam2 = Picamera2()
config = picam2.create_still_configuration(transform=Transform(hflip=True, vflip=True))
picam2.configure(config)
picam2.start()


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


# CV CAM FUNCTION
def capture_and_send():
    try:
        nparray = picam2.capture_array()
        response = sender.send_image(HOST, nparray)
        return response
    except Exception as e:
        print(e.__class__.__name__)


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
    global START, DISTANCE_TRAVELED, ULTRASOUND_DISTANCE

    """
    # RL Test code
    approach_obstacle_and_advance(25, 80)
    DISTANCE_TRAVELED = 0

    encode_to_stm("ne042")
    encode_to_stm("nq043")

    approach_obstacle_and_advance(40, 100)

    task_RL()
    task_endR()
    """
    """
    # RR Test code
    approach_obstacle_and_advance(25, 80)
    DISTANCE_TRAVELED = 0

    encode_to_stm("ne042")
    encode_to_stm("nq043")

    approach_obstacle_and_advance(40, 100)

    task_RR()
    task_endL()
    """

    
    # LL Test code
    approach_obstacle_and_advance(25, 80)
    DISTANCE_TRAVELED = 0

    encode_to_stm("nq042")
    encode_to_stm("ne043")

    approach_obstacle_and_advance(60, 100)
    encode_to_stm("nw020")

    task_LL()
    task_endR()
    

    """
    # LR Test code
    approach_obstacle_and_advance(25, 80)
    DISTANCE_TRAVELED = 0

    encode_to_stm("nq042")
    encode_to_stm("ne043")

    approach_obstacle_and_advance(40, 100)

    task_LR()
    task_endL()
    """




def task_LL():
    encode_to_stm("nq043")
    encode_to_stm("nw020")
    encode_to_stm("ne133")
    encode_to_stm("nw030")
    encode_to_stm("ne090")


def task_LR():
    encode_to_stm("ne087")
    encode_to_stm("nw007")
    encode_to_stm("nq180")
    encode_to_stm("nw045")
    encode_to_stm("nq085")


def task_RL():
    encode_to_stm("nq087")
    encode_to_stm("nw010")
    encode_to_stm("ne180")
    encode_to_stm("nw030")
    encode_to_stm("ne088")


def task_RR():
    encode_to_stm("ne043")
    encode_to_stm("nw020")
    encode_to_stm("nq133")
    encode_to_stm("nw045")
    encode_to_stm("nq085")


def task_endL():
    global DISTANCE_TRAVELED
    distance_command = DISTANCE_TRAVELED + 55

    encode_to_stm("nw{:03}".format(int(distance_command)))
    DISTANCE_TRAVELED = 0

    encode_to_stm("nq043")
    encode_to_stm("nw020")
    encode_to_stm("ne050")
    encode_to_stm("ns000")

    approach_obstacle_and_advance(20, 50)


def task_endR():
    global DISTANCE_TRAVELED
    distance_command = DISTANCE_TRAVELED + 55
    encode_to_stm("nw{:03}".format(int(distance_command)))
    DISTANCE_TRAVELED = 0

    encode_to_stm("ne043")
    encode_to_stm("nw011")
    encode_to_stm("nq050")
    encode_to_stm("ns000")

    approach_obstacle_and_advance(20, 50)


def get_image():
    print("Getting response from CV")
    byte_response = capture_and_send()
    response = int(byte_response.decode())
    if response == 39:
        response = "L"
        print("CV Response is:" + str(response))
    elif response == 38:
        response = "R"
        print("CV Response is:" + str(response))
    else:
        print("CV Response is not L or R, received:" + str(response))

    return response


def approach_obstacle_and_advance(target_distance, dash_distance):
    global DISTANCE_TRAVELED, ULTRASOUND_DISTANCE

    print("========== APPROACHING OBSTACLE ==========")

    desired_stop = False
    while not desired_stop:
        time.sleep(0.006)
        ULTRASOUND_DISTANCE = get_distance()
        print("Ulltrasonic reads: " + str(ULTRASOUND_DISTANCE))
        if ULTRASOUND_DISTANCE >= dash_distance:  # it's safe to dash
            encode_to_stm("nw{:03}".format(dash_distance-5))
            DISTANCE_TRAVELED += dash_distance-5
        elif (
            ULTRASOUND_DISTANCE <= dash_distance
            and ULTRASOUND_DISTANCE > target_distance
        ):
            encode_to_stm("nw{:03}".format(round(ULTRASOUND_DISTANCE - target_distance),0))
            desired_stop = True
            DISTANCE_TRAVELED += round((ULTRASOUND_DISTANCE - target_distance),0)

        elif ULTRASOUND_DISTANCE < target_distance and ULTRASOUND_DISTANCE > 2:

            encode_to_stm("nx{:03}".format(round(target_distance - ULTRASOUND_DISTANCE),0))
            desired_stop = True
            DISTANCE_TRAVELED -= round((ULTRASOUND_DISTANCE - target_distance),0)
        else:
            print("Unexpected Behaviour!")
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
