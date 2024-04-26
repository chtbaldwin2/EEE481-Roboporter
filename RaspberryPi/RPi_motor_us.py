import serial
import time
import threading
from evdev import InputDevice, categorize, ecodes, KeyEvent

# Initialise both Arduinos
# motorArd - this controls the motors
# ultraArd - this controls the US sensors
motorArd = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
ultraArd = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

# Function that reads the serial port
def serial_reader():
    ultraArd.reset_input_buffer()
    while True:

        # Decodes the serial messages (gets rid of any tags or hidden characters)
        decode = ultraArd.readline().decode('utf-8').rstrip()

        # Determine which reading this is from (0,1,2,3)
        if decode.startswith('%0'):
            sensor0 = decode[2:]
            print(sensor0)

            # If the sensor value is less than 40, don't let the Roboporter move forward
            if sensor0 < 40:
                motorArd.write(b"stopForward\n");
            
        elif decode.startswith('%1'):
            sensor1 = decode[2:]
            print(sensor1)

            # Don't let RP go left
            if sensor1 < 40:
                motorArd.write(b"stopLeft\n");
            
        elif decode.startswith('%2'):
            sensor2 = decode[2:]
            print(sensor2)

            #Don't let RP go right
            if sensor2 < 40:
                motorArd.write(b"stopRight\n");
            
        elif decode.startswith('%3'):
            sensor3 = decode[2:]
            print(sensor3)

            # Don't let RP go back
            if sensor3 < 40:
                motorArd.write(b"stopBack\n");
            
            

def gamepad_listener():
    
    # Gamepad is the controller input
    gamepad = InputDevice('/dev/input/event2')
    
    for event in gamepad.read_loop():

        # Part of the evdev/gamepad package
        if event.type == ecodes.EV_KEY:
            keyevent = categorize(event)

            # If a key is pressed down
            if keyevent.keystate == KeyEvent.key_down:

                # 305 is the code given when the "B" button is pressed, which is what we're using to go right
                if keyevent.scancode == 305:
                    motorArd.write(b"goRight\n")

                # 310 is the code for when the back left trigger is pressed, signalling for the motors to decelerate
                elif keyevent.scancode == 310:
                    motorArd.write(b"goBackward\n")

                # 311 - back right trigger - move forward
                elif keyevent.scancode == 311:
                    motorArd.write(b"goForward\n")

                # 306 - "Y" button - move left
                elif keyevent.scancode == 306:
                    motorArd.write(b"goLeft\n")

                # any other input - stop the motors
                else:
                    motorArd.write(b"stop\n")

# Main function - create threads for each of the functions
if __name__ == '__main__':
    serial_thread = threading.Thread(target=serial_reader)
    gamepad_thread = threading.Thread(target=gamepad_listener)

    serial_thread.start()
    gamepad_thread.start()

    serial_thread.join()
    gamepad_thread.join()
