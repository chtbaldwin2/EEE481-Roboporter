import serial
import time
import threading
from evdev import InputDevice, categorize, ecodes, KeyEvent

motorArd = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
ultraArd = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

def serial_reader():
    ultraArd.reset_input_buffer()
    while True:
        decode = ultraArd.readline().decode('utf-8').rstrip()
        if decode.startswith('%'):
            line = decode[1:]
            print(line)

def gamepad_listener():
    gamepad = InputDevice('/dev/input/event2')
    for event in gamepad.read_loop():
        if event.type == ecodes.EV_KEY:
            keyevent = categorize(event)
            if keyevent.keystate == KeyEvent.key_down:
                if keyevent.scancode == 305:
                    motorArd.write(b"goRight\n")
                elif keyevent.scancode == 310:
                    motorArd.write(b"goBackward\n")
                elif keyevent.scancode == 311:
                    motorArd.write(b"goForward\n")
                elif keyevent.scancode == 306:
                    motorArd.write(b"goLeft\n")
                else:
                    motorArd.write(b"stop\n")

if __name__ == '__main__':
    serial_thread = threading.Thread(target=serial_reader)
    gamepad_thread = threading.Thread(target=gamepad_listener)

    serial_thread.start()
    gamepad_thread.start()

    serial_thread.join()
    gamepad_thread.join()
