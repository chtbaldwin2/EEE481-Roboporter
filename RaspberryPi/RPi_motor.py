#!/usr/bin/env python3
import serial
import time

from evdev import InputDevice, categorize, ecodes, KeyEvent
gamepad = InputDevice('/dev/input/event2')
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

if __name__ == '__main__':

    ser.reset_input_buffer()

    for event in gamepad.read_loop():
        if event.type == ecodes.EV_KEY:
            keyevent = categorize(event)
            if (keyevent.keystate == KeyEvent.key_down):
                if keyevent.scancode == 305:
                    ser.write(b"goRight\n")
                    line = ser.readline().decode('utf-8').rstrip()
                    print(line)
                elif keyevent.scancode == 310:
                    ser.write(b"goBackward\n")
                    line = ser.readline().decode('utf-8').rstrip()
                    print(line)
                elif keyevent.scancode == 311:
                    ser.write(b"goForward\n")
                    line = ser.readline().decode('utf-8').rstrip()
                    print(line)
                elif keyevent.scancode == 306:
                    ser.write(b"goLeft\n")
                    line = ser.readline().decode('utf-8').rstrip()
                    print(line)
                else:
                    ser.write(b"stop\n")
                    line = ser.readline().decode('utf-8').rstrip()
                    print(line)
                    
                    
