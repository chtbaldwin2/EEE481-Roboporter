import serial
import time
import threading
from evdev import InputDevice, categorize, ecodes, KeyEvent
from rplidar import RPLidar, MAX_MOTOR_PWM, RPLidarException
from matplotlib import pyplot as plt
import numpy as np

# Global variable to store sensor values
sensor = 0

# Convert scans for plotting and data storage
def convert_scan_into_plot(scan):
    thetas = []
    dists = []
    for s in scan:
        angle = s[1]
        dist = s[2]
        thetas.append(angle_conversion(angle))
        dists.append(dist)
    
    return thetas, dists

# Convert angles to radians
def angle_conversion(angle):
    return np.radians(-angle)

# Main code: start lidar scans and plot the graph
def plotting_lidar(scans_generator, pols, fig, ax, axbackground):
    global sensor  # Accessing the global sensor0 variable
    thetas = []
    dists = []
    try:
        # Start lidar scanning
        scan = next(scans_generator)

        # Collected lidar data to be plotted
        thetas, dists = convert_scan_into_plot(scan)
        
        # Determine color based on sensor0 value
        color = 'g' if int(sensor) >= 40 else 'r'
        pols.set_markerfacecolor(color)
        pols.set_markeredgecolor(color)
        
        pols.set_data(thetas, dists)

        fig.canvas.restore_region(axbackground)
        ax.draw_artist(pols)
        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events() # Flush for next plot
        
    except RPLidarException:
        plt.close('all')
        print("Could not connect to lidar, please run again")
        
    return thetas, dists

# Function that reads the serial port
def serial_reader():
    global sensor  # Accessing the global sensor0 variable
    ultraArd.reset_input_buffer()
    while True:
        # Decodes the serial messages (gets rid of any tags or hidden characters)
        decode = ultraArd.readline().decode('utf-8').rstrip()
        sensor = decode[4:]

        # Determine which reading this is from (0,1,2,3)
        #if decode.startswith('%0'):
            #sensor0 = decode[4:]
           # print(sensor0)
            
        #elif decode.startswith('%1'):
            #sensor1 = decode[2:]
            #print(sensor1)
            
       # elif decode.startswith('%2'):
            #sensor2 = decode[2:]
            #print(sensor2)
            
        #elif decode.startswith('%3'):
            #sensor3 = decode[2:]
            #print(sensor3)
            
# Function to handle LiDAR plotting
def lidar_plotter():
    fig = plt.figure(facecolor='k')
    fig.canvas.manager.set_window_title('LiDAR RADAR')
    
    # Display specifications
    ax = fig.add_subplot(1,1,1, polar=True, facecolor='#000000')
    ax.tick_params(axis='both', colors='w')
    dist_max = 3000.0
    ax.set_ylim(0.0, dist_max)
    ax.set_xlim(0.0, 2*np.pi)
        
    # Display plot points
    pols, = ax.plot([], linestyle='', marker='o', markerfacecolor='g', markeredgecolor='g', markeredgewidth=1.0, markersize=3.0, alpha=1) # Green point plot

    # Plotting
    fig.canvas.draw()
    fig.show()
    fig.canvas.blit(ax.bbox)
    fig.canvas.flush_events()
    axbackground = fig.canvas.copy_from_bbox(ax.bbox)
    
    lidar = RPLidar('/dev/ttyUSB0')
    lidar.motor_speed = MAX_MOTOR_PWM
    
    scans = lidar.iter_scans(scan_type="express")
    while True:
        plotting_lidar(scans, pols, fig, ax, axbackground)
        lidar.clean_input()

    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    exit()

# Function to listen for gamepad input
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

                # 310 is the code for when the back left trigger is pressed, signaling for the motors to decelerate
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

if __name__ == '__main__':
    motorArd = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
    ultraArd = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    
    # Create threads for each function
    serial_thread = threading.Thread(target=serial_reader)
    gamepad_thread = threading.Thread(target=gamepad_listener)
    lidar_thread = threading.Thread(target=lidar_plotter)

    # Start threads
    serial_thread.start()
    gamepad_thread.start()
    lidar_thread.start()

    # Join threads
    serial_thread.join()
    gamepad_thread.join()
    lidar_thread.join()
