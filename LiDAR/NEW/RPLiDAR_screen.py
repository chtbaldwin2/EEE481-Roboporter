from rplidar import RPLidar, MAX_MOTOR_PWM
from matplotlib import pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import serial
import keyboard
import math
import logging

#sudo pip3 install xyz --break-system-packages

#display setup
fig = plt.figure(facecolor='k')
fig.canvas.toolbar.pack_forget()
fig.canvas.manager.set_window_title('LiDAR RADAR')

#display spec
ax = fig.add_subplot(1,1,1, polar=True, facecolor='#006b70')
ax.tick_params(axis='both', colors='w')
dist_max = 5000.0
angle_max = 361
ax.set_ylim(0.0,dist_max)
ax.set_xlim(0.0,2*np.pi)
ax.set_position([-0.05, -0.05, 1.1, 1.05])
ax.set_rticks(np.linspace(0.0, dist_max,5))
ax.set_thetagrids(np.linspace(0.0, 360, 9))

#display plot points
pols, = ax.plot([], linestyle = '', marker = 'o', markerfacecolor = 'r', markeredgecolor = 'w', markeredgewidth = 1.0, markersize = 3.0, alpha = 1)
line1, = ax.plot([], color = 'w',linewidth = 4.0)

#plotting
fig.canvas.draw()
fig.show()
fig.canvas.blit(ax.bbox)
fig.canvas.flush_events()
axbackground = fig.canvas.copy_from_bbox(ax.bbox)

lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

#ang1= np.zeros((360,1))
#dist= np.zeros((360,1))
#theta= np.zeros((360,1))
dists=np.ones((angle_max,))
#distlist = []
#thetalist = []


     #print(f"Pos: {pos}, distance {dist}s")
 #for c in range(16):
     #print(f"{c}: {ang[c]}");

#currently: given new datapoint, it is not stored to any lists. 

#need to do: given new datapoint, add it to a list that contains distance and angle.
#every time new point acquired, plot list with data point added to it. map correct lists to the plot

def angle_conversion (angle):
	if angle<90:
		return -angle*(np.pi/180)
	if angle<180:
		return -angle*(np.pi/180)
	if angle<270:
		return (360-angle)*(np.pi/180)
	else:
		return (360-angle)*(np.pi/180)

# radians conversion
theta = np.zeros(angle_max)
for angle in range(angle_max):
	theta[angle] = angle_conversion(angle)
lidar.motor_speed = MAX_MOTOR_PWM
for scan in lidar.iter_scans(max_buf_meas=False,min_len=100):
	#print("===================New spin data===================")
	
	for a in scan:
		#add to list
		dist = a[2]
		angle = a[1]
		
		dists[int(angle)] = dist
		#print(angle,dist)

		try:
			#print(theta,dists)
			#for k, _ in enumerate(dists):
			#	dists[k]=4900;
			
			#theta2 = np.linspace(0, 6, 50)
			#dists2 = np.linspace(0, 6000, 50)
			
			# populate theta and dists
			pols.set_data(theta,dists)
			fig.canvas.restore_region(axbackground)
			ax.draw_artist(pols)
			
			#radar sweep line
			line1.set_data(np.repeat(angle_conversion(angle), 2),
				np.linspace(0.0, dist_max, 2))
			ax.draw_artist(line1)
			#line1.set_data(theta, dists)
			#ax.draw_artist(line1)
			fig.canvas.blit(ax.bbox)
			fig.canvas.flush_events() #flush for next plot
			
			#if keyboard.is_pressed('q'):
			#	plt.close('all')
			#	print("User need to Quit the application")
			#	break
		
		except RPLidarException:
			plt.close('all')
			print('Keyboard Interrupt')
			
		
	lidar.clean_input()
	

lidar.stop()
lidar.stop_motor()
lidar.disconnect()
exit()
			

