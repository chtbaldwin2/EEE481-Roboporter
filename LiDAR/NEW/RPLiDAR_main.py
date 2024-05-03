from rplidar import RPLidar, MAX_MOTOR_PWM, RPLidarException
from matplotlib import pyplot as plt
import numpy as np

# convert scans for plotting, data store
def convert_scan_into_plot(scan):
	thetas = []
	dists = []
	for s in scan:
		angle = s[1]
		dist = s[2]
		thetas.append(angle_conversion(angle))
		dists.append(dist)
	
	return thetas, dists
	
# converts angles to radians
def angle_conversion(angle):
	if angle<90:
		return angle*(np.pi/180)
	if angle<180:
		return angle*(np.pi/180)
	if angle<270:
		return -(360-angle)*(np.pi/180)
	else:
		return -(360-angle)*(np.pi/180)

# main code. starts the lidar scans, and plots the graph. Uses angle_conversion and conver_scan_into_plot
def plotting_lidar(scans_generator, pols, fig, ax, axbackground):
	try:
		# lidar to start scanning
		scan = next(scans_generator)
		print(len(scan))

		# collected lidar data to be plotted
		thetas, dists = convert_scan_into_plot(scan)
		pols.set_data(thetas, dists)

		fig.canvas.restore_region(axbackground)
		ax.draw_artist(pols)
		fig.canvas.blit(ax.bbox)
		fig.canvas.flush_events() #flush for next plot

		#pols.set_markerfacecolor('r')
		
		#ax.draw_artist(pols2) #plot pols2 red for USSensor low values
		#fig.canvas.blit(ax.bbox)
		#fig.canvas.flush_events()
		
	except RPLidarException:
		plt.close('all')
		print("Could not connect to lidar, please run again")
		
	return thetas, dists


# initialisation for the code.
if __name__ == '__main__':
	#display setup
	fig = plt.figure(facecolor='k')
	fig.canvas.toolbar.pack_forget()
	fig.canvas.manager.set_window_title('LiDAR RADAR')
	
	#display spec
	ax = fig.add_subplot(1,1,1, polar=True, facecolor='#006b70')
	ax.tick_params(axis='both', colors='w')
	dist_max = 5000.0
	ax.set_ylim(0.0,dist_max)
	ax.set_xlim(0.0,2*np.pi)
	ax.set_position([-0.05, -0.05, 1.1, 1.05])
	ax.set_rticks(np.linspace(0.0, dist_max,5))
	ax.set_thetagrids(np.linspace(0.0, 360, 9))
		
	#display plot points
	pols, = ax.plot([], linestyle = '', marker = 'o', markerfacecolor = 'g', markeredgecolor = 'w', markeredgewidth = 1.0, markersize = 3.0, alpha = 1) #green point plot
	#pols2, = ax.plot([], linestyle = '', marker = 'o', markerfacecolor = 'r', markeredgecolor = 'w', markeredgewidth = 1.0, markersize = 3.0, alpha = 1) #red point plot

	#plotting
	fig.canvas.draw()
	fig.show()
	fig.canvas.blit(ax.bbox)
	fig.canvas.flush_events()
	axbackground = fig.canvas.copy_from_bbox(ax.bbox)
	
	lidar = RPLidar('/dev/ttyUSB0')
	lidar.motor_speed = MAX_MOTOR_PWM
	
	scans = lidar.iter_scans(scan_type="express")
	idx = 0
	while idx < 100:
		thetas, dists = plotting_lidar(scans, pols, fig, ax, axbackground)
		lidar.clean_input()
		idx += 1

	lidar.stop()
	lidar.stop_motor()
	lidar.disconnect()
	exit()

		
