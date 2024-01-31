# DumbLIDAR v1.0
# Unexpected Maker
#
# Based of variuous code examples from the internet
# Plus contributions from Mark Olsson, Michael Himing, and my YT chat

from serial import Serial
import time
from math import atan,pi,floor
import matplotlib.pyplot as plt
import numpy as np

class DumbLIDAR:
	def __init__(self, port, chunk_size=6000, no_value=0):
		"""Initialize the connection and set port and baudrate."""
		self._port = port
		self._baudrate = 115200
		self._chunk_size=chunk_size
		self._no_value = no_value
		self._is_connected = False
		self._is_scanning = False
		self._generate_image = True
		
	def Connect(self):
		"""Begin serial connection with Lidar by opening serial port.\nReturn success status True/False.\n"""
		if not self._is_connected:
			self._s = Serial(self._port, self._baudrate)

		if self._s.is_open:
			self._is_connected = True
			self._s.reset_input_buffer()
		else:
			self._is_connected = False
		
		return self._is_connected

	def Generate_PNG(self, data, out_name="lidar_output.png", show_axis=True):
		"""Generate a PNG of the scan data"""
		radius_data = np.array(data)
		theta = 2 * np.pi * np.arange(1, 0, -(1/360))
		area = 2
		fig = plt.figure()
		ax = fig.add_subplot(projection='polar')
		ax.set_rlim(0,3500)

		if not show_axis:
			ax.axis('off')

		c = ax.scatter(theta, radius_data, c=radius_data, s=area, cmap='hsv', alpha=0.75)

		plt.savefig(out_name)
		time.sleep(0.5)

			
	@classmethod    
	def _AngleCorr(cls,dist):
		if dist==0:
			return 0
		else:
			return (atan(21.8*((155.3-dist)/(155.3*dist)))*(180/pi))
	@classmethod  
	def _HexArrToDec(cls,data):
		littleEndianVal = 0
		for i in range(0,len(data)):
			littleEndianVal = littleEndianVal+(data[i]*(256**i))
		return littleEndianVal
	
	@classmethod
	def _Calculate(cls,d):
		ddict=[]
		LSN=d[1]
		Angle_fsa = ((DumbLIDAR._HexArrToDec((d[2],d[3]))>>1)/64.0)#+DumbLIDAR._AngleCorr(DumbLIDAR._HexArrToDec((d[8],d[9]))/4)
		Angle_lsa = ((DumbLIDAR._HexArrToDec((d[4],d[5]))>>1)/64.0)#+DumbLIDAR._AngleCorr(DumbLIDAR._HexArrToDec((d[LSN+6],d[LSN+7]))/4)
		if Angle_fsa<Angle_lsa:
			Angle_diff = Angle_lsa-Angle_fsa
		else:
			Angle_diff = 360+Angle_lsa-Angle_fsa
		for i in range(0,2*LSN,2):
			# Distance calculation
			dist_i = DumbLIDAR._HexArrToDec((d[8+i],d[8+i+1]))/4
			# Ignore zero values, they result in massive noise when
			# computing mean of distances for each angle.
			if dist_i == 0:
				continue
			# Intermediate angle solution
			Angle_i_tmp = ((Angle_diff/float(LSN-1))*(i/2))+Angle_fsa
			# Angle correction
			Angle_i_tmp += DumbLIDAR._AngleCorr(dist_i)
			if Angle_i_tmp > 360:
				Angle_i = Angle_i_tmp-360
			elif Angle_i_tmp < 0:
				Angle_i = Angle_i_tmp+360
			else:
				Angle_i = Angle_i_tmp
			ddict.append((dist_i,Angle_i))
		return ddict
	
	@classmethod
	def _CheckSum(cls,data):
		try:
			ocs = DumbLIDAR._HexArrToDec((data[6],data[7]))
			LSN = data[1]
			cs = 0x55AA^DumbLIDAR._HexArrToDec((data[0],data[1]))^DumbLIDAR._HexArrToDec((data[2],data[3]))^DumbLIDAR._HexArrToDec((data[4],data[5]))
			for i in range(0,2*LSN,2):
				cs = cs^DumbLIDAR._HexArrToDec((data[8+i],data[8+i+1])) 
			if(cs==ocs):
				return True
			else:
				return False
		except Exception as e:
			return False
	@classmethod
	def _Mean(cls,data):
		if(len(data)>0):
			return int(sum(data)/len(data))
		return 0
	
	def StartScanning(self):
		"""Begin the lidar and returns a generator which returns a dictionary consisting angle(degrees) and distance(meters).\nReturn Format : {angle(1):distance, angle(2):distance,....................,angle(360):distance}."""
		if self._is_connected:
			self._is_scanning = True
			self._s.reset_input_buffer()
			time.sleep(0.5)
			self._s.read(7)
			distdict = {}
			countdict = {}
			lastChunk = None
			while self._is_scanning:
				for i in range(0,360):
					distdict.update({i:[]})
	 
				data = self._s.read(self._chunk_size).split(b"\xaa\x55")
				if lastChunk is not None:
					data[0] = lastChunk + data[0]
				lastChunk = data.pop()
	
				for e in data:
					try:
						if(e[0]==0):
							if(DumbLIDAR._CheckSum(e)):
								d = DumbLIDAR._Calculate(e)
								for ele in d:
									angle = floor(ele[1])
									if(angle>=0 and angle<360):
										distdict[angle].append(ele[0])
					except Exception as e:
						pass
				for i in distdict.keys():
					if len(distdict[i]) > 0:
						distdict[i]=self._Mean(distdict[i])
					else:
						distdict[i]=self._no_value
				yield distdict  
		else:
			raise Exception("Device is not connected")
			
	def StopScanning(self):
		"""Stops scanning but keeps serial connection alive."""
		if self._is_connected:
			if self._is_scanning:
				self._is_scanning = False
				self._s.reset_input_buffer()
		else:
			raise Exception("Device is not connected")
		
		
	def Disconnect(self):
		"""Stop scanning and close serial communication with Lidar."""
		if self._is_connected:
			if self._is_scanning:
				self.StopScanning()
			self._s.close()
			self._is_connected = False
		else:
			raise Exception("Device is not connected")
			
   
port = "/dev/ttyUSB0"
lidar = DumbLIDAR(port)

# Open the Serial port, and if connected, start scanning
if lidar.Connect():

	# Scan is a generator - every iteration is a full 360 degrees
	scan = lidar.StartScanning()
	# timer for checking for 30 second scan time
	t = time.time() 
	while (time.time() - t) < 30:
		
		start_time = time.time()
		data = next(scan)
		data = [[data[i]] for i in range(360)]

		if lidar._generate_image:
			lidar.Generate_PNG(data)
			
		print(f"Scan Time: {round(time.time()-start_time, 2)}s")
			
		
	lidar.StopScanning()
	lidar.Disconnect()
else:
	print("Error connecting to device")
	