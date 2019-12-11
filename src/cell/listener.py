#!/usr/bin/env python3

import rospy
import socket
import numpy as np
# from latis_msgs.msg import raw_radar
from cell_estimator.msg import imu
from cell_estimator.msg import RelPos
from cell_estimator.msg import PositionVelocityTime
import time
import val

DEBUGGER = False

def printer(str1, str2):
	if DEBUGGER:
		print(str1, str2)


class udp_receiver():

    def __init__(self):
        # Set our class attributes (persistant variables)
        self.udp_port = 5555
        self.buffer_size = 2048
        self.BASE_FOUND = False

        # Define a socket listener to receive the radar packets
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #  Open UDP Socket
        self.sock.bind(('', self.udp_port))
        self.sock.setblocking(True)
        print('UDP socket', str(self.udp_port), ' opened.')

        # ROS stuff
        rospy.Subscriber('base/lla', PositionVelocityTime, self.base_lla_callback)
        self.pub_imu = rospy.Publisher('imu', imu, queue_size=1024) #I just used the queue size from latis
        self.pub_NED = rospy.Publisher('NED', RelPos, queue_size=1024)
        self.pub_lla = rospy.Publisher('lla', PositionVelocityTime, queue_size=1024)

    def main_loop(self):

        #  Read in Data
        # while True:
        imu_data = imu()
        NED_data = RelPos()
        lla_data = PositionVelocityTime()

        byteArray, addr = self.sock.recvfrom(self.buffer_size)

        info = str(byteArray)

        one = info.find(" 1, ")
        three = info.find(" 3, ")
        four = info.find(" 4, ")
        five = info.find(" 5, ")

        printer("Data:", info)

        if one != -1:
            gps = info[one:three-2].split(',')
            printer("data:", gps)
            lat = float(gps[1])
            lon = float(gps[2])
            alt = float(gps[3])
            printer("lat:", lat)
            printer("lon:", lon)
            printer("alt:", alt)
            lla_data.lla[0] = lat
            lla_data.lla[1] = lon
            lla_data.lla[2] = alt
            lla_data.header.stamp = rospy.Time.now()
 
            self.pub_lla.publish(lla_data)
            if self.BASE_FOUND:
                ned = self.GPS2NED(lat,lon,alt)
                NED_data.relPosNED[0] = ned[0]
                NED_data.relPosNED[1] = ned[1]
                NED_data.relPosNED[2] = ned[2]
                NED_data.header.stamp = rospy.Time.now()
                self.pub_NED.publish(NED_data)

        if three != -1:
        	accel = info[three:four-2].split(',')
        	printer("Accel:", accel)
        	x_accel = float(accel[1])
        	y_accel = float(accel[2])
        	z_accel = float(accel[3])
        	printer("x accel:", x_accel)
        	printer("y accel:", y_accel)
        	printer("z accel:", z_accel)
        	imu_data.linear_acceleration.x = x_accel
        	imu_data.linear_acceleration.y = y_accel
        	imu_data.linear_acceleration.z = z_accel

        if four != -1:
            gyro = info[four:five-2].split(',')
            printer("Gyro:", gyro)
            x_gyro = float(gyro[1])
            y_gyro = float(gyro[2])
            z_gyro = float(gyro[3])
            printer("x gyro:", x_gyro)
            printer("y gyro:", y_gyro)
            printer("z gyro:", z_gyro)
            imu_data.angular_velocity.x = x_gyro
            imu_data.angular_velocity.y = y_gyro
            imu_data.angular_velocity.z = z_gyro
            lla_data.header.stamp = rospy.Time.now()
            self.pub_imu.publish(imu_data)
        print("Done reading.")

    def base_lla_callback(self, msg):
        print('got lla callback')
        self.base_lat = msg.lla[0]
        self.base_lon = msg.lla[1]
        self.base_alt = msg.lla[2]
        self.BASE_FOUND = True

    def GPS2NED(self, lat, lon, alt):
        lat_deg = lat # Latitude
        lon_deg = lon # Longitude
        alt = alt # Altitude

        #send in lon (lambda, or longitude) as an angle in degrees North ex. 76.42816
        #send in lat (phi, or latitude) as an angle in degrees East ex.38.14626
        #both of those are converted into radians
        #send in alt as a altitude (mean sea level) provo = about 1500

        # lon_ref <-- rLam is the reference longitude (in RADIANS)
        # lat_ref <-- rPhi is the reference latitude (in RADIANS)
        # The reference angles define where the 0,0,0 point is in the local NED coordinates
    	# Keep track of the origin

        # reference longitude and latitude
        # lon_ref = -origin[0] * val.d2r    # lon <-- rLam
        # lat_ref = origin[1] * val.d2r     # lat <-- rPhi
        # r_h = origin[2]                        # range --> height? ... or radar height?
        lat_ref = self.base_lat * val.d2r
        lon_ref = -self.base_lon * val.d2r
        r_h = self.base_alt

        # Convert the angles into Earth Centered Earth Fixed Reference Frame

        # TO DO : check if x and y are correct
        chi = np.sqrt( 1 - val.e_frst_num_ecc * np.sin(lat_ref) * np.sin(lat_ref) )
        xr = ( val.e_smaj / chi + r_h ) * np.cos(lat_ref) * np.cos(lon_ref)
        yr = ( val.e_smaj / chi + r_h ) * np.cos(lat_ref) * np.sin(lon_ref)
        zr = ( val.e_smaj * (1 - val.e_frst_num_ecc) / chi + r_h ) * np.sin(lat_ref)


        chi = np.sqrt(1 - val.e_frst_num_ecc * np.sin(lat_ref)**2 )

        #Convert the incoming angles to radians
        lon = lon_deg * val.d2r         # typical longitude greek is lambda
        lat = lat_deg * val.d2r         # " " latitude greek is phi

        # Convert the angles into Earth Centered Earth Fixed Reference Frame
        x = ( val.e_smaj / chi + alt)* np.cos(lat)*np.cos(lon)
        y = ( val.e_smaj / chi + alt)* np.cos(lat)*np.sin(lon)
        z = ( val.e_smaj * (1 - val.e_frst_num_ecc) / chi + alt) * np.sin(lat)

        # Find the difference between the point x, y, z to the reference point in ECEF
        dx = x - xr
        dy = y - yr
        dz = z - zr

        ned = []
        # Rotate the point in ECEF to the Local NED
        N = (-np.sin(lat_ref)*np.cos(lon_ref)*dx) + (-np.sin(lat_ref)*np.sin(lon_ref)*dy) + np.cos(lat_ref)*dz
        E = (np.sin(lon_ref)*dx) - (np.cos(lon_ref)*dy)
        D = (-np.cos(lat_ref)*np.cos(lon_ref)*dx) + (-np.cos(lat_ref)*np.sin(lon_ref)*dy) + (-np.sin(lat_ref)*dz)

        ned = np.array([N,E,D])
        return ned

    # def run_receiver(self):
    #     self.main_loop()


if __name__ == '__main__':
    receiver = udp_receiver()
    receiver.main_loop()
