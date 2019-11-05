import rospy

class Estimator():
    def imu_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        # rospy.loginfo(rospy.get_caller_id() + " orientation x %s", data.orientation.x)
        print("imu linear_acceleration:", data.linear_acceleration)
        print("imu angular_velocity:", data.angular_velocity)

    def ned_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        print("ned data:")#, data)

    def lla_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        print("lla data:", data.lla)
