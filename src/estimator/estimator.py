import rospy

class Estimator():
    def imu_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        # rospy.loginfo(rospy.get_caller_id() + " orientation x %s", data.orientation.x)
        print("imu_callback \n")

    def ned_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        print("ned_callback \n")

    def lla_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        print("lla_callback \n")