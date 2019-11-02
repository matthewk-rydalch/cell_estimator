import rospy

class Estimator():
    def imu_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        # rospy.loginfo(rospy.get_caller_id() + " orientation x %s", data.orientation.x)
        # print("imu_callback \n")
        a=1

    def ned_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        # print("ned_callback \n")
        a=1

    def lla_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        # print("lla_callback \n")
        a=1
        
    def rover_RelPos_callback(self, data):
        #this line is simply to verify that messages are being subscribed to during development
        print("rover_RelPos_callback \n")