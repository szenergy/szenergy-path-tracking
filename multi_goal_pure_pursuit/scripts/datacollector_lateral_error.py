import rospy

from std_msgs.msg import Float64, Int32

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

import datetime
import csv

class LateralDataCollector():
    def cbLateralError(self, data):
        if (self.initialized):
            self.lat_error_list.append([rospy.Time.now().to_time(), data.data, self.pptype])

    def cbPpType(self, data):
        self.initialized = True
        print("PP TYPE: {0}".format(data.data))
        self.pptype = data.data

    def __init__(self):
        self.initialized = False
        self.lat_error_list = []

    def init(self):
        self.lat_error_pub = rospy.Subscriber("/trajectory_following/debug/lateral_deviation", Float64, self.cbLateralError)
        self.lat_pp_type = rospy.Subscriber("trajectory_following/debug/pp_type", Int32, self.cbPpType)

    def saveLog(self):
        with open('./data/{0}_short_multigoal.csv'.format(datetime.datetime.now().strftime("%Y_%B_%d_%H_%M_%S")), 'w') as f:
            spamwriter = csv.writer(f, delimiter=',')
            spamwriter.writerow(['time','lateral_error', 'pp_type'])
            for lat_error in self.lat_error_list:
                spamwriter.writerow(lat_error)
                

def main():
    rospy.init_node('datacollector_lateral_error')
    
    lat_dat_col = LateralDataCollector()
    lat_dat_col.init()
    rospy.spin()
    lat_dat_col.saveLog()
    


if __name__=="__main__":
    main()