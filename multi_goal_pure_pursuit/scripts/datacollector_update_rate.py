import rospy
from std_msgs.msg import Float64

class TimeOfCalcDataCollector():
    def cbTimeOfCalc(self, data):
        if (not self.initialized):
            self.avg_time_of_calc = data.data
            self.max_time_of_calc = data.data
            self.initialized = True
        else:
            self.avg_time_of_calc = (self.avg_time_of_calc + data.data)/2.0
            self.max_time_of_calc = max(data.data, self.max_time_of_calc)
    
    def cbTimer(self, timer_event):
        print("Avg time of calc: {0}\t Max time of calc: {1}".format(self.avg_time_of_calc, self.max_time_of_calc))

    def __init__(self):
        self.initialized = False
        self.avg_time_of_calc = 0.0
        self.calc_time_timer = rospy.Timer(rospy.Duration(5.0), self.cbTimer)

    def init(self):
        self.time_of_calc = rospy.Subscriber("/trajectory_following/debug/time_of_calculation", 
            Float64, self.cbTimeOfCalc)

def main():
    rospy.init_node('datacollector_lateral_error')
    time_of_calc_data = TimeOfCalcDataCollector()
    time_of_calc_data.init()
    rospy.spin()

if __name__=="__main__":
    main()